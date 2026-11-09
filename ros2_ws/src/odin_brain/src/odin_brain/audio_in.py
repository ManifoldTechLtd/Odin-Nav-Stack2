# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Microphone audio capture with RMS VAD, half-duplex self-listening suppression, and remote ASR client.

Pipeline:
    arecord (S16_LE / 16kHz / mono) -> frame RMS energy detection -> silence tail ->
    WAV packaging -> HTTP POST to scripts/asr_server.py -> text -> EventBus / on_text callback
"""

import array
import base64
import io
import json
import math
import os
import re
import subprocess
import threading
import time
import urllib.request
import wave
from typing import Callable, Optional

# Wake words required before accepting voice commands outside an active dialog window
WAKE_WORDS_DEFAULT = ("奥丁", "奥丁奥丁", "丁子", "哈吉丁")

# Common homophone variants from ASR
_WAKE_VARIANTS = ("奥亭", "奥顶", "奥叮", "熬丁", "奥汀", "丁字", "顶子")


def extract_wake_command(text: str, wake_words=None):
    """Check whether `text` starts with a wake word.

    Returns (hit, matched_wake_word, remaining_command), matching longer wake words first.
    """
    words = list(wake_words) if wake_words else list(WAKE_WORDS_DEFAULT)
    cleaned = re.sub(r'[\s，,。！!？?、；;：:…\-\"]+', "", text or "")
    if not cleaned:
        return False, "", ""
    candidates = sorted(set(words + list(_WAKE_VARIANTS)), key=len, reverse=True)
    for word in candidates:
        if cleaned.startswith(word):
            return True, word, cleaned[len(word):]
    return False, "", cleaned


class AudioListener(object):
    def __init__(self, asr_url: str = "", store=None, speaker=None, event_bus=None,
                 on_text: Optional[Callable[[str], None]] = None,
                 device: str = "default", sample_rate: int = 16000,
                 frame_ms: int = 30, threshold: int = 500,
                 silence_ms: int = 900, min_speech_ms: int = 300,
                 max_speech_ms: int = 10000, verbose: bool = True,
                 asr_timeout: float = 15.0, wake_words=None, wake_enabled: bool = True,
                 on_wake: Optional[Callable[[str], None]] = None):
        self.asr_url = (asr_url or os.environ.get("ODIN_ASR_URL", "")).strip()
        self.store = store
        self.speaker = speaker
        self.event_bus = event_bus
        self.on_text = on_text
        self.device = (device or "").strip() or os.environ.get("ODIN_MIC_DEVICE", "default").strip() or "default"
        self.sample_rate = int(sample_rate)
        self.frame_ms = int(frame_ms)
        self.threshold = int(threshold)
        self.silence_ms = int(silence_ms)
        try:
            self.silence_ms = int(os.environ.get("ODIN_MIC_SILENCE_MS", str(self.silence_ms)))
        except ValueError:
            pass
        self.min_speech_ms = int(min_speech_ms)
        self.max_speech_ms = int(max_speech_ms)
        self.verbose = verbose
        self.asr_timeout = float(asr_timeout)
        env_wake = os.environ.get("ODIN_WAKE_WORDS", "").strip()
        if env_wake:
            self.wake_words = [w.strip() for w in re.split(r"[,，\s]+", env_wake) if w.strip()]
        else:
            self.wake_words = list(wake_words) if wake_words else list(WAKE_WORDS_DEFAULT)
        self.wake_enabled = bool(wake_enabled) and \
            os.environ.get("ODIN_WAKE_ENABLE", "1").strip().lower() not in ("0", "false", "off", "no")
        try:
            self.wake_window = float(os.environ.get("ODIN_WAKE_WINDOW", "0") or 0)
        except ValueError:
            self.wake_window = 0.0
        self._wake_until = 0.0
        self.on_wake = on_wake

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._frame_bytes = int(self.sample_rate * self.frame_ms / 1000) * 2  # 16-bit

    @property
    def enabled(self) -> bool:
        return bool(self.asr_url) and self.event_bus is not None

    def start(self) -> bool:
        if not self.enabled:
            if self.verbose:
                print("[mic ] 未配置 ODIN_ASR_URL，语音输入关闭")
            return False
        if self._thread is not None and self._thread.is_alive():
            return True
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="odin-mic")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            try:
                self._proc.terminate()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        try:
            self._proc = subprocess.Popen(
                ["arecord", "-q", "-D", self.device, "-f", "S16_LE",
                 "-r", str(self.sample_rate), "-c", "1", "-t", "raw"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except Exception as exc:
            print("[mic ] 启动 arecord 失败（%s），语音输入关闭" % exc)
            return

        if self.verbose:
            print("[mic ] 麦克风已开（%s，设备 %s，阈值 %d，静音收尾 %dms）"
                  % (self.asr_url, self.device, self.threshold, self.silence_ms))

        speech = bytearray()
        silence_frames = 0
        speech_frames = 0
        max_silence = max(1, int(self.silence_ms / self.frame_ms))
        min_speech = max(1, int(self.min_speech_ms / self.frame_ms))
        max_frames = max(1, int(self.max_speech_ms / self.frame_ms))

        while not self._stop.is_set():
            frame = self._proc.stdout.read(self._frame_bytes)
            if not frame or len(frame) < self._frame_bytes:
                time.sleep(0.05)
                continue

            # Half-duplex: discard frames while TTS is speaking to avoid self-triggering
            if self.speaker is not None and getattr(self.speaker, "is_speaking", False):
                speech = bytearray()
                silence_frames = 0
                speech_frames = 0
                continue

            loud = _rms(frame) >= self.threshold
            if loud:
                speech.extend(frame)
                speech_frames += 1
                silence_frames = 0
            elif speech_frames > 0:
                speech.extend(frame)
                silence_frames += 1

            if speech_frames >= max_frames:
                self._finish(bytes(speech), speech_frames)
                speech = bytearray()
                silence_frames = 0
                speech_frames = 0
                continue

            if speech_frames >= min_speech and silence_frames >= max_silence:
                self._finish(bytes(speech), speech_frames)
                speech = bytearray()
                silence_frames = 0
                speech_frames = 0

        try:
            if self._proc is not None:
                self._proc.terminate()
        except Exception:
            pass

    def _finish(self, pcm: bytes, frames: int) -> None:
        if not pcm or frames < 1:
            return
        text = self._transcribe(pcm)
        if not text:
            return

        listening = bool(self.event_bus and self.event_bus.is_listening())
        now = time.time()
        window_active = self.wake_window > 0 and now < self._wake_until

        if self.wake_enabled and not listening and not window_active:
            hit, word, command = extract_wake_command(text, self.wake_words)
            if not hit:
                if self.verbose:
                    print("[mic ] 未唤醒，忽略：%s" % text)
                return
            if self.verbose:
                print("[mic ] 已唤醒（%s）：%s" % (word, command or "（未带指令）"))
            if self.wake_window > 0:
                self._wake_until = now + self.wake_window
            if not command:
                if word == "哈吉丁":
                    if self.on_text is not None:
                        self.on_text(text)
                elif self.on_wake is not None:
                    self.on_wake(word)
                return
            text = command
        elif self.verbose:
            print("[mic ] 识别：%s" % text)

        if listening:
            self.event_bus.publish("ASR", {"text": text})
            return
        if self.verbose:
            print("[mic ] 语音命令：%s" % text)
        if self.on_text is not None:
            try:
                self.on_text(text)
                return
            except Exception:
                pass
        if self.event_bus is not None:
            self.event_bus.publish("ASR", {"text": text})

    def _transcribe(self, pcm: bytes) -> str:
        if not self.asr_url:
            return ""
        wav_bytes = _pcm_to_wav(pcm, self.sample_rate)
        payload = {
            "audio_b64": base64.b64encode(wav_bytes).decode("ascii"),
            "hotwords": self.store.hotwords() if self.store is not None else [],
        }
        request = urllib.request.Request(
            self.asr_url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.asr_timeout) as response:
                body = json.loads(response.read().decode("utf-8", "replace") or "{}")
        except Exception as exc:
            if self.verbose:
                print("[mic ] ASR 请求失败：%s" % exc)
            return ""
        return str(body.get("text", "")).strip()


def _rms(frame: bytes) -> float:
    """Compute root-mean-square energy of 16-bit mono PCM samples."""
    samples = array.array("h")
    samples.frombytes(frame[: len(frame) - (len(frame) % 2)])
    if not samples:
        return 0.0
    total = 0
    for value in samples:
        total += value * value
    return math.sqrt(total / float(len(samples)))


def _pcm_to_wav(pcm: bytes, sample_rate: int) -> bytes:
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(int(sample_rate))
        handle.writeframes(pcm)
    return buffer.getvalue()
