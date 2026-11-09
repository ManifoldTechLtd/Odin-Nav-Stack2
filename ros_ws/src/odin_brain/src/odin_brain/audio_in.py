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

"""Microphone audio capture with lightweight VAD, half-duplex self-hearing suppression, and remote ASR client.

Pipeline:
    arecord(S16_LE/16k/mono) -> frame RMS energy VAD -> silence tail ->
    WAV/Opus encoding -> POST scripts/asr_server.py -> text -> EventBus / callback
"""

import array
import base64
import io
import json
import math
import os
import re
import subprocess
import tempfile
import threading
import time
import urllib.request
import wave
from typing import Callable, Optional

# Default wake words
WAKE_WORDS_DEFAULT = ("奥丁", "奥丁奥丁", "丁子", "哈吉丁")

# Homophone variants for wake-word matching
_WAKE_VARIANTS = (
    "奥亭", "奥顶", "奥叮", "熬丁", "奥汀", "丁字", "顶子",
    "浩婷", "奥迪", "奥帝", "皓婷", "浩庭", "豪庭", "奥庭",
)

# Short filler words ignored outside ask_and_wait listening windows
_NOISE_WORDS = frozenset((
    "嗯", "啊", "哦", "呃", "唉", "诶", "欸", "呵", "哈", "嗷",
    "那个", "这个", "就是", "然后", "那个那个", "嗯嗯", "啊啊",
))


def _is_noise(text: str) -> bool:
    """Return True if text is a filler interjection or non-command single character."""
    t = (text or "").strip()
    if not t:
        return True
    if t in _NOISE_WORDS:
        return True
    if len(t) == 1 and t not in "去停走回起转进退左右前后中":
        return True
    return False


def _pinyin_lower(text: str) -> str:
    """Convert Chinese characters to lowercase tone-free pinyin (empty string if pypinyin is unavailable)."""
    try:
        from pypinyin import lazy_pinyin
    except Exception:
        return ""
    try:
        return "".join(lazy_pinyin(text)).lower()
    except Exception:
        return ""


def _edit_distance_le(a: str, b: str, limit: int = 1) -> int:
    """Bounded Levenshtein distance (returns limit + 1 if distance exceeds limit)."""
    if a == b:
        return 0
    if abs(len(a) - len(b)) > limit:
        return limit + 1
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        if min(cur) > limit:
            return limit + 1
        prev = cur
    return prev[-1]


def extract_wake_command(text: str, wake_words=None):
    """Check whether text starts with a wake word.

    Returns (is_woken, matched_wake_word, remaining_command) using:
      1) Exact prefix match (longest first, including homophone variants);
      2) Pinyin prefix match (exact or edit distance <= 1, or prefix of length >= 4).
    """
    words = list(wake_words) if wake_words else list(WAKE_WORDS_DEFAULT)
    cleaned = re.sub(r'[\s，,。！!？?、；;：:…\-\"]+', "", text or "")
    if not cleaned:
        return False, "", ""

    ranked = sorted(set(words), key=len, reverse=True)

    def _tail_after(index: int, word: str) -> str:
        """Extract command suffix after the wake word and strip repeated trailing characters."""
        rest = cleaned[index:]
        while rest and rest[0] == word[-1]:
            rest = rest[1:]
        return rest.strip()

    # 1) Exact match
    candidates = sorted(set(ranked + list(_WAKE_VARIANTS)), key=len, reverse=True)
    for word in candidates:
        if cleaned.startswith(word):
            return True, word, _tail_after(len(word), word)

    # 2) Pinyin match
    for word in ranked:
        n = len(word)
        if len(cleaned) < n:
            continue
        want = _pinyin_lower(word)
        got = _pinyin_lower(cleaned[:n])
        if not want or not got:
            continue
        if got == want or _edit_distance_le(got, want, 1) <= 1:
            return True, word, _tail_after(n, word)
        if len(got) >= 4 and want.startswith(got):
            return True, word, _tail_after(n, word)

    return False, "", cleaned


class AudioListener(object):
    def __init__(self, asr_url: str = "", store=None, speaker=None, event_bus=None,
                 on_text: Optional[Callable[[str], None]] = None,
                 device: str = "default", sample_rate: int = 16000,
                 frame_ms: int = 30, threshold: int = 500,
                 silence_ms: int = 900, min_speech_ms: int = 300,
                 max_speech_ms: int = 15000, verbose: bool = True,
                 asr_timeout: float = 30.0, wake_words=None, wake_enabled: bool = True,
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
        try:
            self.max_speech_ms = int(os.environ.get("ODIN_MIC_MAX_SPEECH_MS",
                                                    str(self.max_speech_ms)))
        except ValueError:
            pass
        self.verbose = verbose
        self.asr_timeout = float(asr_timeout)
        env_wake = os.environ.get("ODIN_WAKE_WORDS", "").strip()
        if env_wake:
            self.wake_words = [w.strip() for w in re.split(r"[,，\s]+", env_wake) if w.strip()]
        else:
            self.wake_words = list(wake_words) if wake_words else list(WAKE_WORDS_DEFAULT)
        self.wake_enabled = bool(wake_enabled) and \
            os.environ.get("ODIN_WAKE_ENABLE", "1").strip().lower() not in ("0", "false", "off", "no")
        # Continuous conversation wake-free window (seconds, 0=disabled)
        try:
            self.wake_window = float(os.environ.get("ODIN_WAKE_WINDOW", "30") or 0)
        except ValueError:
            self.wake_window = 0.0
        try:
            self.wake_extend_sec = float(os.environ.get("ODIN_WAKE_EXTEND_SEC", "15") or 15)
        except ValueError:
            self.wake_extend_sec = 15.0
        self._wake_until = 0.0
        self.on_wake = on_wake

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._last_err_ts = 0.0
        self._frame_bytes = int(self.sample_rate * self.frame_ms / 1000) * 2  # 16-bit PCM

    @property
    def enabled(self) -> bool:
        return bool(self.asr_url) and self.event_bus is not None

    def start(self) -> bool:
        if not self.enabled:
            if self.verbose:
                print("[mic ] ODIN_ASR_URL not configured, voice input disabled")
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
            print("[mic ] Failed to start arecord (%s), voice input disabled" % exc)
            return

        if self.verbose:
            print("[mic ] Microphone active (%s, device=%s, threshold=%d, silence=%dms)"
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

            # Half-duplex suppression: drop frames while TTS is speaking
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

    def extend_wake_window(self) -> None:
        """Extend the wake-free window after task/TTS completion."""
        if self.wake_window <= 0:
            return
        span = min(self.wake_extend_sec, self.wake_window)
        self._wake_until = time.time() + span

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
                    print("[mic ] Ignored (not woken): %s" % text)
                return
            if self.verbose:
                print("[mic ] Wake word '%s': %s" % (word, command or "(no command)"))
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
            print("[mic ] Recognized: %s" % text)

        if self.wake_window > 0 and not listening:
            self._wake_until = now + self.wake_window

        if not listening and _is_noise(text):
            if self.verbose:
                print("[mic ] Ignored noise: %s" % text)
            return

        if listening:
            self.event_bus.publish("ASR", {"text": text})
            return
        if self.verbose:
            print("[mic ] Voice command: %s" % text)
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
        started = time.time()
        wav_bytes = _pcm_to_wav(pcm, self.sample_rate)
        payload = {
            "audio_b64": base64.b64encode(wav_bytes).decode("ascii"),
            "hotwords": self.store.hotwords() if self.store is not None else [],
        }
        body_bytes = json.dumps(payload, ensure_ascii=False).encode("utf-8")

        opus = _to_opus(wav_bytes)
        if opus:
            body_bytes = opus
            hot_json = json.dumps(payload["hotwords"], ensure_ascii=False).encode("utf-8")
            headers = {"Content-Type": "audio/opus",
                       "X-Hotwords-B64": base64.b64encode(hot_json).decode("ascii")}
        else:
            headers = {"Content-Type": "application/json"}

        for attempt in (1, 2):
            request = urllib.request.Request(
                self.asr_url, data=body_bytes, headers=headers, method="POST")
            try:
                with urllib.request.urlopen(request, timeout=self.asr_timeout) as response:
                    body = json.loads(response.read().decode("utf-8", "replace") or "{}")
                text = str(body.get("text", "")).strip()
                if self.verbose:
                    audio_sec = len(pcm) / float(self.sample_rate * 2)
                    print("[mic ] ASR %.2fs (audio %.1fs, %.1fKB %s, server %sms)" % (
                        time.time() - started, audio_sec, len(body_bytes) / 1024.0,
                        "opus" if opus else "wav+b64",
                        body.get("latency_ms", "-")))
                return text
            except Exception as exc:
                if attempt == 2:
                    self._note_error("ASR request failed (retried): %s (%.0fKB, %s)"
                                     % (exc, len(body_bytes) / 1024.0,
                                        "opus" if opus else "wav+base64"))
        return ""

    def _note_error(self, message: str) -> None:
        """Throttle error messages to at most once every 10 seconds."""
        now = time.time()
        if now - self._last_err_ts < 10.0:
            return
        self._last_err_ts = now
        if self.verbose:
            print("[mic ] %s" % message)


def _to_opus(wav_bytes: bytes, bitrate: int = 32) -> bytes:
    """Compress WAV bytes to Opus using opusenc if available."""
    encoder = _which("opusenc")
    if not encoder or not wav_bytes:
        return b""
    src_path = dst_path = ""
    try:
        src = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        src_path = src.name
        src.write(wav_bytes)
        src.close()
        dst_path = src_path + ".opus"
        subprocess.run([encoder, "--quiet", "--bitrate", str(bitrate), src_path, dst_path],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
        with open(dst_path, "rb") as handle:
            return handle.read()
    except Exception:
        return b""
    finally:
        for path in (src_path, dst_path):
            if not path:
                continue
            try:
                os.unlink(path)
            except OSError:
                pass


def _rms(frame: bytes) -> float:
    """Compute RMS energy of 16-bit mono PCM samples."""
    samples = array.array("h")
    samples.frombytes(frame[: len(frame) - (len(frame) % 2)])
    if not samples:
        return 0.0
    total = 0
    for value in samples:
        total += value * value
    return math.sqrt(total / float(len(samples)))


def _which(name: str) -> str:
    for path in os.environ.get("PATH", "/usr/bin:/bin").split(":"):
        candidate = os.path.join(path, name)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return ""


def _pcm_to_wav(pcm: bytes, sample_rate: int) -> bytes:
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(int(sample_rate))
        handle.writeframes(pcm)
    return buffer.getvalue()
