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

"""Text-to-speech (TTS) client with multi-backend support (edge-tts, espeak-ng, cloud) and automatic fallback."""

import json
import os
import queue
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request
from typing import Optional


class Speaker(object):
    def __init__(self, url: str = "", enabled: bool = True, timeout_sec: float = 20.0,
                 player: str = "paplay", max_queue: int = 4):
        self.url = (url or os.environ.get("ODIN_TTS_URL", "")).strip()
        backend = os.environ.get("ODIN_TTS_BACKEND", "auto").strip().lower()
        env_enable = os.environ.get("ODIN_TTS_ENABLE", "1").strip().lower()
        if env_enable in ("0", "false", "off", "no"):
            backend = "off"
        self.backend = backend
        self.voice = os.environ.get("ODIN_TTS_VOICE", "").strip()
        self.espeak_voice = os.environ.get("ODIN_ESPEAK_VOICE", "cmn").strip()
        self.espeak_speed = os.environ.get("ODIN_ESPEAK_SPEED", "155").strip()
        try:
            self.volume = float(os.environ.get("ODIN_TTS_VOLUME", "0.5"))
        except ValueError:
            self.volume = 0.5
        if self.volume > 1.0:
            self.volume = self.volume / 100.0
        self.volume = max(0.0, min(1.0, self.volume))
        self.cloud_url = os.environ.get("ODIN_TTS_CLOUD_URL", "").strip()
        self.cloud_key = os.environ.get("ODIN_TTS_CLOUD_KEY", "").strip()
        self.cloud_voice = os.environ.get("ODIN_TTS_CLOUD_VOICE", "").strip()
        self.timeout_sec = float(timeout_sec)
        self.player = player
        self.has_espeak = _which("espeak-ng") or _which("espeak")

        self.enabled = bool(enabled) and self._resolve_backend()
        self._queue = queue.Queue(maxsize=max(1, max_queue))
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._stop_flag = threading.Event()
        self._worker = None
        self._playing_flag = False
        self._last_play_end_ts = 0.0
        if self.enabled:
            self._worker = threading.Thread(target=self._loop, daemon=True)
            self._worker.start()

    def describe(self) -> str:
        if not self.enabled:
            return "关闭（backend=%s）" % self.backend
        vol = "%d%%" % int(self.volume * 100)
        if self.backend == "edge":
            return "edge：%s（voice=%s，音量 %s）" % (self.url, self.voice or "默认", vol)
        if self.backend == "espeak":
            return "espeak（本机，%s，语速 %s，音量 %s）" % (
                self.espeak_voice, self.espeak_speed, vol)
        if self.backend == "cloud":
            return "cloud：%s（voice=%s，音量 %s）" % (
                self.cloud_url or "未配置 URL", self.cloud_voice or "默认", vol)
        return self.backend

    def set_volume(self, value: float) -> float:
        """Set playback volume in [0.0, 1.0] (or [0, 100] as percentage)."""
        value = float(value)
        if value > 1.0:
            value = value / 100.0
        self.volume = max(0.0, min(1.0, value))
        return self.volume

    def speak(self, text: str) -> bool:
        text = (text or "").strip()
        if not self.enabled or not text:
            return False
        try:
            self._queue.put_nowait(text)
            return True
        except queue.Full:
            return False

    def stop(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                except Exception:
                    pass
            self._proc = None
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def wait_idle(self, timeout_sec: float = 20.0) -> None:
        if not self.enabled:
            return
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self._lock:
                playing = self._proc is not None and self._proc.poll() is None
            if self._queue.empty() and not playing:
                return
            time.sleep(0.1)

    def _resolve_backend(self) -> bool:
        if self.backend in ("off", "none", ""):
            return False
        if self.backend == "edge":
            return bool(self.url)
        if self.backend == "espeak":
            return bool(self.has_espeak)
        if self.backend == "cloud":
            return bool(self.cloud_url and self.cloud_key)
        if self.url:
            self.backend = "edge"
            return True
        if self.has_espeak:
            self.backend = "espeak"
            return True
        return False

    def _loop(self) -> None:
        while not self._stop_flag.is_set():
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._play(text)
            except Exception:
                pass

    def _play(self, text: str) -> None:
        """Synthesize and play audio using the active backend, falling back from edge to espeak on failure."""
        if self.backend == "edge":
            data = self._fetch_edge(text)
            if data:
                self._play_bytes(data)
                return
            if self.has_espeak:
                print("[tts ] edge 不可用，降级到 espeak")
                self._play_espeak(text)
            return
        if self.backend == "espeak":
            self._play_espeak(text)
            return
        if self.backend == "cloud":
            self._play_cloud(text)

    def _play_bytes(self, data: bytes) -> None:
        handle = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        path = handle.name
        try:
            handle.write(data)
            handle.close()
            self._play_file(path)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

    def _play_espeak(self, text: str) -> None:
        binary = _which("espeak-ng") or _which("espeak")
        if not binary:
            return
        handle = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        path = handle.name
        handle.close()
        try:
            subprocess.run([binary, "-v", self.espeak_voice, "-s", self.espeak_speed,
                            "-w", path, text],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=20)
            self._play_file(path)
        except Exception as exc:
            print("[tts ] espeak 失败：%s" % exc)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

    def _play_cloud(self, text: str) -> None:
        """Cloud TTS HTTP POST client."""
        if not (self.cloud_url and self.cloud_key):
            return
        payload = {"text": text}
        if self.cloud_voice:
            payload["voice"] = self.cloud_voice
        request = urllib.request.Request(
            self.cloud_url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + self.cloud_key},
            method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                self._play_bytes(response.read())
        except Exception as exc:
            print("[tts ] cloud 失败：%s" % exc)

    def _play_file(self, path: str) -> None:
        volume = int(max(0.0, min(1.0, self.volume)) * 65536)
        cmd = [self.player, "--volume=%d" % volume, path]
        with self._lock:
            try:
                self._proc = subprocess.Popen(cmd,
                                              stdout=subprocess.DEVNULL,
                                              stderr=subprocess.DEVNULL)
            except Exception:
                self._proc = subprocess.Popen([self.player, path],
                                              stdout=subprocess.DEVNULL,
                                              stderr=subprocess.DEVNULL)
            proc = self._proc
            self._playing_flag = True
        try:
            proc.wait()
        finally:
            with self._lock:
                self._playing_flag = False
                self._last_play_end_ts = time.time()
                if self._proc is proc:
                    self._proc = None

    @property
    def is_speaking(self) -> bool:
        """True while audio is playing or within the 250ms post-playback echo suppression window."""
        if not self.enabled:
            return False
        if self._playing_flag:
            return True
        return (time.time() - self._last_play_end_ts) < 0.25

    def _fetch_edge(self, text: str) -> Optional[bytes]:
        if not self.url:
            return None
        payload = {"text": text}
        if self.voice:
            payload["voice"] = self.voice
        request = urllib.request.Request(
            self.url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return response.read()
        except urllib.error.HTTPError as exc:
            print("[tts ] edge HTTP %s" % exc.code)
        except Exception as exc:
            print("[tts ] edge 拉取失败：%s" % exc)
        return None


def _which(name: str) -> str:
    for path in os.environ.get("PATH", "/usr/bin:/bin").split(":"):
        candidate = os.path.join(path, name)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return ""
