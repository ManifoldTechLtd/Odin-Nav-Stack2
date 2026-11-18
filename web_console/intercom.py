"""Audio intercom: browser recording / TTS played on the robot speaker."""

import asyncio
import hashlib
from concurrent.futures import ThreadPoolExecutor
import os
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Callable, Optional


MAX_QUEUED = 5


class AudioIntercom:
    """Handles remote microphone audio playback and Text-to-Speech on the robot."""

    PRESET_PHRASES = (
        "请让一让，机器狗正在巡检",
        "请注意安全，保持距离",
        "机器狗受阻，请协助让行",
        "正在执行导航任务，请勿触碰",
        "收到，正在处理",
        "谢谢配合",
    )

    def __init__(
        self,
        cache_dir: Optional[str] = None,
        default_voice: str = "zh-CN-XiaoxiaoNeural",
        custom_play_fn: Optional[Callable[[str, dict], None]] = None,
        custom_tts_fn: Optional[Callable[[str, str], None]] = None,
    ):
        self.cache_dir = cache_dir or os.path.join(tempfile.gettempdir(), "ons2_intercom_cache")
        os.makedirs(self.cache_dir, exist_ok=True)
        self.default_voice = default_voice
        self._custom_play_fn = custom_play_fn
        self._custom_tts_fn = custom_tts_fn
        self._play_lock = threading.Lock()
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="intercom")
        self._pending_lock = threading.Lock()
        self._pending = 0
        self.last_error: Optional[str] = None
        self.last_spoken_at: Optional[float] = None
        self.last_spoken_summary: str = ""
        self.play_count: int = 0

    def _submit(self, fn, *args):
        with self._pending_lock:
            if self._pending >= MAX_QUEUED:
                raise ValueError("播报队列已满，请稍后再试")
            self._pending += 1

        def job():
            try:
                fn(*args)
                self.last_error = None
            except Exception as exc:
                self.last_error = str(exc) or exc.__class__.__name__
            finally:
                with self._pending_lock:
                    self._pending -= 1

        self._worker.submit(job)
        return {"ok": True, "queued": True, "message": "已加入播报队列"}

    def speak_tts_async(self, text: str, volume: int = 100, voice: Optional[str] = None) -> dict:
        clean_text = (text or "").strip()
        if not clean_text:
            raise ValueError("播报文字不能为空")
        res = self._submit(self.speak_tts, clean_text, volume, voice)
        return dict(res, text=clean_text[:200], volume=volume)

    def speak_raw_async(self, raw_bytes: bytes, mime_or_ext: str = "webm", volume: int = 100) -> dict:
        if not raw_bytes or len(raw_bytes) < 64:
            raise ValueError("录音数据为空或过短")
        return self._submit(self.speak_raw_audio, raw_bytes, mime_or_ext, volume)

    def speak_tts(self, text: str, volume: int = 100, voice: Optional[str] = None) -> dict:
        clean_text = (text or "").strip()
        if not clean_text:
            raise ValueError("播报文字不能为空")
        if len(clean_text) > 200:
            clean_text = clean_text[:200]
        voice = voice or self.default_voice
        volume = max(20, min(int(volume), 250))

        key = hashlib.sha1(f"{voice}|{clean_text}".encode("utf-8")).hexdigest()[:16]
        base_wav = os.path.join(self.cache_dir, f"tts_{key}.wav")

        if not (os.path.isfile(base_wav) and os.path.getsize(base_wav) > 256):
            if self._custom_tts_fn is not None:
                self._custom_tts_fn(clean_text, base_wav)
            else:
                self._synthesize_to_wav(clean_text, voice, base_wav)

        play_wav = self._apply_volume_if_needed(base_wav, volume)
        meta = {"mode": "tts", "text": clean_text, "volume": volume}
        self._play_wav_file(play_wav, meta)
        return {
            "ok": True,
            "mode": "tts",
            "text": clean_text,
            "volume": volume,
            "play_count": self.play_count,
        }

    def speak_raw_audio(self, raw_bytes: bytes, mime_or_ext: str = "webm", volume: int = 100) -> dict:
        if not raw_bytes or len(raw_bytes) < 64:
            raise ValueError("录音数据为空或过短")
        volume = max(20, min(int(volume), 250))
        ext = "webm"
        lower = (mime_or_ext or "").lower()
        for cand in ("wav", "ogg", "mp4", "m4a", "mp3", "aac", "webm"):
            if cand in lower:
                ext = cand
                break

        stamp = int(time.time() * 1000)
        in_path = os.path.join(self.cache_dir, f"mic_{stamp}_in.{ext}")
        out_wav = os.path.join(self.cache_dir, f"mic_{stamp}_out.wav")
        try:
            with open(in_path, "wb") as f:
                f.write(raw_bytes)
            if self._custom_play_fn is not None and not shutil.which("ffmpeg"):
                out_wav = in_path
            else:
                vol_ratio = volume / 100.0
                subprocess.run(
                    [
                        "ffmpeg", "-y", "-loglevel", "error",
                        "-i", in_path,
                        "-af", f"volume={vol_ratio:.2f}",
                        "-ar", "16000", "-ac", "1", "-f", "wav",
                        out_wav,
                    ],
                    check=True,
                    timeout=15,
                )
            meta = {"mode": "mic", "bytes": len(raw_bytes), "volume": volume}
            self._play_wav_file(out_wav, meta)
            return {
                "ok": True,
                "mode": "mic",
                "bytes": len(raw_bytes),
                "volume": volume,
                "play_count": self.play_count,
            }
        finally:
            for p in (in_path, out_wav):
                if p and os.path.isfile(p) and self._custom_play_fn is None:
                    try:
                        os.remove(p)
                    except OSError:
                        pass

    def _synthesize_to_wav(self, text: str, voice: str, out_wav: str) -> None:
        mp3_path = out_wav[:-4] + ".mp3"
        edge_ok = False
        try:
            import edge_tts

            async def _run_edge():
                await edge_tts.Communicate(text, voice).save(mp3_path)

            asyncio.run(_run_edge())
            if os.path.isfile(mp3_path) and os.path.getsize(mp3_path) > 256:
                subprocess.run(
                    [
                        "ffmpeg", "-y", "-loglevel", "error",
                        "-i", mp3_path,
                        "-ar", "16000", "-ac", "1", "-f", "wav",
                        out_wav,
                    ],
                    check=True,
                    timeout=20,
                )
                edge_ok = True
        except Exception:
            edge_ok = False
        finally:
            if os.path.isfile(mp3_path):
                try:
                    os.remove(mp3_path)
                except OSError:
                    pass

        if edge_ok and os.path.isfile(out_wav):
            return

        if shutil.which("espeak-ng"):
            subprocess.run(
                ["espeak-ng", "-v", "cmn", "-s", "150", "-w", out_wav, text],
                check=True,
                timeout=10,
            )
            return

        if shutil.which("ffmpeg"):
            # Fallback attention chime if no offline TTS binary is installed
            subprocess.run(
                [
                    "ffmpeg", "-y", "-loglevel", "error",
                    "-f", "lavfi", "-i", "sine=frequency=880:duration=0.6",
                    "-ar", "16000", "-ac", "1", out_wav,
                ],
                check=True,
                timeout=10,
            )
            return

        raise RuntimeError("未找到可用的语音合成器 (edge-tts / espeak-ng / ffmpeg)")

    def _apply_volume_if_needed(self, base_wav: str, volume: int) -> str:
        if volume == 100 or not shutil.which("ffmpeg") or self._custom_play_fn is not None:
            return base_wav
        scaled_wav = base_wav[:-4] + f"_v{volume}.wav"
        if os.path.isfile(scaled_wav) and os.path.getsize(scaled_wav) > 256:
            return scaled_wav
        vol_ratio = volume / 100.0
        subprocess.run(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-i", base_wav,
                "-af", f"volume={vol_ratio:.2f}",
                "-ar", "16000", "-ac", "1", "-f", "wav",
                scaled_wav,
            ],
            check=True,
            timeout=10,
        )
        return scaled_wav

    def _detect_usb_card_indices(self) -> list:
        indices = []
        try:
            if os.path.isfile("/proc/asound/cards"):
                with open("/proc/asound/cards", "r", encoding="utf-8", errors="ignore") as f:
                    for line in f:
                        if "USB-Audio" in line or "USB Audio" in line:
                            parts = line.strip().split()
                            if parts and parts[0].isdigit():
                                indices.append(int(parts[0]))
        except Exception:
            pass
        return indices

    def _play_wav_file(self, wav_path: str, meta: dict) -> None:
        with self._play_lock:
            self.play_count += 1
            self.last_spoken_at = time.time()
            self.last_spoken_summary = meta.get("text") or f"麦克风原声 ({meta.get('bytes', 0)}B)"
            if self._custom_play_fn is not None:
                self._custom_play_fn(wav_path, meta)
                return

            if shutil.which("amixer"):
                for ctl in ("Master", "Headphone", "Speaker", "PCM"):
                    subprocess.run(
                        ["amixer", "-c", "0", "sset", ctl, "95%", "unmute"],
                        capture_output=True,
                        timeout=3,
                    )

            players = []
            usb_cards = self._detect_usb_card_indices()
            if shutil.which("aplay"):
                for c in usb_cards:
                    players.append(["aplay", "-q", "-D", f"plughw:{c},0", wav_path])
            if shutil.which("paplay"):
                players.append(["paplay", wav_path])
            if shutil.which("pw-play"):
                players.append(["pw-play", wav_path])
            if shutil.which("aplay"):
                players.append(["aplay", "-q", "-D", "plughw:0,0", wav_path])
                players.append(["aplay", "-q", wav_path])
            if shutil.which("ffplay"):
                players.append(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", wav_path])

            last_err = None
            for cmd in players:
                try:
                    env = os.environ.copy()
                    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
                    env.setdefault("SDL_AUDIODRIVER", "alsa")
                    env.setdefault("AUDIODEV", "plughw:0,0")
                    res = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
                    if res.returncode == 0:
                        return
                    last_err = res.stderr.strip() or f"exit {res.returncode}"
                except Exception as exc:
                    last_err = str(exc)

            raise RuntimeError(f"音频播放失败 ({last_err or '无可用播放器'})")

    def get_status(self) -> dict:
        return {
            "play_count": self.play_count,
            "last_spoken_at": self.last_spoken_at,
            "last_spoken_summary": self.last_spoken_summary,
            "pending": self._pending,
            "last_error": self.last_error,
            "presets": list(self.PRESET_PHRASES),
        }


