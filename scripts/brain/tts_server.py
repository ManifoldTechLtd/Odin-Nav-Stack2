#!/usr/bin/env python3
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

"""Local TTS HTTP server: synthesizes speech via edge-tts and converts to 16kHz mono WAV/Opus via ffmpeg.

Usage:
    pip install edge-tts
    python3 scripts/tts_server.py --port 18081

Endpoints:
    POST /tts    {"text": "...", "voice": "zh-CN-XiaoxiaoNeural"} -> audio/wav or audio/opus
    GET  /health -> {"ok": true, "voice": "..."}
"""

import argparse
import asyncio
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

CACHE_DIR = os.path.join(tempfile.gettempdir(), "odin_tts_cache")
DEFAULT_VOICE = "zh-CN-XiaoxiaoNeural"
VOICE = DEFAULT_VOICE
TIMEOUT_SEC = 25


def synthesize(text: str, voice: str = None) -> str:
    """Synthesize speech to WAV with disk caching and return the WAV file path."""
    voice = voice or VOICE
    key = hashlib.sha1(("%s|%s" % (voice, text)).encode("utf-8")).hexdigest()[:16]
    os.makedirs(CACHE_DIR, exist_ok=True)
    wav_path = os.path.join(CACHE_DIR, key + ".wav")
    if os.path.isfile(wav_path) and os.path.getsize(wav_path) > 1024:
        return wav_path

    mp3_path = os.path.join(CACHE_DIR, key + ".mp3")

    async def _run():
        import edge_tts
        await edge_tts.Communicate(text, voice).save(mp3_path)

    asyncio.run(_run())
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", mp3_path,
         "-ar", "16000", "-ac", "1", "-f", "wav", wav_path],
        check=True, timeout=30)
    try:
        os.remove(mp3_path)
    except OSError:
        pass
    return wav_path


# Common phrases pre-warmed in background cache at startup (override via ODIN_TTS_PREWARM).
PREWARM_TEXTS = tuple(
    w.strip()
    for w in os.environ.get(
        "ODIN_TTS_PREWARM",
        "OK,Roger,Copy,Standing by,Paused,Resumed,Cancelled,Returning home,Mission completed,Arrived",
    ).split(",")
    if w.strip()
)


def prewarm_async(delay: float = 3.0) -> None:
    """Asynchronously pre-warm common phrases after server startup."""
    def _run() -> None:
        time.sleep(delay)
        ok = 0
        for text in PREWARM_TEXTS:
            try:
                synthesize_opus(text)
                ok += 1
            except Exception:
                pass
        print("[tts ] Pre-warmed %d/%d common phrases" % (ok, len(PREWARM_TEXTS)), flush=True)

    threading.Thread(target=_run, daemon=True).start()


def synthesize_opus(text: str, voice: str = None, bitrate: int = 32) -> str:
    """Transcode cached WAV into Opus for low-bandwidth links."""
    wav_path = synthesize(text, voice)
    opus_path = wav_path[:-4] + ".opus"
    if os.path.isfile(opus_path) and os.path.getsize(opus_path) > 512:
        return opus_path
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-i", wav_path,
             "-c:a", "libopus", "-b:a", "%dk" % bitrate, opus_path],
            check=True, timeout=30)
    except Exception:
        return ""
    return opus_path if os.path.isfile(opus_path) else ""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        path = urlparse(self.path).path
        if path.rstrip("/").endswith("/health"):
            self._json(200, {"ok": True, "voice": VOICE, "cache": CACHE_DIR})
            return
        self._json(404, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        if not path.rstrip("/").endswith("/tts"):
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            text = str(body.get("text", "")).strip()
            voice = body.get("voice") or VOICE
        except Exception as exc:
            self._json(400, {"error": "bad json: %s" % exc})
            return
        if not text:
            self._json(400, {"error": "empty text"})
            return
        if len(text) > 500:
            text = text[:500]

        started = time.time()
        want_opus = "opus" in (self.path or "").lower()
        try:
            audio_path = synthesize_opus(text, voice) if want_opus else synthesize(text, voice)
        except Exception as exc:
            self._json(500, {"error": "tts failed: %s" % exc})
            return
        if not audio_path:
            self._json(500, {"error": "tts failed"})
            return
        try:
            with open(audio_path, "rb") as handle:
                data = handle.read()
        except OSError as exc:
            self._json(500, {"error": "read audio failed: %s" % exc})
            return

        self.send_response(200)
        self.send_header("Content-Type", "audio/opus" if want_opus else "audio/wav")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("X-TTS-Ms", str(int((time.time() - started) * 1000)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code: int, payload: dict):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        sys.stderr.write("[tts] %s\n" % (fmt % args))


def main(argv=None) -> int:
    global VOICE
    parser = argparse.ArgumentParser(description="Odin TTS server (edge-tts + ffmpeg)")
    parser.add_argument("--port", type=int, default=int(os.environ.get("ODIN_TTS_PORT", "18081")))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--voice", default=os.environ.get("ODIN_TTS_VOICE", DEFAULT_VOICE))
    args = parser.parse_args(argv)

    VOICE = args.voice
    try:
        import edge_tts  # noqa: F401
    except ImportError:
        print("[fail] edge-tts not installed: pip install edge-tts", file=sys.stderr)
        return 2
    if not subprocess.run(["which", "ffmpeg"], stdout=subprocess.DEVNULL).returncode == 0:
        print("[fail] ffmpeg not installed", file=sys.stderr)
        return 2

    print("[info] tts_server: http://%s:%d/tts  voice=%s" % (args.host, args.port, VOICE))
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    prewarm_async()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
