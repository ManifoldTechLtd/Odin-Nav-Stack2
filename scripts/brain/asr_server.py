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

"""Local speech recognition (ASR) HTTP server (default: 0.0.0.0:18082).

Powered by FunASR SeACo-Paraformer with dynamic hotword boosting:
- Serves both the Hermes web dashboard and the robot microphone input.
- Automatically converts input audio (wav/webm/ogg/mp3) to 16kHz mono WAV via ffmpeg.
- Removes redundant spaces between CJK characters in recognition output.
"""

import argparse
import base64
import json
import os
import re
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List

from funasr import AutoModel

DEFAULT_HOTWORDS: List[str] = [
    w.strip()
    for w in os.environ.get("ODIN_ASR_HOTWORDS", "").split(",")
    if w.strip()
]

_MODEL = None
_MODEL_LOCK = threading.Lock()


def get_model():
    global _MODEL
    if _MODEL is None:
        print("[asr ] Loading FunASR SeACo-Paraformer model...", flush=True)
        _MODEL = AutoModel(model="paraformer-zh", disable_update=True)
        print("[asr ] Model loaded and ready.", flush=True)
    return _MODEL


def _convert_to_16k_wav(raw_bytes: bytes) -> str:
    """Convert raw audio bytes (webm/ogg/wav) to a temporary 16kHz mono s16le WAV file via ffmpeg."""
    src_fd, src_path = tempfile.mkstemp(prefix="odin_asr_in_", suffix=".bin")
    dst_fd, dst_path = tempfile.mkstemp(prefix="odin_asr_out_", suffix=".wav")
    os.close(src_fd)
    os.close(dst_fd)
    try:
        with open(src_path, "wb") as handle:
            handle.write(raw_bytes)
        subprocess.run(
            ["ffmpeg", "-y", "-i", src_path, "-ar", "16000", "-ac", "1", "-f", "wav", dst_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=15,
            check=True,
        )
        return dst_path
    finally:
        try:
            os.unlink(src_path)
        except OSError:
            pass


def _clean_cjk_spaces(text: str) -> str:
    """Remove whitespace between CJK characters while preserving spaces between English words."""
    if not text:
        return ""
    out = re.sub(r"(?<=[\u4e00-\u9fff])\s+(?=[\u4e00-\u9fff0-9])", "", text)
    out = re.sub(r"(?<=[0-9])\s+(?=[\u4e00-\u9fff])", "", out)
    return out.strip()


def transcribe_bytes(raw_bytes: bytes, extra_hotwords: List[str]) -> dict:
    started = time.time()
    wav_path = ""
    try:
        wav_path = _convert_to_16k_wav(raw_bytes)
        merged_words: List[str] = []
        for word in list(extra_hotwords or []) + DEFAULT_HOTWORDS:
            w = str(word).strip()
            if len(w) >= 2 and w not in merged_words:
                merged_words.append(w)
        hotword_str = " ".join(merged_words)
        model = get_model()
        with _MODEL_LOCK:
            res = model.generate(input=wav_path, hotword=hotword_str)
        raw_text = ""
        if isinstance(res, list) and res and isinstance(res[0], dict):
            raw_text = str(res[0].get("text") or "")
        text = _clean_cjk_spaces(raw_text)
        latency_ms = int((time.time() - started) * 1000)
        print(f"[asr ] {latency_ms}ms -> {text!r}", flush=True)
        return {"ok": True, "text": text, "raw_text": raw_text, "latency_ms": latency_ms}
    finally:
        if wav_path:
            try:
                os.unlink(wav_path)
            except OSError:
                pass


class ASRHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _send_json(self, code: int, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/health"):
            self._send_json(200, {"ok": True, "engine": "funasr-seaco-paraformer-zh"})
            return
        self._send_json(404, {"ok": False, "error": "Not found"})

    def do_POST(self):
        if not self.path.startswith("/asr"):
            self._send_json(404, {"ok": False, "error": "Use POST /asr"})
            return
        length = int(self.headers.get("Content-Length") or "0")
        if length <= 0 or length > 25 * 1024 * 1024:
            self._send_json(400, {"ok": False, "error": "Invalid body length"})
            return
        body_raw = self.rfile.read(length)

        # Support direct binary audio upload (opus/webm/wav) with hotwords in HTTP headers.
        ctype = (self.headers.get("Content-Type") or "").lower()
        if "json" not in ctype:
            hotwords = []
            try:
                b64_hot = self.headers.get("X-Hotwords-B64", "") or ""
                if b64_hot:
                    hotwords = json.loads(base64.b64decode(b64_hot).decode("utf-8"))
                else:
                    hotwords = json.loads(self.headers.get("X-Hotwords", "[]") or "[]")
            except Exception:
                hotwords = []
            try:
                result = transcribe_bytes(body_raw, hotwords if isinstance(hotwords, list) else [])
                self._send_json(200, result)
            except Exception as exc:
                self._send_json(500, {"ok": False, "error": str(exc)})
            return

        try:
            body = json.loads(body_raw.decode("utf-8"))
        except Exception as exc:
            self._send_json(400, {"ok": False, "error": f"Invalid JSON: {exc}"})
            return

        b64 = str(body.get("audio_b64") or "").strip()
        if "," in b64 and b64.startswith("data:"):
            b64 = b64.split(",", 1)[1]
        if not b64:
            self._send_json(400, {"ok": False, "error": "Missing audio_b64"})
            return
        try:
            audio_bytes = base64.b64decode(b64)
            hotwords = body.get("hotwords") or []
            result = transcribe_bytes(audio_bytes, hotwords if isinstance(hotwords, list) else [])
            self._send_json(200, result)
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})


def main():
    parser = argparse.ArgumentParser(description="Odin Brain local hotword ASR service")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18082)
    args = parser.parse_args()

    get_model()
    server = ThreadingHTTPServer((args.host, args.port), ASRHandler)
    print(f"[asr ] Listening on http://{args.host}:{args.port}/asr", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
