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

"""Minimal OpenAI-compatible HTTP proxy for robots without direct internet access.

Forwards requests (including SSE streaming) and injects the upstream API key.

Usage:
    export ODIN_LLM_API_KEY=<your_api_key>
    python3 scripts/llm_proxy.py --port 18080
    python3 scripts/llm_proxy.py --upstream https://open.bigmodel.cn/api/paas/v4 --port 18080
"""

import argparse
import os
import sys
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UPSTREAM = "https://open.bigmodel.cn/api/paas/v4"
API_KEY = ""
TIMEOUT = 120


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path.rstrip("/").endswith("/models"):
            self._forward("GET", None)
            return
        self._json(200, {"object": "list", "data": [],
                         "note": "llm_proxy: use POST /v1/chat/completions"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(length) if length else b""
        self._forward("POST", body)

    def _forward(self, method, body):
        path = self.path
        suffix = path.split("/v1", 1)[1] if "/v1" in path else path
        url = UPSTREAM.rstrip("/") + suffix
        headers = {"Content-Type": "application/json"}
        if API_KEY:
            headers["Authorization"] = "Bearer " + API_KEY

        request = urllib.request.Request(url, data=body, headers=headers, method=method)
        try:
            response = urllib.request.urlopen(request, timeout=TIMEOUT)
        except urllib.error.HTTPError as exc:
            detail = exc.read()
            self._json(exc.code, _safe_json(detail, {"error": {"message": detail[:300].decode("utf-8", "replace")}}))
            return
        except Exception as exc:
            self._json(502, {"error": {"code": "proxy", "message": "Upstream unreachable: %s" % exc}})
            return

        content_type = response.headers.get("Content-Type", "application/json")
        is_stream = "event-stream" in content_type
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache")
        if not is_stream:
            data = response.read()
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        # Stream SSE chunks directly
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()
        try:
            while True:
                chunk = response.read1(4096)
                if not chunk:
                    break
                self.wfile.write(b"%x\r\n" % len(chunk) + chunk + b"\r\n")
                self.wfile.flush()
            self.wfile.write(b"0\r\n\r\n")
        except Exception:
            pass
        finally:
            try:
                response.close()
            except Exception:
                pass

    def _json(self, code, payload):
        import json
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        sys.stderr.write("[proxy] %s\n" % (fmt % args))


def _safe_json(raw, fallback):
    import json
    try:
        return json.loads(raw.decode("utf-8", "replace"))
    except Exception:
        return fallback


def _load_key_from_env_file(path: str, key_name: str = "ODIN_LLM_API_KEY") -> str:
    """Fallback loader for API key from an env file when environment variable is unset."""
    if not os.path.isfile(path):
        return ""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                name, value = line.split("=", 1)
                if name.strip() == key_name:
                    return value.strip().strip("\"'")
    except OSError:
        pass
    return ""


def main(argv=None):
    global UPSTREAM, API_KEY, TIMEOUT
    parser = argparse.ArgumentParser(description="OpenAI-compatible HTTP proxy")
    parser.add_argument("--upstream", default=os.environ.get("ODIN_LLM_UPSTREAM", UPSTREAM))
    parser.add_argument("--api-key-env", default="ODIN_LLM_API_KEY")
    parser.add_argument("--env-file",
                        default=os.environ.get("ODIN_LLM_ENV_FILE",
                                               os.path.expanduser("~/.config/odin_brain/llm.env")),
                        help="Fallback env file for reading API key")
    parser.add_argument("--port", type=int, default=int(os.environ.get("ODIN_LLM_PROXY_PORT", "18080")))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args(argv)

    UPSTREAM = args.upstream.rstrip("/")
    TIMEOUT = args.timeout
    API_KEY = os.environ.get(args.api_key_env, "").strip()
    key_source = "env var"
    if not API_KEY:
        API_KEY = _load_key_from_env_file(args.env_file, args.api_key_env)
        key_source = "env file %s" % args.env_file if API_KEY else "none"
    if not API_KEY:
        print("[warn] %s is empty and not found in %s" % (args.api_key_env, args.env_file), file=sys.stderr)
    else:
        print("[info] API key source: %s (%s...)" % (key_source, API_KEY[:8]))

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("[info] llm_proxy: http://%s:%d/v1  ->  %s" % (args.host, args.port, UPSTREAM))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
