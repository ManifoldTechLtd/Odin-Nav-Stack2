"""HTTP server for the web console: static page, JSON state and the map image."""

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def make_handler(state):
    class ConsoleRequestHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):
            pass

        def _send(self, status, body, content_type, extra_headers=()):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            for name, value in extra_headers:
                self.send_header(name, value)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _send_json(self, payload, status=200):
            self._send(status, json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                       "application/json; charset=utf-8")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                with open(os.path.join(STATIC_DIR, "index.html"), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                return self._send_json(state.snapshot())
            if path == "/api/map.png":
                version, png = state.map_png()
                if png is None:
                    return self._send_json({"ok": False, "error": "map not received"}, status=404)
                etag = '"{}"'.format(version)
                if self.headers.get("If-None-Match") == etag:
                    return self._send(304, b"", "image/png", [("ETag", etag)])
                return self._send(200, png, "image/png", [("ETag", etag)])
            return self._send_json({"ok": False, "error": "Not Found"}, status=404)

    return ConsoleRequestHandler


def make_server(host, port, state):
    server = ThreadingHTTPServer((host, port), make_handler(state))
    server.daemon_threads = True
    return server
