"""HTTP(S) server for the web console: page, JSON API, map image, camera and audio."""

import json
import os
import shutil
import socket
import ssl
import subprocess
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_BODY_BYTES = 15 * 1024 * 1024


class DualProtocolHTTPServer(ThreadingHTTPServer):
    """Single-port HTTP + HTTPS server that performs TLS peek & handshake in worker threads."""

    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 64

    def __init__(self, server_address, RequestHandlerClass, ssl_context: Optional[ssl.SSLContext] = None):
        super().__init__(server_address, RequestHandlerClass)
        self.ssl_context = ssl_context

    def process_request_thread(self, request, client_address):
        if self.ssl_context is not None:
            try:
                request.settimeout(5.0)
                first_byte = request.recv(1, socket.MSG_PEEK)
                if not first_byte:
                    self.shutdown_request(request)
                    return
                if first_byte == b"\x16":
                    request = self.ssl_context.wrap_socket(request, server_side=True)
                request.settimeout(60.0)
            except Exception:
                try:
                    self.shutdown_request(request)
                except Exception:
                    pass
                return
        super().process_request_thread(request, client_address)


def ensure_self_signed_cert(cert_dir: Optional[str] = None) -> Optional[ssl.SSLContext]:
    """Generates a self-signed certificate so the same port serves both HTTP and HTTPS."""
    cert_dir = cert_dir or os.path.join(tempfile.gettempdir(), "ons2_web_console_tls")
    os.makedirs(cert_dir, exist_ok=True)
    cert_path = os.path.join(cert_dir, "cert.pem")
    key_path = os.path.join(cert_dir, "key.pem")
    if not (os.path.isfile(cert_path) and os.path.isfile(key_path)):
        if not shutil.which("openssl"):
            return None
        try:
            subprocess.run(
                ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-keyout", key_path, "-out", cert_path,
                 "-days", "3650", "-nodes", "-subj", "/CN=ons2-web-console"],
                check=True, capture_output=True, timeout=10,
            )
        except Exception:
            return None
    try:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(certfile=cert_path, keyfile=key_path)
        return ctx
    except Exception:
        return None


def _post_routes(hub):
    def safety_control(req):
        if "force_monitor" in req:
            hub.stall.set_force_monitor(bool(req["force_monitor"]))
        if "window_sec" in req or "stall_radius_m" in req:
            hub.stall.set_config(window_sec=req.get("window_sec"), stall_radius_m=req.get("stall_radius_m"))
        return {"ok": True, "safety": hub.stall.evaluate()}

    def safety_ack(req):
        hub.stall.acknowledge_alarm(mute_sec=float(req.get("mute_sec", 60.0)))
        return {"ok": True, "safety": hub.stall.evaluate()}

    return {
        "/api/camera/control": lambda req: {
            "ok": True, "camera": hub.camera.set_enabled(bool(req.get("enabled", False)), fps=req.get("fps"))},
        "/api/safety/control": safety_control,
        "/api/safety/ack": safety_ack,
        "/api/audio/speak_tts": lambda req: hub.intercom.speak_tts_async(
            text=req.get("text", ""), volume=int(req.get("volume", 100)), voice=req.get("voice")),
        "/api/nav/command": lambda req: hub.nav.execute_nav_command(req.get("action", "")),
        "/api/teleop/control": lambda req: hub.nav.set_teleop_takeover(
            active=bool(req.get("active", False)), resume_navigation=bool(req.get("resume_navigation", False))),
        "/api/teleop/cmd_vel": lambda req: hub.nav.set_teleop_velocity(
            vx=float(req.get("vx", 0.0)), vy=float(req.get("vy", 0.0)), vyaw=float(req.get("vyaw", 0.0))),
        "/api/mission/start": lambda req: hub.mission.start(
            req.get("waypoints"), loop=req.get("loop", False), temporary=req.get("temporary", False)),
        "/api/mission/command": lambda req: hub.mission.command(req.get("action", "")),
        "/api/chassis/control": lambda req: hub.mission.set_chassis(req.get("enabled", False)),
        "/api/stack/start": lambda req: hub.stack.start(
            req.get("map", ""), req.get("navigation_mode", "replan_periodic"),
            planner=req.get("planner", "neupan"), sru_policy=req.get("sru_policy", "main")),
        "/api/stack/stop": lambda req: hub.stack.stop(),
        "/api/record/start": lambda req: hub.recorder.start(
            req.get("topics"), name=req.get("name", ""), topic_types=hub.topic_types()),
        "/api/record/stop": lambda req: hub.recorder.stop(),
    }


def make_handler(hub):
    routes = _post_routes(hub)

    class ConsoleRequestHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):
            pass

        def handle(self):
            try:
                super().handle()
            except (ConnectionResetError, BrokenPipeError, TimeoutError, ssl.SSLError, OSError):
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

        def _read_body(self) -> bytes:
            length = int(self.headers.get("Content-Length", "0") or "0")
            return self.rfile.read(min(length, MAX_BODY_BYTES)) if length > 0 else b""

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                with open(os.path.join(STATIC_DIR, "index.html"), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                return self._send_json(hub.get_snapshot())
            if path == "/api/map.png":
                version, png = hub.console.map_png()
                if png is None:
                    return self._send_json({"ok": False, "error": "map not received"}, status=404)
                etag = '"{}"'.format(version)
                if self.headers.get("If-None-Match") == etag:
                    return self._send(304, b"", "image/png", [("ETag", etag)])
                return self._send(200, png, "image/png", [("ETag", etag)])
            if path == "/api/stack/state":
                from web_console.stack import MODES, PLANNERS
                payload = hub.stack.status()
                payload.update({"maps": hub.stack.list_maps(), "modes": list(MODES),
                                "planners": list(PLANNERS), "sru_policies": hub.stack.list_sru_policies()})
                return self._send_json(payload)
            if path == "/api/record/state":
                from web_console.recorder import PRESETS
                return self._send_json(dict(hub.recorder.status(), presets=PRESETS))
            if path == "/api/record/topics":
                return self._send_json({"topics": hub.topics()})
            if path == "/api/camera/frame.jpg":
                jpeg, _, _ = hub.camera.get_latest_frame()
                if not jpeg and hub.camera.enabled:
                    jpeg, _, _ = hub.camera.wait_for_next_frame(0, timeout=0.8)
                if not jpeg:
                    return self._send(204, b"", "image/jpeg")
                return self._send(200, jpeg, "image/jpeg", [("Cache-Control", "no-store, no-cache, must-revalidate")])
            return self._send_json({"ok": False, "error": "Not Found"}, status=404)

        def do_POST(self):
            parsed = urlparse(self.path)
            body = self._read_body()
            try:
                if parsed.path == "/api/audio/speak_raw":
                    qs = parse_qs(parsed.query)
                    volume = int((qs.get("volume") or ["100"])[0])
                    mime = (qs.get("mime") or [self.headers.get("Content-Type", "webm")])[0]
                    return self._send_json(hub.intercom.speak_raw_async(body, mime_or_ext=mime, volume=volume))
                handler = routes.get(parsed.path)
                if handler is None:
                    return self._send_json({"ok": False, "error": "Not Found"}, status=404)
                return self._send_json(handler(json.loads(body.decode("utf-8") or "{}")))
            except Exception as exc:
                return self._send_json({"ok": False, "error": str(exc)}, status=400)

    return ConsoleRequestHandler


def make_server(host, port, hub, enable_tls=True):
    ssl_ctx = ensure_self_signed_cert() if enable_tls else None
    return DualProtocolHTTPServer((host, port), make_handler(hub), ssl_context=ssl_ctx)
