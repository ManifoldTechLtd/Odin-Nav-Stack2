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

"""Resident HTTP server for odin_brain (default port 18090).

Endpoints:
    GET  /health       -> {"ok", "engine_ready", "map_id", "places_count", "uptime_sec"}
    GET  /status       -> Real-time navigation and pose snapshot
    GET  /job/<id>     -> Asynchronous job result
    POST /command      -> {"instruction": "...", "dry_run": false} (supports ?async=1)
"""

import argparse
import json
import os
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
if os.path.isdir(_SRC) and _SRC not in sys.path:
    sys.path.insert(0, _SRC)

# Resident state
_ENGINE = None
_STORE = None
_NAV = None
_SPEAKER = None
_LLM = None
_LOCK = threading.Lock()
_STARTED_AT = time.time()
_OPTS = {}
# Asynchronous job table: job_id -> {state, ok, reply, instruction, started_at, finished_at}
_JOBS = {}
_JOBS_MAX = 50


def _log(message: str) -> None:
    print("[srv ] %s" % message, flush=True)


def build_engine(args) -> None:
    """Initialize the resident BrainEngine and its dependencies."""
    global _ENGINE, _STORE, _NAV, _SPEAKER, _LLM

    from odin_brain.engine import BrainEngine
    from odin_brain.llm_client import build_client
    from odin_brain.places import PlaceStore
    from odin_brain.planner import RulePlanner
    from odin_brain.speech import Speaker

    places_path = os.path.expanduser(args.places)
    _STORE = PlaceStore(places_path)
    _log("Places: %s (map_id=%s, %d places)"
         % (places_path, _STORE.map_id or "-", len(_STORE.places)))

    planner = RulePlanner(_STORE)
    if args.no_llm:
        _LLM = None
        _log("Parser: rule-based (LLM disabled)")
    else:
        try:
            _LLM = build_client()
        except Exception as exc:  # noqa: BLE001
            _log("Parser: rule-based (LLM init failed: %s)" % exc)
            _LLM = None
        if _LLM is not None and _LLM.available:
            _log("Parser: LLM-first (%s)" % _LLM.config.describe())
        else:
            _LLM = None
            _log("Parser: rule-based (LLM unavailable, falling back)")

    if args.mock:
        from odin_brain.mock_nav import MockNav
        _NAV = MockNav()
        _log("Nav backend: Mock")
    else:
        import rospy
        from odin_brain.ros1_nav import Ros1Nav
        if not rospy.core.is_initialized():
            rospy.init_node("odin_brain_server", anonymous=True, disable_signals=True)
        _NAV = Ros1Nav(map_frame=args.map_frame, base_frame=args.base_frame)
        ready, message = _NAV.wait_ready(timeout_sec=float(args.nav_timeout))
        _log("Nav backend: ROS1 (%s)" % ("ready" if ready else message))

    _ENGINE = BrainEngine(_STORE, planner, _LLM, _NAV,
                          dry_run=bool(args.dry_run), verbose=True, loop=False)

    _SPEAKER = Speaker(enabled=not args.no_tts)
    _log("TTS: %s" % _SPEAKER.describe())
    if _SPEAKER is not None and _ENGINE is not None:
        _ENGINE.on_speak = _SPEAKER.speak
        _ENGINE.executor.speaker = _SPEAKER

    _OPTS.update({
        "places": places_path,
        "mock": bool(args.mock),
        "no_llm": bool(args.no_llm),
        "no_tts": bool(args.no_tts),
    })
    _log("Engine ready (resident)")


def _status_snapshot() -> dict:
    data = {"ok": True, "uptime_sec": round(time.time() - _STARTED_AT, 1)}
    if _STORE is not None:
        data["map_id"] = _STORE.map_id or ""
        data["places_count"] = len(_STORE.places)
    if _NAV is not None:
        try:
            snapshot = _NAV.snapshot()
            data["nav"] = {
                "state": snapshot.get("state"),
                "mission_id": snapshot.get("mission_id"),
                "progress": snapshot.get("progress"),
            }
        except Exception as exc:  # noqa: BLE001
            data["nav_error"] = str(exc)
        try:
            pose = _NAV.get_pose()
            if pose:
                x, y, yaw = float(pose["x"]), float(pose["y"]), float(pose["yaw"])
                data["pose"] = {
                    "x": round(x, 3), "y": round(y, 3),
                    "yaw": round(yaw, 4), "yaw_deg": round(yaw * 57.29578, 1),
                }
                if _STORE is not None and _STORE.places:
                    nearest = min(_STORE.places,
                                  key=lambda p: (p.x - x) ** 2 + (p.y - y) ** 2)
                    dist = ((nearest.x - x) ** 2 + (nearest.y - y) ** 2) ** 0.5
                    data["nearest_place"] = {"name": nearest.name,
                                            "distance_m": round(dist, 2)}
            else:
                data["pose"] = None
        except Exception as exc:  # noqa: BLE001
            data["pose_error"] = str(exc)
    if _ENGINE is not None:
        data["busy"] = bool(_ENGINE.busy)
    data["config"] = dict(_OPTS)
    return data


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "OdinBrainServer/1.0"

    def _json(self, code: int, payload: dict) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, fmt, *args):  # noqa: A003
        sys.stderr.write("[srv ] %s\n" % (fmt % args))

    def do_GET(self):  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        if path.endswith("/health"):
            payload = _status_snapshot()
            payload["engine_ready"] = _ENGINE is not None
            self._json(200 if _ENGINE is not None else 503, payload)
            return
        if path.endswith("/status"):
            self._json(200, _status_snapshot())
            return
        if "/job/" in path:
            job_id = path.rsplit("/", 1)[-1]
            job = _JOBS.get(job_id)
            if job is None:
                self._json(404, {"ok": False, "error": "job not found or expired"})
            else:
                self._json(200, dict(job, job_id=job_id))
            return
        self._json(404, {"ok": False,
                         "error": "not found (available: /health /status /job/<id> /command)"})

    def do_POST(self):  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        if not path.endswith("/command"):
            self._json(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
        except Exception as exc:  # noqa: BLE001
            self._json(400, {"ok": False, "error": "bad json: %s" % exc})
            return

        instruction = str(body.get("instruction") or "").strip()
        if not instruction:
            self._json(400, {"ok": False, "error": "instruction cannot be empty"})
            return
        if _ENGINE is None:
            self._json(503, {"ok": False, "error": "engine not ready"})
            return

        want_dry = body.get("dry_run")

        query = parse_qs(parsed.query)
        async_mode = (query.get("async", ["0"])[0] or "0").lower() in ("1", "true", "yes")
        if async_mode:
            job_id = uuid.uuid4().hex[:12]
            _JOBS[job_id] = {"state": "DISPATCHED", "ok": True, "reply": "",
                             "instruction": instruction, "started_at": time.time()}
            if len(_JOBS) > _JOBS_MAX:
                for stale in sorted(_JOBS, key=lambda k: _JOBS[k].get("started_at", 0))[:- _JOBS_MAX]:
                    _JOBS.pop(stale, None)

            def _worker() -> None:
                with _LOCK:
                    previous = _ENGINE.dry_run
                    if want_dry is not None:
                        _ENGINE.dry_run = bool(want_dry)
                        _ENGINE.executor.dry_run = bool(want_dry)
                    try:
                        result = _ENGINE.handle(instruction)
                        _JOBS[job_id].update({
                            "state": getattr(result, "state", "") or "COMPLETED",
                            "ok": bool(getattr(result, "ok", False)),
                            "reply": getattr(result, "reply", "") or "",
                            "intent": getattr(result, "intent", "") or "",
                            "finished_at": time.time(),
                        })
                    except Exception as exc:  # noqa: BLE001
                        _JOBS[job_id].update({"state": "ERROR", "ok": False,
                                             "error": "%s: %s" % (type(exc).__name__, exc),
                                             "finished_at": time.time()})
                    finally:
                        _ENGINE.dry_run = previous
                        _ENGINE.executor.dry_run = previous

            threading.Thread(target=_worker, daemon=True).start()
            self._json(202, {"ok": True, "state": "DISPATCHED", "job_id": job_id,
                             "instruction": instruction,
                             "hint": "Poll GET /status for live progress; GET /job/<id> for final state"})
            return

        started = time.time()
        with _LOCK:
            previous_dry = _ENGINE.dry_run
            if want_dry is not None:
                _ENGINE.dry_run = bool(want_dry)
                _ENGINE.executor.dry_run = bool(want_dry)
            try:
                result = _ENGINE.handle(instruction)
            except Exception as exc:  # noqa: BLE001
                _ENGINE.dry_run = previous_dry
                _ENGINE.executor.dry_run = previous_dry
                self._json(500, {"ok": False, "state": "ERROR",
                                 "error": "%s: %s" % (type(exc).__name__, exc)})
                return
            _ENGINE.dry_run = previous_dry
            _ENGINE.executor.dry_run = previous_dry

        steps = []
        plan = getattr(result, "plan", None)
        if plan is not None:
            try:
                steps = [getattr(s, "label", "") or getattr(s, "kind", "")
                         for s in (plan.steps or [])]
            except Exception:  # noqa: BLE001
                steps = []

        self._json(200, {
            "ok": bool(getattr(result, "ok", False)),
            "state": getattr(result, "state", "") or "",
            "intent": getattr(result, "intent", "") or "",
            "source": getattr(result, "source", "") or "",
            "reply": getattr(result, "reply", "") or "",
            "mission_id": getattr(result, "mission_id", "") or "",
            "steps": steps,
            "elapsed_sec": round(time.time() - started, 3),
            "dry_run": bool(want_dry) if want_dry is not None else None,
        })


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Odin Brain resident HTTP server")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=int(os.environ.get("ODIN_BRAIN_SERVER_PORT", "18090")))
    parser.add_argument("--places", default=os.environ.get(
        "ODIN_BRAIN_PLACES", "~/.config/odin_brain/places.json"))
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--nav-timeout", type=float, default=8.0)
    parser.add_argument("--dry-run", action="store_true", help="Default to dry-run mode")
    parser.add_argument("--mock", action="store_true", help="Use MockNav without ROS")
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--no-tts", action="store_true")
    args = parser.parse_args(argv)

    build_engine(args)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    _log("Listening on http://%s:%d  (/health /status /command)" % (args.host, args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
