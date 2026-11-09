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

"""FastAPI backend routes for the Odin Brain Dashboard Plugin (/api/plugins/odin-brain/*)."""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, PlainTextResponse

PLUGIN_ROOT = Path(__file__).resolve().parent.parent
if str(PLUGIN_ROOT) not in sys.path:
    sys.path.insert(0, str(PLUGIN_ROOT))

from __init__ import (  # type: ignore
    odin_command_handler,
    odin_control_handler,
    odin_speak_handler,
    odin_status_handler,
)

router = APIRouter()


@router.get("/status")
async def api_status():
    raw = await asyncio.to_thread(odin_status_handler, {})
    return JSONResponse(json.loads(raw))


@router.post("/command")
async def api_command(request: Request):
    try:
        body: Dict[str, Any] = await request.json()
    except Exception:
        body = {}

    instruction = str(body.get("instruction") or "").strip()
    if not instruction:
        return JSONResponse({"ok": False, "error": "指令不能为空"}, status_code=400)

    via_hermes = bool(body.get("via_hermes", False))
    if via_hermes:
        # Execute via Hermes Agent CLI toolset
        def _run_hermes() -> Dict[str, Any]:
            hermes_bin = os.path.expanduser("~/.local/bin/hermes")
            prompt = f"请立即调用 odin_command 工具执行以下机器人指令，并汇报执行结果：{instruction}"
            proc = subprocess.run(
                [hermes_bin, "--cli", "-t", "odin_brain", "-z", prompt],
                capture_output=True,
                text=True,
                timeout=90,
            )
            return {
                "ok": proc.returncode == 0,
                "execution_target": "hermes_agent (glm-4-flash -> odin_brain)",
                "intent": "hermes_agent",
                "source": "glm-4-flash-250414",
                "state": "COMPLETED" if proc.returncode == 0 else "FAILED",
                "reply": (proc.stdout or proc.stderr or "").strip(),
                "missions_dispatched": [],
                "spoken_sentences": [],
                "execution_trace": [
                    f"[hermes] model=glm-4-flash-250414 toolset=odin_brain",
                    f"[input ] {instruction}",
                    f"[reply ] {(proc.stdout or proc.stderr or '').strip()}",
                ],
            }

        res = await asyncio.to_thread(_run_hermes)
        return JSONResponse(res)

    raw = await asyncio.to_thread(odin_command_handler, body)
    return JSONResponse(json.loads(raw))


@router.post("/control")
async def api_control(request: Request):
    try:
        body: Dict[str, Any] = await request.json()
    except Exception:
        body = {}
    raw = await asyncio.to_thread(odin_control_handler, body)
    return JSONResponse(json.loads(raw))


@router.post("/speak")
async def api_speak(request: Request):
    try:
        body: Dict[str, Any] = await request.json()
    except Exception:
        body = {}
    raw = await asyncio.to_thread(odin_speak_handler, body)
    return JSONResponse(json.loads(raw))


@router.post("/transcribe")
async def api_transcribe(request: Request):
    """Receive base64 audio from browser MediaRecorder, inject Go2 place hotwords, and transcribe via local asr_server (18082)."""
    import urllib.request
    from __init__ import _ensure_odin_importable, _sync_places_from_go2  # type: ignore

    try:
        body: Dict[str, Any] = await request.json()
    except Exception:
        body = {}

    audio_b64 = str(body.get("audio_b64") or "").strip()
    if not audio_b64:
        return JSONResponse({"ok": False, "error": "audio_b64 不能为空"}, status_code=400)

    def _do_asr() -> Dict[str, Any]:
        import base64
        import tempfile
        _ensure_odin_importable()
        from odin_brain.places import PlaceStore

        try:
            store = PlaceStore(str(_sync_places_from_go2()))
            hotwords = store.hotwords()
        except Exception:
            hotwords = []

        raw_b64 = audio_b64
        if "," in raw_b64 and raw_b64.startswith("data:"):
            raw_b64 = raw_b64.split(",", 1)[1]
        raw_bytes = base64.b64decode(raw_b64)

        # Convert browser webm/ogg audio to 16kHz mono WAV via ffmpeg before sending to ASR server
        if not (len(raw_bytes) >= 12 and raw_bytes[:4] == b"RIFF" and raw_bytes[8:12] == b"WAVE"):
            in_fd, in_path = tempfile.mkstemp(prefix="web_mic_", suffix=".webm")
            out_fd, out_path = tempfile.mkstemp(prefix="web_mic_", suffix=".wav")
            os.close(in_fd)
            os.close(out_fd)
            try:
                with open(in_path, "wb") as fh:
                    fh.write(raw_bytes)
                subprocess.run(
                    ["ffmpeg", "-y", "-i", in_path, "-ar", "16000", "-ac", "1", "-f", "wav", out_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=15,
                    check=True,
                )
                with open(out_path, "rb") as fh:
                    raw_bytes = fh.read()
            finally:
                for p in (in_path, out_path):
                    try:
                        os.unlink(p)
                    except OSError:
                        pass

        clean_b64 = base64.b64encode(raw_bytes).decode("ascii")
        asr_url = os.environ.get("ODIN_ASR_URL", "http://127.0.0.1:18082/asr")
        payload = json.dumps({"audio_b64": clean_b64, "hotwords": hotwords}, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            asr_url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=20.0) as resp:
                return json.loads(resp.read().decode("utf-8", "replace"))
        except Exception as exc:
            return {
                "ok": False,
                "error": f"本地 ASR 服务 ({asr_url}) 未响应: {exc}。请先运行 python3 scripts/brain/asr_server.py --port 18082",
            }

    res = await asyncio.to_thread(_do_asr)
    return JSONResponse(res)


# Semantic waypoint store endpoints


def _places_snapshot():
    """Sync places.json from Go2 and return (path, PlaceStore)."""
    from __init__ import _sync_places_from_go2  # type: ignore
    from odin_brain.places import PlaceStore

    path = _sync_places_from_go2()
    return path, PlaceStore(str(path))


def _place_to_dict(place) -> Dict[str, Any]:
    item: Dict[str, Any] = {
        "name": place.name,
        "x": round(place.x, 6),
        "y": round(place.y, 6),
        "yaw": round(place.yaw, 6),
        "yaw_deg": round(place.yaw * 180.0 / 3.141592653589793, 2),
        "aliases": list(place.aliases or []),
        "dwell_default_sec": getattr(place, "dwell_default_sec", 0.0),
        "notes": getattr(place, "notes", ""),
    }
    if getattr(place, "z", 0.0):
        item["z"] = round(place.z, 6)
    return item


@router.get("/places")
async def api_places(format: str = "json"):
    """Read semantic waypoint store as JSON or Q9000 plain text."""
    try:
        path, store = await asyncio.to_thread(_places_snapshot)
    except Exception as exc:  # noqa: BLE001
        return JSONResponse({"ok": False, "error": "读取地点库失败: %s" % exc}, status_code=500)

    if (format or "").lower() == "q9000":
        from odin_brain import q9000_waypoints as q9

        items = [{
            "name": p.name,
            "pose": {"x": p.x, "y": p.y, "yaw": p.yaw,
                     **({"z": p.z} if getattr(p, "z", 0.0) else {})},
            "aliases": list(p.aliases or []),
            "dwell_default_sec": getattr(p, "dwell_default_sec", 0.0),
            "notes": getattr(p, "notes", ""),
        } for p in store.places]
        return PlainTextResponse(q9.dump(q9.from_places(items)))

    return JSONResponse({
        "ok": True,
        "source": "go2_places_json",
        "path": str(path),
        "map_id": store.map_id or "",
        "frame_id": store.frame_id or "map",
        "updated_at": store.updated_at or "",
        "count": len(store.places),
        "places": [_place_to_dict(p) for p in store.places],
    })


@router.post("/places/import")
async def api_places_import(request: Request):
    """Import Q9000 waypoint text (preview when commit=false; write and sync when commit=true)."""
    try:
        body = await request.json()
    except Exception as exc:  # noqa: BLE001
        return JSONResponse({"ok": False, "error": "请求体不是合法 JSON: %s" % exc}, status_code=400)

    content = str(body.get("content") or "")
    commit = bool(body.get("commit"))
    if not content.strip():
        return JSONResponse({"ok": False, "error": "content 为空"}, status_code=400)

    from odin_brain import q9000_waypoints as q9

    points = q9.parse(content)
    if not points:
        return JSONResponse({
            "ok": False,
            "error": "未解析到任何航点，检查格式：序号 名称 时间戳 x y z qw qx qy qz",
        }, status_code=400)

    preview = [{
        "seq": w.seq, "name": w.name, "x": round(w.x, 6), "y": round(w.y, 6),
        "z": round(w.z, 6), "yaw": round(w.yaw, 6),
        "yaw_deg": round(w.yaw * 180.0 / 3.141592653589793, 2),
    } for w in points]
    if not commit:
        return JSONResponse({
            "ok": True, "dry_run": True, "count": len(points), "points": preview,
            "hint": "确认无误后带 \"commit\": true 再提交",
        })

    def _write() -> Dict[str, Any]:
        import base64
        import time as _time

        from __init__ import _ssh_go2  # type: ignore

        path, store = _places_snapshot()
        existing = {p.name: p for p in store.places}
        for point in points:
            old = existing.get(point.name)
            if old is not None:
                point.aliases = list(old.aliases or [])
                point.dwell_default_sec = getattr(old, "dwell_default_sec", 0.0)
                point.notes = getattr(old, "notes", "")
        record = q9.to_places_record(points, map_id=store.map_id,
                                     frame_id=store.frame_id or "map")
        payload = json.dumps(record, ensure_ascii=False, indent=2).encode("utf-8")
        b64 = base64.b64encode(payload).decode("ascii")
        remote = "~/.config/odin_brain/places.json"
        stamp = _time.strftime("%Y%m%d_%H%M%S")
        rc_bak, _, err_bak = _ssh_go2("cp %s %s.bak_%s" % (remote, remote, stamp), timeout=15.0)
        rc, _, err = _ssh_go2("echo %s | base64 -d > %s" % (b64, remote), timeout=30.0)
        q9_rc, _, q9_err = _ssh_go2(
            "python3 - <<'PY' 2>&1 || true\n"
            "import os, sys\n"
            "sys.path.insert(0, os.path.expanduser('~/base_inspection/ros_ws/src/odin_brain/src'))\n"
            "from odin_brain import q9000_waypoints as q9\n"
            "from odin_brain.places import PlaceStore\n"
            "import os\n"
            "s = PlaceStore(os.path.expanduser('~/.config/odin_brain/places.json'))\n"
            "items = [{\"name\": p.name, \"pose\": {\"x\": p.x, \"y\": p.y, \"yaw\": p.yaw,\n"
            "          **({\"z\": p.z} if getattr(p, 'z', 0.0) else {})}} for p in s.places]\n"
            "q9.save(q9.default_path(os.path.expanduser('~/.config/odin_brain/places.json')),\n"
            "        q9.from_places(items))\n"
            "print('mirror ok')\n"
            "PY", timeout=40.0)
        return {
            "rc": rc,
            "error": err if rc != 0 else "",
            "backup_rc": rc_bak,
            "backup_error": err_bak if rc_bak != 0 else "",
            "remote": remote,
            "local_cache": str(path),
            "mirror_ok": q9_rc == 0,
            "mirror_error": q9_err if q9_rc != 0 else "",
            "count": len(record.get("places") or []),
        }

    result = await asyncio.to_thread(_write)
    ok = result.get("rc") == 0
    return JSONResponse({
        "ok": ok,
        "committed": True,
        "count": result.get("count"),
        "remote": result.get("remote"),
        "backup_ok": result.get("backup_rc") == 0,
        "q9000_mirror_ok": result.get("mirror_ok"),
        "error": result.get("error") or "",
    }, status_code=200 if ok else 500)


