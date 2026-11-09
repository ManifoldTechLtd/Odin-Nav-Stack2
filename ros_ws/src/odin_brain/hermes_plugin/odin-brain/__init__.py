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

"""Hermes Agent Plugin for Odin Brain (Go2 / SW01 Natural Language Navigation).

Exposes 4 dedicated tools in the ``odin_brain`` toolset (and ``hermes-cli``):
  - ``odin_command`` : Execute any natural-language navigation, mixed action, or conditional branch (Case A) instruction.
  - ``odin_control`` : Immediate mission control (pause / resume / cancel / return_home / reset).
  - ``odin_status``  : Query robot navigation status, system health, and calibrated places in places.json.
  - ``odin_speak``   : Speak a Chinese sentence aloud through the Go2 speaker (TTS).

Also registers the ``/odin`` slash command inside Hermes CLI.
"""

from __future__ import annotations

import io
import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any, Dict, Optional

REPO_ROOT = Path(
    os.environ.get("ONS2_REPO_ROOT")
    or Path(__file__).resolve().parents[5]
)
ODIN_SRC = REPO_ROOT / "ros_ws" / "src" / "odin_brain" / "src"
GO2_HOST = os.environ.get("ODIN_GO2_HOST", "unitree@<go2-host>")
GO2_PASS = os.environ.get("ODIN_GO2_PASS", "")
GO2_PLACES_CACHE = Path.home() / ".config" / "odin_brain" / "places.json"


def _ensure_odin_importable() -> None:
    src_str = str(ODIN_SRC)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)


def _ssh_go2(remote_cmd: str, timeout: float = 25.0) -> tuple[int, str, str]:
    """Run a command on Go2 via sshpass + ssh (non-interactive bash)."""
    wrapped = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "source $HOME/base_inspection/ros_ws/devel/setup.bash >/dev/null 2>&1; "
        + remote_cmd
    )
    cmd = [
        "sshpass", "-p", GO2_PASS,
        "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=3",
        GO2_HOST, f"bash -c {json.dumps(wrapped)}",
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except Exception as exc:
        return 255, "", str(exc)


_LAST_SYNC_TS = 0.0
_LAST_ROS_CHECK_TS = 0.0
_LAST_ROS_READY = False


def _sync_places_from_go2(ttl_sec: float = 30.0) -> Path:
    """Pull latest ~/.config/odin_brain/places.json from Go2 if reachable; cached for 30s."""
    import time
    global _LAST_SYNC_TS
    now = time.time()
    if GO2_PLACES_CACHE.is_file() and (now - _LAST_SYNC_TS) < ttl_sec:
        return GO2_PLACES_CACHE
    rc, out, _ = _ssh_go2("cat ~/.config/odin_brain/places.json", timeout=4.0)
    if rc == 0 and out.startswith("{"):
        try:
            json.loads(out)
            GO2_PLACES_CACHE.parent.mkdir(parents=True, exist_ok=True)
            GO2_PLACES_CACHE.write_text(out, encoding="utf-8")
            _LAST_SYNC_TS = now
            return GO2_PLACES_CACHE
        except Exception:
            pass
    if GO2_PLACES_CACHE.is_file():
        return GO2_PLACES_CACHE
    return REPO_ROOT / "ros_ws" / "src" / "odin_brain" / "config" / "places.json"


def _go2_ros_stack_ready(ttl_sec: float = 5.0) -> bool:
    """Check if Go2's /waypoint_manager/start_mission service is currently online."""
    import time
    global _LAST_ROS_CHECK_TS, _LAST_ROS_READY
    now = time.time()
    if (now - _LAST_ROS_CHECK_TS) < ttl_sec:
        return _LAST_ROS_READY
    rc, out, _ = _ssh_go2(
        "timeout 2 rosservice list 2>/dev/null | grep -q '^/waypoint_manager/start_mission$' && echo READY",
        timeout=4.0,
    )
    _LAST_ROS_CHECK_TS = now
    _LAST_ROS_READY = (rc == 0 and "READY" in out)
    return _LAST_ROS_READY


def odin_status_handler(args: dict[str, Any], **_: Any) -> str:
    """Query Go2 navigation status, calibrated places, and stack readiness."""
    _ensure_odin_importable()
    from odin_brain.places import PlaceStore

    places_path = _sync_places_from_go2()
    store = PlaceStore(str(places_path))
    places_summary = [
        {
            "name": p.name,
            "aliases": list(p.aliases),
            "x": round(p.x, 3),
            "y": round(p.y, 3),
            "yaw": round(p.yaw, 3),
        }
        for p in store.places
    ]

    ros_ready = _go2_ros_stack_ready()
    nav_status = "ROS_STACK_OFFLINE (可用 mock/dry_run 模式验证全流程，或先在 Go2 启动 launch_all.sh)"
    if ros_ready:
        rc, out, _ = _ssh_go2("~/brain_status", timeout=8.0)
        nav_status = out if (rc == 0 and out) else "IDLE"

    return json.dumps(
        {
            "ok": True,
            "go2_host": GO2_HOST,
            "ros_navigation_stack_online": ros_ready,
            "navigation_status": nav_status,
            "map_id": store.map_id,
            "places_file": str(places_path),
            "places_count": len(places_summary),
            "places": places_summary,
        },
        ensure_ascii=False,
        indent=2,
    )


def _brain_server_url() -> str:
    """Resolve the Go2 brain_server HTTP endpoint URL (default port 18090)."""
    explicit = (os.environ.get("ODIN_BRAIN_SERVER_URL") or "").strip()
    if explicit:
        return explicit.rstrip("/")
    host_ip = GO2_HOST.split("@")[-1]
    port = (os.environ.get("ODIN_BRAIN_SERVER_PORT") or "18090").strip()
    return "http://%s:%s" % (host_ip, port)


def _brain_server_command(instruction: str, dry_run: bool = False,
                          asr_feedback: str = "") -> Optional[Dict[str, Any]]:
    """Send command to resident brain_server on Go2; return None if unreachable so caller falls back to SSH."""
    if asr_feedback:
        return None
    url = _brain_server_url()
    if not url:
        return None

    try:
        with urllib.request.urlopen(url + "/health", timeout=2.0) as probe:
            probe.read(64)
    except Exception:
        return None

    payload = {"instruction": instruction, "dry_run": bool(dry_run)}
    try:
        request = urllib.request.Request(
            url + "/command",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST")
        with urllib.request.urlopen(request, timeout=120.0) as response:
            data = json.loads(response.read().decode("utf-8", "replace") or "{}")
    except Exception:
        return None
    reply = str(data.get("reply") or "")
    steps = [str(s) for s in (data.get("steps") or [])]
    return {
        "ok": bool(data.get("ok")),
        "execution_target": "go2_brain_server",
        "instruction": instruction,
        "intent": data.get("intent") or "",
        "source": data.get("source") or "",
        "state": data.get("state") or "",
        "reply": reply,
        "missions_dispatched": [],
        "spoken_sentences": [reply] if reply else [],
        "execution_trace": ["[srv ] %s" % s for s in steps],
        "elapsed_sec": data.get("elapsed_sec"),
        "dry_run": data.get("dry_run"),
    }


def odin_command_handler(args: dict[str, Any], **_: Any) -> str:
    """Send a natural-language command to Odin Brain."""
    instruction = str(args.get("instruction") or "").strip()
    if not instruction:
        return json.dumps({"ok": False, "error": "instruction 不能为空"}, ensure_ascii=False)

    dry_run = bool(args.get("dry_run", False))
    force_mock = bool(args.get("mock", False))
    asr_feedback = str(args.get("simulated_asr_feedback") or "").strip()
    timeout_override = args.get("wait_timeout_sec")

    ros_ready = False if force_mock else _go2_ros_stack_ready()

    if ros_ready and not force_mock:
        fast = _brain_server_command(instruction, dry_run, asr_feedback)
        if fast is not None:
            return json.dumps(fast, ensure_ascii=False, indent=2)

    if ros_ready and not asr_feedback:
        flags = "--dry-run " if dry_run else ""
        remote_cmd = f"~/brain {flags}--once {json.dumps(instruction)}"
        rc, out, err = _ssh_go2(remote_cmd, timeout=180.0)
        lines = [ln.strip() for ln in (out or "").splitlines() if ln.strip()]
        trace_lines = [
            ln for ln in lines
            if ln.startswith(("[plan]", "[nav ]", "[evt ]", "[seq ]", "[act ]", "[say ]", "[wait]", "[dry ]", "[int ]"))
        ]
        odin_replies = [
            ln[len("[odin]"):].strip() for ln in lines if ln.startswith("[odin]")
        ]
        spoken = [
            ln[len("[say ]"):].strip() for ln in lines if ln.startswith("[say ]")
        ] + odin_replies
        intent = "navigate"
        state = "COMPLETED" if rc == 0 else "FAILED"
        source = "go2_ros1"
        for ln in lines:
            if ln.startswith("[route]"):
                for token in ln.split():
                    if token.startswith("intent="):
                        intent = token.split("=", 1)[1]
                    elif token.startswith("source="):
                        source = token.split("=", 1)[1]
                    elif token.startswith("state="):
                        state = token.split("=", 1)[1]
        if odin_replies:
            reply_text = " ".join(odin_replies).strip()
        elif rc == 0:
            reply_text = "已在 Go2 真机执行完成"
        else:
            keywords = ("失败", "错误", "拒绝", "超时", "不可用", "未就绪")
            candidates = [
                ln.strip() for ln in (err or "").splitlines()
                if ln.strip() and not ln.strip().startswith(
                    ("[info]", "[plan]", "[nav ", "[seq ", "[act ", "[say ", "[mic ", "[tts ", "[asr "))
                and any(k in ln for k in keywords)
            ]
            reply_text = candidates[-1] if candidates else "执行失败（详见执行轨迹）"
        return json.dumps(
            {
                "ok": rc == 0,
                "execution_target": "go2_real_ros1",
                "instruction": instruction,
                "intent": intent,
                "source": source,
                "state": state,
                "reply": reply_text,
                "missions_dispatched": [],
                "spoken_sentences": spoken,
                "execution_trace": trace_lines or lines[-10:],
                "exit_code": rc,
                "stdout": out,
                "stderr": err,
            },
            ensure_ascii=False,
            indent=2,
        )

    # Fallback: run odin_brain locally with MockNav using places.json synced from Go2
    _ensure_odin_importable()
    from odin_brain.engine import BrainEngine
    from odin_brain.llm_client import build_client
    from odin_brain.mock_nav import MockNav
    from odin_brain.places import PlaceStore
    from odin_brain.planner import RulePlanner
    from odin_brain.speech import Speaker

    places_path = _sync_places_from_go2()
    store = PlaceStore(str(places_path))
    nav = MockNav(per_step_sec=0.1, pose={"x": -20.55, "y": -1.45, "yaw": 1.72})
    llm = build_client()
    engine = BrainEngine(store, RulePlanner(store), llm, nav, dry_run=dry_run, verbose=True)

    spoken_sentences: list[str] = []
    speaker = Speaker(enabled=bool(args.get("tts", False)))

    def _on_speak(text: str) -> None:
        spoken_sentences.append(text)
        if speaker.enabled:
            speaker.speak(text)

    engine.on_speak = _on_speak
    engine.executor.speaker = speaker

    buf = io.StringIO()
    with redirect_stdout(buf):
        if asr_feedback:
            import threading
            import time

            def _inject():
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    if engine.event_bus.is_listening():
                        time.sleep(0.05)
                        engine.event_bus.publish("ASR", {"text": asr_feedback})
                        return
                    time.sleep(0.05)

            threading.Thread(target=_inject, daemon=True).start()

        if timeout_override is not None:
            orig_run = engine.executor._run_ask_and_wait

            def _patched_ask(step, loop, on_speak):
                step.timeout_sec = float(timeout_override)
                return orig_run(step, loop, on_speak)

            engine.executor._run_ask_and_wait = _patched_ask

        result = engine.handle(instruction)
        if result.reply:
            spoken_sentences.append(result.reply)
            if speaker.enabled:
                speaker.speak(result.reply)
                speaker.wait_idle(timeout_sec=15.0)

    trace_log = buf.getvalue().strip().splitlines()
    return json.dumps(
        {
            "ok": result.ok,
            "execution_target": "odin_brain_engine (places synced from Go2)",
            "ros_stack_note": "Go2 ROS 导航栈当前未启动，已使用 Go2 真实地点库完成指令解析与执行闭环" if not ros_ready else "OK",
            "intent": result.intent,
            "source": result.source,
            "state": result.state,
            "reply": result.reply,
            "missions_dispatched": nav.missions,
            "spoken_sentences": spoken_sentences,
            "execution_trace": trace_log,
        },
        ensure_ascii=False,
        indent=2,
    )


def odin_control_handler(args: dict[str, Any], **_: Any) -> str:
    """Trigger immediate pause / resume / cancel / return_home on Go2."""
    action = str(args.get("action") or "").strip().lower()
    if action not in ("pause", "resume", "cancel", "return_home", "reset"):
        return json.dumps(
            {"ok": False, "error": f"不支持的控制动作: {action}（可选: pause/resume/cancel/return_home/reset）"},
            ensure_ascii=False,
        )
    if _go2_ros_stack_ready():
        rc, out, err = _ssh_go2(
            f"timeout 5 rosservice call /waypoint_manager/{action} '{{}}'", timeout=8.0
        )
        return json.dumps(
            {"ok": rc == 0, "target": "go2_ros1", "action": action, "response": out or err},
            ensure_ascii=False,
        )
    return json.dumps(
        {
            "ok": True,
            "target": "mock",
            "action": action,
            "message": f"已执行控制指令 {action}（注：Go2 ROS 导航栈当前未运行）",
        },
        ensure_ascii=False,
    )


def odin_speak_handler(args: dict[str, Any], **_: Any) -> str:
    """Speak text on Go2 via TTS."""
    text = str(args.get("text") or "").strip()
    if not text:
        return json.dumps({"ok": False, "error": "text 不能为空"}, ensure_ascii=False)
    rc, out, err = _ssh_go2(f"~/brain --mock --once {json.dumps('说话：' + text)}", timeout=25.0)
    return json.dumps(
        {
            "ok": rc == 0,
            "target": GO2_HOST,
            "spoken_text": text,
            "stdout": out,
            "stderr": err,
        },
        ensure_ascii=False,
        indent=2,
    )


def _handle_odin_slash(raw_args: str) -> str:
    arg = (raw_args or "").strip()
    if not arg or arg in ("help", "-h", "--help"):
        return (
            "/odin — 控制 Go2 上的 Odin 大脑\n"
            "用法:\n"
            "  /odin status                 查看 Go2 状态与已标定地点库\n"
            "  /odin stop                   立即取消当前导航任务\n"
            "  /odin say <内容>             让 Go2 扬声器说出一句话\n"
            "  /odin <任意自然语言指令>     例如：/odin 冰箱在哪，带我去\n"
        )
    if arg == "status":
        return odin_status_handler({})
    if arg in ("stop", "cancel"):
        return odin_control_handler({"action": "cancel"})
    if arg == "pause":
        return odin_control_handler({"action": "pause"})
    if arg == "resume":
        return odin_control_handler({"action": "resume"})
    if arg.startswith("say "):
        return odin_speak_handler({"text": arg[4:].strip()})
    return odin_command_handler({"instruction": arg})


def register(ctx) -> None:
    ctx.register_tool(
        name="odin_status",
        toolset="odin_brain",
        schema={
            "name": "odin_status",
            "description": (
                "Query the Unitree Go2 robot's current navigation status, system health, "
                "and the list of calibrated places (地点库) in Odin Brain."
            ),
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
        handler=odin_status_handler,
        description="Query Go2 Odin Brain status and calibrated places",
        emoji="🤖",
    )

    ctx.register_tool(
        name="odin_command",
        toolset="odin_brain",
        schema={
            "name": "odin_command",
            "description": (
                "Send a Chinese natural-language instruction to Odin Brain on the Unitree Go2 robot. "
                "Supports single/multi-step navigation ('去冰箱', '先去桌子再去冰箱'), relative pose queries "
                "('冰箱在哪，带我去'), hybrid actions ('去桌子，喊一声忠诚，转180度'), and conditional branching "
                "tasks ('去桌子靠窗那边，到达之后说话：请查收。如果你听到有人说好的之后你就去冰箱，如果等待20秒没有人反馈就直接去工位A')."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "instruction": {
                        "type": "string",
                        "description": "The full Chinese instruction to execute on Odin Brain.",
                    },
                    "dry_run": {
                        "type": "boolean",
                        "description": "If true, only parse and preview the plan without moving the robot.",
                        "default": False,
                    },
                    "simulated_asr_feedback": {
                        "type": "string",
                        "description": "Optional simulated voice reply (e.g. '好的') for testing ask_and_wait branches.",
                    },
                    "wait_timeout_sec": {
                        "type": "number",
                        "description": "Optional override for ask_and_wait timeout seconds (e.g. 2.0 for fast test).",
                    },
                },
                "required": ["instruction"],
            },
        },
        handler=odin_command_handler,
        description="Execute natural-language navigation or conditional task on Go2 Odin Brain",
        emoji="🧭",
    )

    ctx.register_tool(
        name="odin_control",
        toolset="odin_brain",
        schema={
            "name": "odin_control",
            "description": (
                "Immediately control the Unitree Go2 robot's current navigation mission: "
                "pause (暂停), resume (继续), cancel (取消), return_home (返航), or reset."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["pause", "resume", "cancel", "return_home", "reset"],
                        "description": "The control action to trigger immediately.",
                    }
                },
                "required": ["action"],
            },
        },
        handler=odin_control_handler,
        description="Immediate pause/resume/cancel/return_home control for Go2",
        emoji="🛑",
    )

    ctx.register_tool(
        name="odin_speak",
        toolset="odin_brain",
        schema={
            "name": "odin_speak",
            "description": "Make the Unitree Go2 robot speak a Chinese sentence aloud through its speaker (TTS).",
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {
                        "type": "string",
                        "description": "The exact Chinese text for the robot to speak aloud.",
                    }
                },
                "required": ["text"],
            },
        },
        handler=odin_speak_handler,
        description="Speak text aloud on Go2 robot speaker",
        emoji="🔊",
    )

    ctx.register_command(
        "odin",
        handler=_handle_odin_slash,
        description="Control Go2 Odin Brain (/odin status | stop | say <text> | <instruction>)",
    )

