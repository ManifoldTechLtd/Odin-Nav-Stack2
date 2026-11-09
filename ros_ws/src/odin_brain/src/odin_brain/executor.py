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

"""Plan executor: compiles a Plan into segments and executes them sequentially.

Segment types:
    mission       -- Consecutive goto steps dispatched in a single start_mission call
    atomic steps  -- speak / turn / move_rel / wait / ask_and_wait / home
"""

import json
import math
import time
from typing import Callable, Dict, List, Optional

from .event_bus import EventBus
from .models import Place, Plan, Step
from .plan_compiler import MISSION, compile_segments, describe_segments, is_pure_mission


class ExecResult(object):
    def __init__(self, ok: bool, state: str, message: str, mission_id: str = ""):
        self.ok = ok
        self.state = state
        self.message = message
        self.mission_id = mission_id

    def __repr__(self) -> str:
        return "ExecResult(ok=%s, state=%s, message=%s)" % (self.ok, self.state, self.message)


class Executor:
    def __init__(self, nav, dry_run: bool = False, verbose: bool = True,
                 max_timeout_sec: float = 900.0, event_bus: Optional[EventBus] = None,
                 speaker=None):
        self.nav = nav
        self.dry_run = dry_run
        self.verbose = verbose
        self.max_timeout_sec = max_timeout_sec
        self.event_bus = event_bus if event_bus is not None else EventBus()
        self.speaker = speaker
        self._announce = ""

    def run(self, plan: Plan, loop: bool = False,
            on_speak: Optional[Callable[[str], None]] = None) -> ExecResult:
        if not plan.steps:
            return ExecResult(False, "INVALID", "计划为空")

        segments = compile_segments(plan)
        self._announce = getattr(plan, "announce", "") or ""
        if self.verbose:
            print("[plan] %s" % describe_segments(segments))

        if is_pure_mission(segments):
            return self._run_mission(plan, loop, on_speak)
        return self._run_segments(segments, loop, on_speak)

    def _run_mission(self, plan: Plan, loop: bool,
                     on_speak: Optional[Callable[[str], None]] = None) -> ExecResult:
        waypoints = plan.waypoints()
        if not waypoints:
            return ExecResult(False, "INVALID", "计划里没有可执行的目的地")
        if self.dry_run:
            print("[dry ] Waypoints to dispatch (%d):" % len(waypoints))
            print(json.dumps(waypoints, ensure_ascii=False, indent=2))
            return ExecResult(True, "DRY_RUN", "dry-run 预览完成")

        if hasattr(self.nav, "precheck"):
            for wp in waypoints:
                feasible, reason, _ = self.nav.precheck(wp["x"], wp["y"], wp["yaw"])
                if not feasible:
                    msg = "去往「%s」的路径不可行（%s），已取消下发" % (wp["name"], reason)
                    print("[nav ] %s" % msg)
                    return ExecResult(False, "PRECHECK_FAILED", msg)

        ok, message, mission_id = self.nav.start_mission(plan, loop=loop)
        if not ok:
            return ExecResult(False, "REJECTED", "任务被拒绝：%s" % message)
        print("[nav ] Dispatched mission_id=%s (%d waypoints)" % (mission_id, len(waypoints)))
        state = self._wait_with_interrupts(mission_id, self._estimate_timeout(plan), loop)
        print("[nav ] Mission finished: %s" % state)
        ok_state = state in ("COMPLETED", "PAUSED")
        message = "任务结束状态 %s" % state if state != "PAUSED" else "已暂停"
        if ok_state and state == "COMPLETED" and self._announce and on_speak is not None:
            places = "、".join(step.label for step in plan.goto_steps()) or "目的地"
            text = self._announce.replace("{place}", places)
            if text:
                print("[say ] %s" % text)
                try:
                    on_speak(text)
                except Exception:
                    pass
        return ExecResult(ok_state, state, message, mission_id)

    def _wait_with_interrupts(self, mission_id: str, timeout_sec: float, loop: bool) -> str:
        """Wait for mission completion while handling cancel, pause, and preempt_temp interrupts.

        Filters terminal states by mission_id with a 5s initial grace period so stale
        terminal states from a previous mission are not mistaken for immediate completion.
        """
        deadline = time.time() + timeout_sec
        grace_until = time.time() + 5.0
        last_key = None
        while time.time() < deadline:
            action, payload = self.event_bus.pop_interrupt()
            if action == "cancel":
                print("[int ] Cancel requested -> cancelling mission")
                self.nav.trigger("cancel")
                return "CANCELLED"
            if action == "pause":
                print("[int ] Pause requested")
                self.nav.trigger("pause")
                paused = self._wait_resume_or_cancel()
                if paused != "RESUMED":
                    return paused
                print("[int ] Resumed current mission")
                continue
            if action == "preempt_temp" and payload is not None:
                self._run_temp_mission(payload, mission_id, loop)

            snapshot = self.nav.snapshot()
            events = self.nav.drain_events() if hasattr(self.nav, "drain_events") else []
            key = (snapshot.get("state"), snapshot.get("idx"))
            if events or key != last_key:
                self._on_tick(snapshot, events)
                last_key = key
            state = snapshot.get("state")
            current_id = str(snapshot.get("mission_id") or "")
            if state in ("COMPLETED", "FAILED", "CANCELLED", "PAUSED"):
                if not current_id or current_id == mission_id or time.time() > grace_until:
                    return state
            time.sleep(0.1)
        return "TIMEOUT"

    def _wait_resume_or_cancel(self, timeout_sec: float = 300.0) -> str:
        """Block while paused until resumed (RESUMED), cancelled (CANCELLED), or timed out."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            action, _payload = self.event_bus.pop_interrupt()
            if action == "resume":
                print("[int ] Resume requested -> resuming mission")
                self.nav.trigger("resume")
                return "RESUMED"
            if action == "cancel":
                print("[int ] Cancel requested")
                self.nav.trigger("cancel")
                return "CANCELLED"
            state = self.nav.snapshot().get("state")
            if state == "NAVIGATING":
                print("[int ] Navigation state returned to NAVIGATING -> continuing")
                return "RESUMED"
            if state in ("CANCELLED", "FAILED", "COMPLETED"):
                return state
            time.sleep(0.1)
        print("[int ] Pause wait timed out (%.0fs)" % timeout_sec)
        return "PAUSE_TIMEOUT"

    def _run_temp_mission(self, temp_plan: Plan, main_mission_id: str, loop: bool) -> bool:
        """Execute a temporary preempting mission: pause -> start_mission(temporary=True) -> wait for PAUSED on main_mission_id -> resume."""
        print("[int ] Preempting: pausing main mission and dispatching temporary mission")
        self.nav.trigger("pause")
        ok, message, temp_id = self.nav.start_mission(temp_plan, loop=loop, temporary=True)
        if not ok:
            print("[int ] Temporary mission rejected (%s), resuming main mission" % message)
            self.nav.trigger("resume")
            return False
        print("[int ] Temporary mission %s dispatched, waiting for completion" % temp_id)
        deadline = time.time() + 300.0
        while time.time() < deadline:
            snapshot = self.nav.snapshot()
            state = snapshot.get("state")
            mission = str(snapshot.get("mission_id") or "")
            if state == "PAUSED" and mission == main_mission_id:
                print("[int ] Temporary mission finished, main mission back to PAUSED -> resuming")
                self.nav.trigger("resume")
                return True
            time.sleep(0.25)
        print("[int ] Temporary mission timed out, attempting to resume main mission")
        self.nav.trigger("resume")
        return False

    def _run_segments(self, segments: List[Dict], loop: bool,
                      on_speak: Optional[Callable[[str], None]]) -> ExecResult:
        total = len(segments)
        for index, segment in enumerate(segments, 1):
            kind = segment["type"]
            if self.verbose:
                print("[seq ] (%d/%d) %s" % (index, total, self._segment_label(segment)))
            if kind == MISSION:
                result = self._run_mission(Plan(steps=segment["steps"]), loop, on_speak)
            elif kind == "home":
                result = self._run_home()
            elif kind == "speak":
                result = self._run_speak(segment["step"], on_speak)
            elif kind == "turn":
                result = self._run_turn(segment["step"])
            elif kind == "move_rel":
                result = self._run_move_rel(segment["step"])
            elif kind == "wait":
                result = self._run_wait(segment["step"])
            elif kind == "ask_and_wait":
                result = self._run_ask_and_wait(segment["step"], loop, on_speak)
            elif kind == "som_goto":
                result = self._run_mission(Plan(steps=[segment["step"]]), loop, on_speak)
            else:
                result = ExecResult(False, "INVALID", "未知步骤类型：%s" % kind)
            if not result.ok:
                print("[seq ] Step %d failed (%s), aborting remaining steps" % (index, result.message))
                return result
        return ExecResult(True, "COMPLETED", "全部 %d 步完成" % total)

    @staticmethod
    def _segment_label(segment: Dict) -> str:
        if segment["type"] == MISSION:
            return "导航：" + " → ".join(segment["labels"])
        return segment["step"].label

    def _run_speak(self, step: Step, on_speak: Optional[Callable[[str], None]]) -> ExecResult:
        text = step.text or ""
        if not text:
            return ExecResult(True, "SPOKEN", "空文本")
        print("[say ] %s" % text)
        if on_speak is not None:
            on_speak(text)
        if self.dry_run:
            return ExecResult(True, "DRY_RUN", "预览：说「%s」" % text)
        return ExecResult(True, "SPOKEN", text)

    def _run_turn(self, step: Step) -> ExecResult:
        angle_deg = float(step.angle_deg)
        if self.dry_run:
            print("[dry ] Turn in place %.0f deg" % angle_deg)
            return ExecResult(True, "DRY_RUN", "预览：转 %.0f°" % angle_deg)
        if not hasattr(self.nav, "turn"):
            return ExecResult(False, "UNSUPPORTED", "当前导航后端不支持原地转向")
        ok, message = self.nav.turn(math.radians(angle_deg))
        print("[act ] Turn %.0f deg: %s" % (angle_deg, "ok" if ok else message))
        if not ok:
            return ExecResult(False, "TURN_FAILED", message)
        return ExecResult(True, "TURNED", "转 %.0f° 完成" % angle_deg)

    def _run_move_rel(self, step: Step) -> ExecResult:
        """Relative displacement converted into a map-frame navigation goal with obstacle avoidance."""
        distance = float(step.distance_m)
        if distance <= 0:
            return ExecResult(True, "SKIPPED", "距离为 0")
        pose = self.nav.get_pose() if hasattr(self.nav, "get_pose") else None
        if pose is None:
            return ExecResult(False, "NO_POSE", "取不到当前位姿（TF 不可用或未重定位）")

        yaw = float(pose["yaw"])
        direction = step.direction if step.direction in ("forward", "backward", "left", "right") else "forward"
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        if direction == "forward":
            ux, uy = cos_yaw, sin_yaw
        elif direction == "backward":
            ux, uy = -cos_yaw, -sin_yaw
        elif direction == "left":
            ux, uy = -sin_yaw, cos_yaw
        else:
            ux, uy = sin_yaw, -cos_yaw

        reachable = self._nearest_reachable(pose, ux, uy, yaw, distance)
        if reachable is None:
            return ExecResult(False, "PRECHECK_FAILED",
                              "%s 方向上有障碍物，无法移动（最近可行距离不足 0.4m）" % step.label)
        if reachable < distance - 1e-6:
            print("[act ] %s unreachable at full distance, backed off to %.1fm" % (step.label, reachable))
        distance = reachable

        target = Place(id="__rel__", name="位移目标",
                       x=float(pose["x"]) + ux * distance,
                       y=float(pose["y"]) + uy * distance, yaw=yaw)
        plan = Plan(steps=[Step(place=target, dwell_sec=0.0)])
        print("[act ] %s %.1fm -> (%.2f, %.2f)" % (step.label, distance, target.x, target.y))
        return self._run_mission(plan, loop=False)

    def _nearest_reachable(self, pose: Dict, ux: float, uy: float, yaw: float,
                           distance: float):
        """Back off from distance in 0.2m increments (down to 0.4m) until precheck succeeds."""
        if not hasattr(self.nav, "precheck"):
            return distance
        px, py = float(pose["x"]), float(pose["y"])
        probe = distance
        while probe >= 0.4 - 1e-6:
            ok, _reason, _dist = self.nav.precheck(px + ux * probe, py + uy * probe, yaw)
            if ok:
                return probe
            probe -= 0.2
        return None

    def _run_wait(self, step: Step) -> ExecResult:
        seconds = float(step.dwell_sec)
        if seconds <= 0:
            return ExecResult(True, "SKIPPED", "等待 0 秒")
        if self.dry_run:
            print("[dry ] Wait %.1fs" % seconds)
            return ExecResult(True, "DRY_RUN", "预览：等待 %.1fs" % seconds)
        print("[act ] Waiting %.1fs" % seconds)
        event = self.event_bus.wait(["INTERRUPT"], timeout_sec=seconds)
        if event is not None:
            return ExecResult(False, "INTERRUPTED", "等待被用户中断")
        return ExecResult(True, "WAITED", "已等待 %.1fs" % seconds)

    def _run_ask_and_wait(self, step: Step, loop: bool,
                          on_speak: Optional[Callable[[str], None]]) -> ExecResult:
        """Speak prompt -> listen for ASR keyword or timeout -> execute on_match or on_timeout branch."""
        if step.text:
            self._run_speak(Step(kind="speak", text=step.text), on_speak)
            if self.speaker is not None and hasattr(self.speaker, "wait_idle"):
                self.speaker.wait_idle(timeout_sec=15.0)
        if self.dry_run:
            print("[dry ] Wait for keywords %s (timeout %.0fs)" % (step.expect_keywords, step.timeout_sec))
            return ExecResult(True, "DRY_RUN", "预览条件等待")

        self.event_bus.clear_asr()
        self.event_bus.set_listening(True)
        keywords = [k.strip() for k in step.expect_keywords if k.strip()]
        print("[wait] Listening for voice keywords %s (timeout %.0fs)..." % (keywords, step.timeout_sec))
        event = self.event_bus.wait(["ASR", "INTERRUPT"], timeout_sec=step.timeout_sec)
        self.event_bus.set_listening(False)

        if event is not None and event.type == "INTERRUPT":
            return ExecResult(False, "INTERRUPTED", "任务被用户中断")

        matched = False
        heard = ""
        if event is not None and event.type == "ASR":
            heard = str(event.payload.get("text", ""))
            matched = any(kw in heard for kw in keywords) if keywords else bool(heard)

        if matched:
            print("[wait] Heard '%s' (matched) -> executing on_match (%d steps)" % (heard, len(step.on_match)))
            if not step.on_match:
                return ExecResult(True, "COMPLETED", "语音确认完成")
            return self.run(Plan(steps=step.on_match), loop=loop, on_speak=on_speak)

        print("[wait] Timeout (%.0fs) without keyword match -> executing on_timeout (%d steps)"
              % (step.timeout_sec, len(step.on_timeout)))
        if not step.on_timeout:
            return ExecResult(True, "TIMEOUT_BRANCH_EMPTY", "等待超时")
        return self.run(Plan(steps=step.on_timeout), loop=loop, on_speak=on_speak)

    def _run_home(self) -> ExecResult:
        if self.dry_run:
            print("[dry ] Return home")
            return ExecResult(True, "DRY_RUN", "预览：返航")
        ok, message = self.nav.trigger("return_home")
        print("[nav ] Return home: %s %s" % ("accepted" if ok else "failed", message))
        return ExecResult(ok, "RETURNING_HOME" if ok else "REJECTED", message)

    def _estimate_timeout(self, plan: Plan) -> float:
        total_m = 0.0
        total_dwell = 0.0
        previous = None
        for step in plan.goto_steps():
            if previous is not None:
                total_m += math.hypot(step.place.x - previous.x, step.place.y - previous.y)
            total_dwell += step.dwell_sec
            previous = step.place
        estimate = 30.0 + total_m / 0.25 + total_dwell
        return min(self.max_timeout_sec, max(60.0, estimate))

    def _on_tick(self, snapshot: Dict, events: List[Dict]) -> None:
        if not self.verbose:
            return
        for event in events:
            print("[evt ] %s %s" % (event.get("event", "?"), event.get("message", "")))
        state = snapshot.get("state", "?")
        if state == "NAVIGATING":
            dist = float(snapshot.get("dist_m", -1.0))
            dist_text = ("remaining %.2fm" % dist) if dist >= 0 else "distance unknown"
            print("[nav ] %s waypoint %d/%d, %s, progress %.0f%%" % (
                state,
                int(snapshot.get("idx", 0)) + 1,
                int(snapshot.get("total", 0)),
                dist_text,
                float(snapshot.get("progress", 0.0)),
            ))
