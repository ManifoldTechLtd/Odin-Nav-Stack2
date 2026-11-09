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

"""Core BrainEngine orchestrating intent routing, dialog, navigation, control, and status queries."""

from dataclasses import dataclass, field
import threading
from typing import Callable, Dict, Optional

from . import prompts
from .event_bus import EventBus
from .executor import Executor
from .llm_client import LLMClient, LLMError, LLMUnavailable
from .models import Plan
from .places import PlaceStore, describe_relative_pose
from .planner import PlanError, RulePlanner
from .router import RouteResult, Router
from .session import Session

CONTROL_REPLY = {
    "pause": "好，停下了",
    "resume": "继续走",
    "cancel": "已取消当前任务",
    "return_home": "正在返回起点",
    "reset": "任务已清空",
}

_WHERE_HINTS = ("在哪", "什么位置", "哪个方向", "多远", "怎么走")
_HEALTH_HINTS = ("怎么停", "为什么", "正常吗", "健康", "出什么问题")
_QUICK_QUERY_HINTS = ("到哪", "多远", "还有", "状态", "怎么停", "正常吗", "什么情况", "进度")


@dataclass
class EngineResult:
    intent: str = "chat"
    reply: str = ""
    ok: bool = True
    state: str = ""
    mission_id: str = ""
    plan: Optional[Plan] = None
    source: str = "rule"
    slots: Dict[str, str] = field(default_factory=dict)

    def plan_payload(self) -> list:
        if self.plan is None:
            return []
        payload = []
        for step in self.plan.steps:
            if step.kind == "home":
                payload.append({"kind": "home"})
            else:
                payload.append({
                    "kind": "goto",
                    "place": step.place.name,
                    "x": step.place.x,
                    "y": step.place.y,
                    "yaw": step.place.yaw,
                    "dwell_sec": step.dwell_sec,
                })
        return payload


class BrainEngine(object):
    def __init__(self, store: PlaceStore, planner: RulePlanner,
                 llm: Optional[LLMClient], nav,
                 dry_run: bool = False, verbose: bool = True, loop: bool = False,
                 session: Optional[Session] = None):
        self.store = store
        self.planner = planner
        self.llm = llm
        self.nav = nav
        self.dry_run = dry_run
        self.verbose = verbose
        self.loop = loop
        self.session = session or Session()
        self.router = Router(store, planner, llm)
        self.event_bus = EventBus()
        self.executor = Executor(nav, dry_run=dry_run, verbose=verbose, event_bus=self.event_bus)
        self.on_speak: Optional[Callable[[str], None]] = None
        self._task_thread = None

    def handle(self, text: str, on_token: Optional[Callable[[str], None]] = None) -> EngineResult:
        text = (text or "").strip()
        if not text:
            return EngineResult(intent="chat", reply="", ok=False)
        hint = self._state_hint()
        route = self.router.route(text, hint)
        self.session.note(route.intent, route.place)

        custom = self.router.handler(route.intent)
        if custom is not None:
            reply = custom(route, text)
            result = EngineResult(intent=route.intent, reply=reply or "",
                                  source="custom", slots=route.slots)
        elif route.intent == "navigate":
            result = self._navigate(route, text, hint)
        elif route.intent == "control":
            result = self._control(route)
        elif route.intent == "query":
            result = self._query(route, text)
        elif route.intent == "other":
            result = self._other(route, text)
        else:
            result = self._chat(route, text, hint, on_token)

        result.source = route.source if result.source in ("rule", "") else result.source
        self.session.add_user(text)
        if result.reply:
            self.session.add_assistant(result.reply)
        return result

    @property
    def busy(self) -> bool:
        """True when a background navigation or composite task is currently executing."""
        return self._task_thread is not None and self._task_thread.is_alive()

    def handle_async(self, text: str, on_token: Optional[Callable[[str], None]] = None,
                     on_done: Optional[Callable[["EngineResult"], None]] = None):
        """Execute a command in a background thread and return the thread handle immediately."""
        def _run():
            try:
                result = self.handle(text, on_token)
            except Exception as exc:
                result = EngineResult(intent="chat", reply="执行出错：%s" % exc, ok=False)
            if on_done is not None:
                try:
                    on_done(result)
                except Exception:
                    pass

        self._task_thread = threading.Thread(target=_run, daemon=True, name="odin-task")
        self._task_thread.start()
        return self._task_thread

    def handle_fast(self, text: str) -> Optional[EngineResult]:
        """Fast-path handler while a navigation task is active (control, status query, or temporary preemption)."""
        text = (text or "").strip()
        if not text:
            return None

        action = Router._match_control(text)
        if action:
            self.event_bus.request_interrupt(action)
            return self._control(RouteResult(intent="control", slots={"action": action},
                                             source="rule"))

        if any(word in text for word in _QUICK_QUERY_HINTS):
            return self._query(RouteResult(intent="query", source="rule"), text)

        try:
            plan = self.planner.plan(text)
        except PlanError:
            plan = None
        if plan is not None and plan.goto_steps():
            self.event_bus.request_interrupt("preempt_temp", plan)
            labels = "、".join(step.label for step in plan.goto_steps())
            return EngineResult(intent="navigate", reply="好，我先去%s，回来接着走" % labels,
                                source="rule", plan=plan)

        self.event_bus.publish("ASR", {"text": text})
        return EngineResult(intent="chat", reply="", source="fast")

    def _navigate(self, route: RouteResult, text: str, hint: str) -> EngineResult:
        # Verify place store map_id against runtime map_id before dispatching
        if hasattr(self.nav, "get_runtime_map_id"):
            try:
                ok_map, map_err = self.store.check_map_id(self.nav.get_runtime_map_id())
            except Exception:
                ok_map, map_err = True, ""
            if not ok_map:
                return EngineResult(intent="navigate", reply=map_err, ok=False,
                                    source=route.source, slots=route.slots)

        plan = route.plan
        if plan is None:
            try:
                plan = self.planner.plan(text)
            except PlanError as rule_error:
                plan = self._plan_by_llm(text, hint)
                if plan is None:
                    reply = str(rule_error)
                    if self.llm is not None and not self.llm.available:
                        reply += "（未配置大模型，只能识别地点库里的名字）"
                    return EngineResult(intent="navigate", reply=reply, ok=False,
                                        source=route.source, slots=route.slots)

        where_prefix = ""
        goto_steps = plan.goto_steps()
        if goto_steps and any(word in text for word in _WHERE_HINTS) and hasattr(self.nav, "get_pose"):
            try:
                pose = self.nav.get_pose()
            except Exception:
                pose = None
            if pose:
                try:
                    where_prefix = describe_relative_pose(pose, goto_steps[0].place) + "。"
                except Exception:
                    where_prefix = ""

        exec_result = self.executor.run(plan, loop=self.loop, on_speak=self.on_speak)
        places = "、".join(step.label for step in plan.goto_steps()) or "起点"
        if exec_result.ok:
            if plan.say:
                reply = plan.say
            elif len(plan.steps) > 1:
                reply = "好的，开始执行（%d 步）" % len(plan.steps)
            else:
                reply = "好的"
            if exec_result.state == "DRY_RUN":
                reply = "（预览）%s" % plan.describe()
        else:
            reply = exec_result.message
        if where_prefix:
            reply = where_prefix + reply
        return EngineResult(intent="navigate", reply=reply, ok=exec_result.ok,
                            state=exec_result.state, mission_id=exec_result.mission_id,
                            plan=plan, source=route.source, slots=route.slots)
    def _plan_by_llm(self, text: str, hint: str) -> Optional[Plan]:
        if self.llm is None or not self.llm.available:
            return None
        try:
            return self.llm.plan(text, self.store, state_hint=hint)
        except (LLMError, LLMUnavailable):
            return None

    def _control(self, route: RouteResult) -> EngineResult:
        action = route.action or "pause"
        try:
            ok, message = self.nav.trigger(action)
        except Exception as exc:
            return EngineResult(intent="control", reply="操作失败：%s" % exc, ok=False,
                                source=route.source)
        reply = CONTROL_REPLY.get(action, message) if ok else ("操作失败：%s" % message)
        return EngineResult(intent="control", reply=reply, ok=ok,
                            state=self.nav.snapshot().get("state", ""),
                            source=route.source, slots=route.slots)

    def _query(self, route: RouteResult, text: str = "") -> EngineResult:
        snapshot = self._safe_snapshot()
        state = snapshot.get("state", "UNKNOWN")
        if state == "NAVIGATING":
            index = int(snapshot.get("idx", 0)) + 1
            total = int(snapshot.get("total", 0))
            distance = float(snapshot.get("dist_m", -1.0))
            if distance >= 0:
                reply = "正在去第 %d/%d 个点，还有 %.1f 米" % (index, total, distance)
            else:
                reply = "正在去第 %d/%d 个点" % (index, total)
        elif state in ("ARRIVED", "ALIGNING", "DWELLING"):
            reply = "已经到了，正在停车对齐"
        elif state == "PAUSED":
            reply = "现在是暂停状态，说\"继续\"就走了"
        elif state == "RETURNING_HOME":
            reply = "正在返回起点"
        elif state == "COMPLETED":
            reply = "任务已完成"
        elif state in ("CANCELLED", "FAILED"):
            reply = "任务已结束（%s）" % state.lower()
        elif state == "UNKNOWN":
            reply = "暂时读不到导航状态，检查一下导航栈是否在跑"
        else:
            reply = "当前状态：%s" % state

        wants_health = any(word in (text or "") for word in _HEALTH_HINTS)
        if (wants_health or state in ("FAILED",)) and hasattr(self.nav, "get_health"):
            try:
                health = self.nav.get_health() or {}
            except Exception:
                health = {}
            bad = health.get("bad") or []
            if bad:
                details = "；".join(
                    "%s %s" % (item.get("name", "?"), (item.get("cause", "") or item.get("status", "")))
                    for item in bad[:3]
                )
                reply += "。原因：%s" % details
                advice = (bad[0].get("action") or "").strip()
                if advice:
                    reply += "。建议：%s" % advice

        return EngineResult(intent="query", reply=reply, state=state,
                            source=route.source, slots=route.slots)

    def _safe_snapshot(self) -> Dict:
        try:
            return self.nav.snapshot() or {}
        except Exception:
            return {}

    def _chat(self, route: RouteResult, text: str, hint: str,
              on_token: Optional[Callable[[str], None]]) -> EngineResult:
        if route.source in ("canned", "speak") and route.reply:
            if on_token is not None:
                on_token(route.reply)
            return EngineResult(intent="chat", reply=route.reply, source=route.source)
        if self.llm is None or not self.llm.available:
            reply = route.reply or "我现在只会导航：说\"去某地\"或者\"停一下\"。"
            return EngineResult(intent="chat", reply=reply, source="fallback")
        messages = self.session.messages(prompts.persona(), text, extra_system=hint)
        try:
            if on_token is not None:
                chunks = []
                for delta in self.llm.chat_stream(messages):
                    chunks.append(delta)
                    on_token(delta)
                reply = "".join(chunks).strip()
            else:
                reply = (self.llm.chat(messages) or "").strip()
        except (LLMError, LLMUnavailable) as exc:
            reply = route.reply or "现在连不上语言模型，直接说\"去某地\"我还能走。"
            if self.verbose:
                print("[llm ] 对话失败：%s" % exc)
            return EngineResult(intent="chat", reply=reply, ok=False, source="fallback")
        return EngineResult(intent="chat", reply=reply or route.reply, source=route.source)

    def _other(self, route: RouteResult, text: str) -> EngineResult:
        place = route.place
        if place:
            reply = "这件事我还做不了（%s）。要不要我先带你去%s？" % (text, place)
        else:
            reply = "这件事我还做不了。我目前能带路、暂停、继续、取消和返航。"
        return EngineResult(intent="other", reply=reply, ok=False,
                            source=route.source, slots=route.slots)

    def _state_hint(self) -> str:
        snapshot = self._safe_snapshot()
        if not snapshot:
            return ""
        state = snapshot.get("state", "UNKNOWN")
        hint = "state=%s" % state
        if snapshot.get("total"):
            hint += " idx=%s/%s" % (snapshot.get("idx"), snapshot.get("total"))
        distance = snapshot.get("dist_m")
        if distance is not None and float(distance) >= 0:
            hint += " dist=%.1fm" % float(distance)
        if self.session.last_place:
            hint += " last_place=%s" % self.session.last_place
        return hint
