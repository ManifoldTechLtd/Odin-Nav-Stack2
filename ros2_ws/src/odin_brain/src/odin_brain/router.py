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

"""Intent router classifying user utterances into navigate / control / query / chat / other."""

import json
import re
import unicodedata
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from .llm_client import LLMClient, LLMError, LLMUnavailable
from .models import Plan
from .places import PlaceStore, normalize
from .planner import PlanError, RulePlanner
from . import prompts

# Full-sentence control phrase table
CONTROL_PHRASES = (
    ("pause", ("停", "停车", "停下", "停一下", "停下来", "暂停", "别动", "别走",
               "等一下", "等等", "稍等", "等会", "呆着别动")),
    ("resume", ("继续", "继续走", "继续吧", "走吧", "可以走了", "接着走", "恢复", "恢复任务")),
    ("cancel", ("取消", "取消任务", "算了", "算了取消", "别去了", "不去了", "放弃", "停止任务")),
    ("return_home", ("回来", "回起点", "回家", "返航", "返回起点", "回出发点")),
)

_CONTROL_POLITE = ("请", "帮我", "麻烦", "快", "赶紧", "马上", "立刻", "现在",
                   "先", "给我", "你", "能不能", "可以", "那就", "就")
_CONTROL_TAIL = "吧啊呀哦呢了哈嘛嘛!！。. "

QUERY_WORDS = ("到哪", "还有多远", "多远", "多久", "几点", "进度", "状态",
               "怎么停", "为什么停", "在哪儿", "在哪", "正常吗", "什么情况")

NAV_PREFIXES = ("带我去", "带我", "导航到", "导航去", "去", "到", "前往", "走到",
                "回", "返回", "来", "送我到")
_NAV_POLITE = ("请", "帮我", "麻烦", "快", "现在", "先", "给我", "你", "能不能")

OTHER_WORDS = ("送瓶水", "帮我拿", "搬", "机械臂", "拍照上传", "抓取", "递")

CANNED_REPLIES = (
    ("曼巴", "out!"),
    ("潘虹", "哥们！"),
)

HAJIDING = "哈吉丁"
HAJIDING_IDENTITY = "我是留形科技最忠诚的卫士！忠！诚！"
HAJIDING_SLOGAN = "留形科技！忠！诚！"

SOLDIER_WORDS = ("二等兵", "士兵丁", "二等兵丁")
SOLDIER_REPLY = "忠诚！长官！"

_VALID_CONTROL_ACTIONS = ("pause", "resume", "cancel", "return_home", "reset")

_EXPAND_HINTS = ("短", "长", "多说", "多讲", "展开", "继续", "接着", "详细",
                 "具体", "不够", "再", "深入", "举例")

_SPEAK_PATTERNS = (
    re.compile(r"^(?:说话|说|讲|念|复述|重复|跟我念|说一遍|念一遍|重复一遍)\s*[：:]\s*(.+)$"),
    re.compile(r"^说话\s+(.+)$"),
    re.compile(r"^(?:说一遍|念一遍|重复一遍|跟我念|复述)\s*[：:]?\s*(.+)$"),
    re.compile(r"^(?:你就说|跟(?:他|她|它|对方|大家)说|告诉(?:他|她|它|对方))\s*[：:]?\s*(.+)$"),
    re.compile(r"^(?:喊|叫)\s*[：:]\s*(.+)$"),
)

_QUOTE_CHARS = "“”\"'「」『』＂＇ "

LIUFANG_INTRO = (
    "留形科技堪称具身智能与机器人领域的硬核璀璨明珠！"
    "前沿算法与极致工程完美融合，以颠覆性创新硬核破局，"
    "用无限科技潜能开启机器人的未来新纪元！"
)


def _normalize_control(text: str) -> str:
    """Normalize text for control matching without stripping filler tokens like '一下'."""
    out = unicodedata.normalize("NFKC", text or "").strip().lower()
    return re.sub(r"\s+", "", out)


@dataclass
class RouteResult:
    intent: str = "chat"
    slots: Dict[str, str] = field(default_factory=dict)
    reply: str = ""
    source: str = "rule"
    reason: str = ""
    plan: Optional[Plan] = None

    @property
    def place(self) -> str:
        return self.slots.get("place", "")

    @property
    def action(self) -> str:
        return self.slots.get("action", "")


class Router(object):
    def __init__(self, store: PlaceStore, planner: RulePlanner,
                 llm: Optional[LLMClient] = None):
        self.store = store
        self.planner = planner
        self.llm = llm
        self._handlers: Dict[str, Callable] = {}

    def register(self, intent: str, handler: Callable) -> None:
        """Register or override a custom handler for `intent`."""
        self._handlers[intent] = handler

    def handler(self, intent: str) -> Optional[Callable]:
        return self._handlers.get(intent)

    @property
    def intents(self) -> List[str]:
        return ["navigate", "control", "query", "chat", "other"] + \
               [key for key in self._handlers if key not in
                ("navigate", "control", "query", "chat", "other")]

    def route(self, text: str, state_hint: str = "") -> RouteResult:
        raw = (text or "").strip()
        if not raw:
            return RouteResult(intent="chat", reply="", source="rule",
                               reason="空输入")
        normalized = normalize(raw)

        spoken = self._match_speak(raw)
        if spoken:
            return RouteResult(intent="chat", reply=spoken, source="speak",
                               reason="显式传话指令")

        control = self._match_control(raw)
        if control:
            return RouteResult(intent="control", slots={"action": control},
                               source="rule", reason="控制关键词命中")

        canned = self._match_canned(normalized)
        if canned:
            return RouteResult(intent="chat", reply=canned, source="canned",
                               reason="固定话术命中")

        other = self._match_other(raw)
        if other:
            return RouteResult(intent="other", slots={"place": other},
                               source="rule", reason="超出当前能力")

        try:
            plan = self.planner.plan(raw)
            return RouteResult(intent="navigate", slots={"place": plan.steps[0].label},
                               plan=plan, source="rule", reason="规则解析成功")
        except PlanError:
            pass

        if self._match_query(normalized):
            return RouteResult(intent="query", source="rule", reason="状态询问关键词")

        if self._looks_like_navigate(raw):
            return RouteResult(intent="navigate", source="rule",
                               reason="句首是导航动词但未解析出地点")

        if self.llm is not None and self.llm.available:
            result = self._route_by_llm(raw, state_hint)
            if result is not None:
                return result

        return RouteResult(intent="chat", source="fallback", reason="规则与模型均未判定")

    @staticmethod
    def _match_speak(text: str) -> str:
        """Extract verbatim speech content from explicit repeat/say commands."""
        raw = (text or "").strip()
        if not raw:
            return ""
        for pattern in _SPEAK_PATTERNS:
            match = pattern.match(raw)
            if not match:
                continue
            content = match.group(1).strip().strip(_QUOTE_CHARS).strip()
            return content
        return ""

    @staticmethod
    def _match_control(text: str) -> str:
        """Match full-sentence control commands after stripping polite prefixes and trailing particles."""
        text = _normalize_control(text)
        for _ in range(3):
            for prefix in _CONTROL_POLITE:
                if text.startswith(prefix):
                    text = text[len(prefix):]
                    break
            else:
                break
        text = text.strip(_CONTROL_TAIL)
        for action, phrases in CONTROL_PHRASES:
            for phrase in phrases:
                if text == phrase:
                    return action
                if len(phrase) >= 3 and text.startswith(phrase) and \
                        len(text) - len(phrase) <= 2:
                    return action
        return ""

    @staticmethod
    def _looks_like_navigate(text: str) -> bool:
        """Return True if `text` starts with a navigation verb after stripping polite prefixes."""
        cleaned = _normalize_control(text)
        for _ in range(2):
            for prefix in _NAV_POLITE:
                if cleaned.startswith(prefix):
                    cleaned = cleaned[len(prefix):]
                    break
            else:
                break
        return any(cleaned.startswith(prefix) for prefix in NAV_PREFIXES)

    @staticmethod
    def _match_query(normalized: str) -> bool:
        return any(word in normalized for word in QUERY_WORDS)

    @staticmethod
    def _match_canned(normalized: str) -> str:
        """Match short canned replies unless expansion hints are present."""
        if any(hint in normalized for hint in _EXPAND_HINTS):
            return ""

        for key, reply in CANNED_REPLIES:
            if key in normalized:
                return reply

        if HAJIDING in normalized:
            if any(word in normalized for word in ("什么身份", "是哪位", "角色")) or \
                    ("什么" in normalized and "忠诚" not in normalized and len(normalized) <= 14):
                return HAJIDING_IDENTITY
            rest = normalized.replace(HAJIDING, "").strip("，,。!！?？. 吧啊呀哦呢 ")
            if not rest:
                return HAJIDING_SLOGAN
            return ""

        if any(word in normalized for word in SOLDIER_WORDS):
            return SOLDIER_REPLY

        return ""

    @staticmethod
    def _match_other(raw: str) -> str:
        for word in OTHER_WORDS:
            index = raw.find(word)
            if index >= 0:
                tail = raw[index + len(word):].strip("，,。 ")
                return tail[:12]
        return ""

    def _route_by_llm(self, text: str, state_hint: str = "") -> Optional[RouteResult]:
        system = prompts.router()
        if state_hint:
            system += "\n# 当前状态\n" + state_hint + "\n"
        try:
            content = self.llm.chat([
                {"role": "system", "content": system},
                {"role": "user", "content": text},
            ])
        except (LLMError, LLMUnavailable):
            return None
        payload = _parse_json_object(content)
        if not payload:
            return None
        intent = str(payload.get("intent", "chat")).strip().lower()
        if intent not in ("navigate", "control", "query", "chat", "other"):
            intent = "chat"
        if intent == "control" and any(hint in normalize(text) for hint in _EXPAND_HINTS):
            intent = "chat"
        slots = payload.get("slots") or {}
        if not isinstance(slots, dict):
            slots = {}
        if intent == "control":
            action = str(slots.get("action", "")).strip().lower()
            if action not in _VALID_CONTROL_ACTIONS:
                intent = "chat"
                slots = {}
        result = RouteResult(
            intent=intent,
            slots={key: str(value) for key, value in slots.items() if value},
            reply=str(payload.get("reply", "") or ""),
            source="llm",
            reason=str(payload.get("reason", "") or ""),
        )
        if intent == "navigate":
            try:
                result.plan = self.planner.plan(text)
            except PlanError:
                result.plan = None
        return result


def _parse_json_object(content: str) -> Optional[dict]:
    if not isinstance(content, str):
        return None
    text = content.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
        text = text.strip()
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            return None
        try:
            payload = json.loads(text[start:end + 1])
        except json.JSONDecodeError:
            return None
    return payload if isinstance(payload, dict) else None
