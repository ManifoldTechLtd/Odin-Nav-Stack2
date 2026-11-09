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

"""Rule-based natural-language planner parsing Chinese utterances into ordered Plan steps.

All waypoint coordinates are resolved strictly from PlaceStore.
"""

import re
import unicodedata
from typing import List, Optional, Tuple

from .models import Plan, Step
from .places import PlaceStore, normalize


def _light_normalize(text: str) -> str:
    """NFKC normalization and whitespace removal without stripping filler words (preserves speech text)."""
    out = unicodedata.normalize("NFKC", text or "").strip()
    return re.sub(r"\s+", "", out)


_PUNCT_ONLY_RE = re.compile(r"^[\s，,。；;：:！!？?\"'“”「」『』（）()、·…—-]*$")

_CN_DIGITS = {
    "零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
    "五": 5, "六": 6, "七": 7, "八": 8, "九": 9,
}

_DWELL_RE = re.compile(
    r"(?:待|呆|停留|等|停|站)\s*(?P<num>[0-9]+(?:\.[0-9]+)?|[零〇一二两三四五六七八九十百]+)\s*"
    r"(?P<unit>秒钟|秒|s|sec|分钟|分)?"
)

_SPLIT_RE = re.compile(r"[，,；;。、]|然后|接着|随后|之后|最后|并且|再")

_SEQ_PREFIX_RE = re.compile(
    r"^(?:首先|先|再|然后|接着|随后|最后|之后|另外|还有|并且|同时)+"
)

_VERB_PREFIX_RE = re.compile(
    r"^(?:请|麻烦|帮我个?忙?|帮忙|带我去|带我|带着我去|带|去|到|前往|开往|走到|走到|走|回|返回|来)+"
)

_HOME_WORDS = ("回来", "回起点", "回家", "返回起点", "返回", "回原点", "回出发点", "回到起点")

# Action step patterns (speak / turn / move_rel / wait)
_SAY_STEP_RE = re.compile(
    r"^(?:(?:说话|喊|说|叫|讲)(?:一声|一句|一下|个)?\s*[：:]?\s*)+"
    r"(.+)$")

_ARRIVAL_ONLY_RE = re.compile(r"^(?:到达|到了|抵达|到)(?:之后|以后|后|了)?$")
_ARRIVE_PREFIX_RE = re.compile(r"^(?:到达|到了|抵达)(?:之后|以后|后)?\s*[，,]?\s*")

_SPATIAL_SUFFIX = ("靠窗那边", "靠窗边", "靠窗", "靠门那边", "靠门", "门口",
                   "左边", "右边", "前面", "后面", "旁边", "那边", "这边", "附近")

_COND_BRANCH_RE = re.compile(
    r"^(?P<prefix>.+?)[，,。；;]?\s*(?:如果|若)(?:你)?(?:听到|有人说|收到)(?P<kw_part>.+?)"
    r"(?:之后|后|时)?(?:你就|就|则)(?P<match_branch>.+?)"
    r"[；;，,。]\s*(?:如果|若|否则)(?:等待|等)?\s*"
    r"(?P<timeout_num>[0-9零〇一二两三四五六七八九十百]+)\s*(?P<timeout_unit>秒钟|秒|分钟|分)"
    r"[^，,]*?(?:就|则|直接)(?P<timeout_branch>.+)$")
_TURN_STEP_RE = re.compile(
    r"^(?:(左|右|顺时针|逆时针))?\s*转\s*(\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百]+)\s*(?:度|°)?$")
_MOVE_STEP_RE = re.compile(
    r"^(\u524d\u8fdb|\u5411\u524d|\u5f80\u524d|\u540e\u9000|\u5411\u540e|\u5f80\u540e|\u5de6\u79fb|\u5411\u5de6|\u5f80\u5de6|\u53f3\u79fb|\u5411\u53f3|\u5f80\u53f3)\s*"
    r"(\d+(?:\.\d+)?|[\u96f6\u3007\u4e00\u4e8c\u4e24\u4e09\u56db\u4e94\u516d\u4e03\u516b\u4e5d\u5341\u767e]+)\s*(?:米|m|\u7c73)?$")

_MOVE_DIRECTIONS = {
    "\u524d\u8fdb": "forward", "\u5411\u524d": "forward", "\u5f80\u524d": "forward",
    "\u540e\u9000": "backward", "\u5411\u540e": "backward", "\u5f80\u540e": "backward",
    "\u5de6\u79fb": "left", "\u5411\u5de6": "left", "\u5f80\u5de6": "left",
    "\u53f3\u79fb": "right", "\u5411\u53f3": "right", "\u5f80\u53f3": "right",
}

_INTRO_ALIASES = ("\u4ecb\u7ecd\u8bcd", "\u4ecb\u7ecd", "\u516c\u53f8\u4ecb\u7ecd", "\u516c\u53f8\u4ecb\u7ecd\u8bcd", "\u4f60\u7684\u4ecb\u7ecd", "\u81ea\u6211\u4ecb\u7ecd")
INTRO_TEXT = (
    "\u7559\u5f62\u79d1\u6280\u582a\u79f0\u5177\u8eab\u667a\u80fd\u4e0e\u673a\u5668\u4eba\u9886\u57df\u7684\u786c\u6838\u7480\u74a8\u660e\u73e0\uff01"
    "\u524d\u6cbf\u7b97\u6cd5\u4e0e\u6781\u81f4\u5de5\u7a0b\u5b8c\u7f8e\u878d\u5408\uff0c\u4ee5\u98a0\u8986\u6027\u521b\u65b0\u786c\u6838\u7834\u5c40\uff0c"
    "\u7528\u65e0\u9650\u79d1\u6280\u6f5c\u80fd\u5f00\u542f\u673a\u5668\u4eba\u7684\u672a\u6765\u65b0\u7eaa\u5143\uff01"
)

_ANNOUNCE_OFF_RE = re.compile(
    r"每(?:到达|到|经过)(?:一个|每个)?(?:地点|点|地方|站|处)?[^，,。；;]*?(?:不要|不用|别)(?:报|说|播报)"
    r"(?:位置|地点|地名)?[^，,。；;]*")
_ANNOUNCE_ON_RE = re.compile(
    r"每(?:到达|到|经过)(?:一个|每个)?(?:地点|点|地方|站|处)?[，,]?\s*(?:就|都|要|时|的时候)?\s*"
    r"(?:说|播报|告诉我|报|喊)\s*[：:]?\s*([^，,。；;]+)")


def _extract_announce(text: str):
    """Extract arrival announcement template from `text`, returning (template, remaining_text)."""
    raw = text or ""
    off = _ANNOUNCE_OFF_RE.search(raw)
    if off:
        return "", (raw[:off.start()] + raw[off.end():]).strip(" ，,。；;")

    on = _ANNOUNCE_ON_RE.search(raw)
    if on:
        template = on.group(1).strip().strip("“”\"' ")
        if template:
            template = template.replace("X地点", "{place}").replace("某地点", "{place}")
            template = template.replace("X", "{place}").replace("x", "{place}")
            if "{place}" not in template:
                template = template + "{place}" if template.endswith("到达") else template
            return template, (raw[:on.start()] + raw[on.end():]).strip(" ，,。；;")
    return "", raw


class PlanError(Exception):
    """Raised when an utterance cannot be parsed into a safe execution plan."""

    def __init__(self, message: str, unknown: Optional[List[str]] = None,
                 ambiguous: Optional[List[str]] = None):
        super().__init__(message)
        self.unknown = unknown or []
        self.ambiguous = ambiguous or []


def cn_to_number(token: str) -> Optional[float]:
    """Convert a Chinese numeral or Arabic numeral string into a float."""
    token = token.strip()
    if not token:
        return None
    try:
        return float(token)
    except ValueError:
        pass
    if token == "十":
        return 10.0
    if "十" in token:
        left, _, right = token.partition("十")
        tens = _CN_DIGITS.get(left, 1) if left else 1
        ones = _CN_DIGITS.get(right, 0) if right else 0
        return float(tens * 10 + ones)
    if token in _CN_DIGITS:
        return float(_CN_DIGITS[token])
    if token == "百":
        return 100.0
    return None


def _clean_segment(segment: str) -> str:
    """Strip sequence connectors and navigation verb prefixes from a segment."""
    text = segment.strip()
    for _ in range(2):
        text = _SEQ_PREFIX_RE.sub("", text).strip()
        text = _VERB_PREFIX_RE.sub("", text).strip()
    return text


class RulePlanner:
    """Rule-based planner backed by PlaceStore."""

    def __init__(self, store: PlaceStore, max_steps: int = 10):
        self.store = store
        self.max_steps = max_steps

    def plan(self, text: str) -> Plan:
        raw = (text or "").strip()
        if not raw:
            raise PlanError("输入为空")

        normalized = normalize(raw)

        announce, raw_wo_announce = _extract_announce(raw)
        if raw_wo_announce and raw_wo_announce != raw:
            raw = raw_wo_announce
            normalized = normalize(raw)

        conditional = self._parse_conditional(raw)
        if conditional is not None:
            return Plan(steps=conditional, source="rule", text=raw, announce=announce)

        steps: List[Step] = []
        unknown: List[str] = []
        ambiguous: List[str] = []

        for segment in self._split(normalized):
            if not segment:
                continue
            if _PUNCT_ONLY_RE.match(segment):
                continue
            dwell, remainder = self._extract_dwell(segment)

            if _ARRIVAL_ONLY_RE.match(remainder.strip()):
                continue

            action = self._parse_action(remainder)
            if action is not None:
                steps.append(action)
                continue

            if self._is_home(remainder):
                steps.append(Step(kind="home", dwell_sec=0.0))
                continue

            place, options = self.store.resolve(remainder)
            if place is not None:
                if dwell is None:
                    dwell = place.dwell_default_sec
                steps.append(Step(place=place, dwell_sec=float(dwell or 0.0),
                                  modifier=self._extract_modifier(remainder, place)))
                continue

            if options:
                ambiguous.append("%s（可能是：%s）" % (remainder, "、".join(p.name for p in options)))
                continue

            if dwell is not None and steps and steps[-1].kind == "goto":
                steps[-1].dwell_sec = float(dwell)
                continue

            if dwell is not None and not remainder.strip():
                steps.append(Step(kind="wait", dwell_sec=float(dwell)))
                continue

            place, options = self.store.resolve(segment)
            if place is not None:
                steps.append(Step(place=place, dwell_sec=float(dwell or 0.0)))
                continue
            if options:
                ambiguous.append("%s（可能是：%s）" % (segment, "、".join(p.name for p in options)))
                continue
            cleaned = _clean_segment(segment)
            if steps and not cleaned:
                continue
            unknown.append(cleaned or segment)

        if ambiguous:
            raise PlanError("地点有歧义：" + "；".join(ambiguous), ambiguous=ambiguous)
        if unknown:
            raise PlanError("地点库里没有：" + "、".join(unknown), unknown=unknown)
        if not steps:
            raise PlanError("没能从这句话里识别出目的地")
        if len(steps) > self.max_steps:
            raise PlanError("一次最多 %d 步，当前 %d 步" % (self.max_steps, len(steps)))

        return Plan(steps=steps, source="rule", text=raw, announce=announce)

    def _parse_conditional(self, text: str) -> Optional[List[Step]]:
        """Parse conditional `ask_and_wait` utterances into steps."""
        light = _light_normalize(text)
        match = _COND_BRANCH_RE.match(light)
        if not match:
            return None

        prefix = (match.group("prefix") or "").strip().strip("，,。；; ")
        match_branch = (match.group("match_branch") or "").strip().strip("，,。；; ")
        timeout_branch = (match.group("timeout_branch") or "").strip().strip("，,。；; ")

        number = cn_to_number(match.group("timeout_num"))
        unit = match.group("timeout_unit") or "秒"
        factor = 60.0 if unit in ("分钟", "分") else 1.0
        timeout_sec = float(number or 20) * factor

        steps: List[Step] = []
        if prefix:
            try:
                steps.extend(self.plan(prefix).steps)
            except PlanError:
                pass

        ask_text = ""
        ask_match = re.search(r"(?:说话|说)[：:]\s*(.+)$", prefix)
        if ask_match:
            ask_text = ask_match.group(1).strip().strip("。；; ")
        if steps and steps[-1].kind == "speak":
            steps.pop()

        keywords = self._extract_keywords(match.group("kw_part") or "")

        on_match: List[Step] = []
        if match_branch:
            try:
                on_match = self.plan(match_branch).steps
            except PlanError:
                on_match = []

        on_timeout: List[Step] = []
        if timeout_branch:
            try:
                on_timeout = self.plan(timeout_branch).steps
            except PlanError:
                on_timeout = []

        if not on_match and not on_timeout:
            return None

        steps.append(Step(kind="ask_and_wait", text=ask_text,
                          expect_keywords=keywords, timeout_sec=timeout_sec,
                          on_match=on_match, on_timeout=on_timeout))
        return steps

    @staticmethod
    def _extract_keywords(text: str) -> List[str]:
        """Extract keyword list from a conditional clause."""
        cleaned = text or ""
        for ch in "“”\"'「」『』（）()":
            cleaned = cleaned.replace(ch, "")
        keywords: List[str] = []
        for part in re.split(r"或者|或是|或|、|,", cleaned):
            part = part.strip().strip("，。；;  ")
            part = re.sub(r"^(?:有人)?(?:说|喊|回答|答)\s*", "", part)
            if 1 <= len(part) <= 12 and part not in keywords:
                keywords.append(part)
        return keywords

    @staticmethod
    def _extract_modifier(text: str, place) -> str:
        """Extract spatial modifier suffix following the matched place name."""
        norm_text = normalize(_VERB_PREFIX_RE.sub("", normalize(text)))
        for key in [place.name] + list(place.aliases):
            norm_key = normalize(key)
            if not norm_key or not norm_text.startswith(norm_key):
                continue
            rest = norm_text[len(norm_key):]
            for suffix in _SPATIAL_SUFFIX:
                if suffix in rest:
                    return suffix
        return ""

    @staticmethod
    def _parse_action(text: str):
        """Parse a clause as an atomic action step (speak / turn / move_rel), or return None."""
        cleaned = _SEQ_PREFIX_RE.sub("", (text or "").strip()).strip().strip("，,。. ")
        cleaned = _ARRIVE_PREFIX_RE.sub("", cleaned).strip()
        if not cleaned:
            return None

        match = _SAY_STEP_RE.match(cleaned)
        if match:
            content = match.group(1).strip().strip("“”\"'「」『』 ")
            if content in _INTRO_ALIASES:
                content = INTRO_TEXT
            if content:
                return Step(kind="speak", text=content)

        match = _TURN_STEP_RE.match(cleaned)
        if match:
            side = match.group(1) or ""
            value = cn_to_number(match.group(2))
            if value:
                if side in ("右", "顺时针"):
                    value = -value
                return Step(kind="turn", angle_deg=float(value))

        match = _MOVE_STEP_RE.match(cleaned)
        if match:
            direction = _MOVE_DIRECTIONS.get(match.group(1))
            value = cn_to_number(match.group(2))
            if direction and value:
                return Step(kind="move_rel", direction=direction, distance_m=float(value))

        return None

    @staticmethod
    def _split(text: str) -> List[str]:
        parts = _SPLIT_RE.split(text)
        return [part.strip() for part in parts if part and part.strip()]

    @staticmethod
    def _extract_dwell(segment: str) -> Tuple[Optional[float], str]:
        """Extract dwell duration in seconds from `segment`, returning (seconds, remainder)."""
        match = _DWELL_RE.search(segment)
        if not match:
            return None, segment
        value = cn_to_number(match.group("num"))
        if value is None:
            return None, segment
        unit = match.group("unit") or "秒"
        if unit in ("分钟", "分"):
            value *= 60.0
        remainder = segment[: match.start()] + segment[match.end():]
        return value, remainder.strip()

    @staticmethod
    def _is_home(text: str) -> bool:
        cleaned = _clean_segment(text)
        for word in _HOME_WORDS:
            if cleaned == word or text.strip() == word:
                return True
        return False
