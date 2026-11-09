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

"""OpenAI-compatible LLM client (standard library urllib only, zero external SDK dependencies)."""

import json
import os
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Iterator, List, Optional

from .models import Plan, Step
from .places import PlaceStore, PlaceError
from . import prompts

SYSTEM_PROMPT = """你是巡检导航机器人「Odin」的任务解析器。

# 任务
把用户的一句话解析成有序执行步骤。你只做解析，不生成坐标。

# 铁律
1. 步骤里的地点必须来自下面给出的地点清单，原样使用清单里的 name。
2. 不要编造地点；清单里没有就把它放进 unknown。
3. 停留时间只在用户明确说了"等/待/停留 N 秒"时才填，否则填 0。
4. 用户说"回来/回起点/回家"时，输出 kind="home" 的步骤（不需要地点）。
5. 只输出 JSON，不要 markdown 代码块，不要解释。

# 输出格式
{"steps":[{"kind":"goto","place":"厕所","dwell_sec":5},{"kind":"goto","place":"小A工位","dwell_sec":0}],"say":"好的，先去厕所待五秒，再去小A工位","unknown":[]}

# 地点清单
{places}
"""


class LLMUnavailable(Exception):
    """Raised when the LLM client is unconfigured or unavailable."""


# Environment file loading ($ODIN_BRAIN_LLM_ENV -> ~/.config/odin_brain/llm.env -> ~/.odin_brain.env -> ./llm.env)
_ENV_LOADED = False
_ENV_FILE = None


def env_file_candidates():
    explicit = os.environ.get("ODIN_BRAIN_LLM_ENV", "")
    if explicit:
        yield os.path.expanduser(explicit)
    yield os.path.expanduser("~/.config/odin_brain/llm.env")
    yield os.path.expanduser("~/.odin_brain.env")
    yield os.path.join(os.getcwd(), "llm.env")


def load_llm_env(force: bool = False):
    """Load KEY=VALUE pairs from the first existing config file without overriding existing environment variables."""
    global _ENV_LOADED, _ENV_FILE
    if _ENV_LOADED and not force:
        return _ENV_FILE
    _ENV_LOADED = True
    for path in env_file_candidates():
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path, "r", encoding="utf-8") as handle:
                for raw in handle:
                    line = raw.strip()
                    if not line or line.startswith("#"):
                        continue
                    if line.startswith("export "):
                        line = line[len("export "):].strip()
                    if "=" not in line:
                        continue
                    key, _, value = line.partition("=")
                    key = key.strip()
                    value = value.strip().strip('"').strip("'")
                    if key and key not in os.environ:
                        os.environ[key] = value
        except Exception:
            continue
        _ENV_FILE = path
        return path
    return None


class LLMError(Exception):
    """Raised when an LLM request or response parsing fails."""


class LLMConfig:
    """LLM configuration loaded from environment variables."""

    def __init__(self, base_url: str = "", api_key: str = "", model: str = "",
                 timeout_sec: float = 20.0, temperature: float = 0.1,
                 thinking: str = "disabled", retry_max: int = 2,
                 retry_backoff_sec: float = 1.0):
        self.base_url = (base_url or "").rstrip("/")
        self.api_key = api_key or ""
        self.model = model or ""
        self.timeout_sec = float(timeout_sec)
        self.temperature = float(temperature)
        self.thinking = (thinking or "").strip().lower()
        self.retry_max = max(0, int(retry_max))
        self.retry_backoff_sec = float(retry_backoff_sec)

    @classmethod
    def from_env(cls) -> "LLMConfig":
        load_llm_env()
        return cls(
            base_url=os.environ.get("ODIN_LLM_BASE_URL", ""),
            api_key=os.environ.get("ODIN_LLM_API_KEY", ""),
            model=os.environ.get("ODIN_LLM_MODEL", ""),
            timeout_sec=float(os.environ.get("ODIN_LLM_TIMEOUT", "20")),
            temperature=float(os.environ.get("ODIN_LLM_TEMPERATURE", "0.1")),
            thinking=os.environ.get("ODIN_LLM_THINKING", "disabled"),
            retry_max=int(os.environ.get("ODIN_LLM_RETRY", "2")),
            retry_backoff_sec=float(os.environ.get("ODIN_LLM_RETRY_BACKOFF", "1.0")),
        )

    def describe(self) -> str:
        if not self.enabled:
            return "unconfigured (requires ODIN_LLM_BASE_URL / ODIN_LLM_MODEL)"
        return "%s / model=%s / thinking=%s" % (
            self.base_url, self.model, self.thinking or "default")

    @property
    def enabled(self) -> bool:
        return bool(self.base_url and self.model)


class LLMClient:
    """OpenAI-compatible /chat/completions client."""

    def __init__(self, config: LLMConfig):
        self.config = config

    @property
    def available(self) -> bool:
        return self.config.enabled

    def plan(self, text: str, store: PlaceStore, state_hint: str = "") -> Plan:
        if not self.available:
            raise LLMUnavailable("LLM unconfigured: set ODIN_LLM_BASE_URL / ODIN_LLM_MODEL")
        payload = self._request(text, store, state_hint)
        return self._to_plan(payload, text, store)

    def chat(self, messages: List[Dict[str, str]], **overrides) -> str:
        """Non-streaming chat completion."""
        envelope = self._post_chat(self._build_body(messages, stream=False, **overrides))
        try:
            return envelope["choices"][0]["message"].get("content") or ""
        except Exception:
            raise LLMError("Response format is not OpenAI-compatible")

    def chat_stream(self, messages: List[Dict[str, str]], **overrides) -> Iterator[str]:
        """Streaming SSE chat completion yielding content deltas."""
        response = self._post_chat(
            self._build_body(messages, stream=True, **overrides), stream=True)
        try:
            for raw in response:
                line = raw.decode("utf-8", errors="replace").strip()
                if not line or not line.startswith("data:"):
                    continue
                payload = line[5:].strip()
                if payload == "[DONE]":
                    break
                try:
                    chunk = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                choices = chunk.get("choices") or []
                if not choices:
                    continue
                delta = (choices[0].get("delta") or {}).get("content")
                if delta:
                    yield delta
        finally:
            try:
                response.close()
            except Exception:
                pass

    def _build_body(self, messages: List[Dict[str, str]], stream: bool = False,
                    **overrides) -> Dict[str, Any]:
        body = {
            "model": self.config.model,
            "temperature": self.config.temperature,
            "messages": messages,
        }
        if stream:
            body["stream"] = True
        for key, value in overrides.items():
            if value is not None:
                body[key] = value
        return body

    def _request(self, text: str, store: PlaceStore, state_hint: str) -> Dict[str, Any]:
        system = self._plan_system_prompt(store, state_hint)
        envelope = self._post_chat(self._build_body([
            {"role": "system", "content": system},
            {"role": "user", "content": text},
        ]))
        try:
            content = envelope["choices"][0]["message"].get("content") or ""
        except Exception:
            raise LLMError("Response format is not OpenAI-compatible")
        if not content.strip():
            raise LLMError("Model returned empty content")
        return _extract_json(content)

    @staticmethod
    def _plan_system_prompt(store: PlaceStore, state_hint: str = "") -> str:
        template = prompts.planner()
        block = store.prompt_block()
        if "{places}" in template:
            system = template.replace("{places}", block)
        else:
            system = template + "\n\n# 地点清单\n" + block + "\n"
        if state_hint:
            system += "\n# 当前状态\n" + state_hint + "\n"
        return system

    def _post_chat(self, body: Dict[str, Any], stream: bool = False):
        if self.config.thinking and "thinking" not in body:
            body["thinking"] = {"type": self.config.thinking}
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        url = self.config.base_url + "/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": "Bearer " + (self.config.api_key or "none"),
        }
        last_error = "请求失败"
        for attempt in range(self.config.retry_max + 1):
            request = urllib.request.Request(url, data=data, headers=headers, method="POST")
            try:
                response = urllib.request.urlopen(request, timeout=self.config.timeout_sec)
                if stream:
                    return response
                raw = response.read().decode("utf-8", errors="replace")
                return json.loads(raw)
            except urllib.error.HTTPError as exc:
                detail = exc.read().decode("utf-8", errors="replace")[:300]
                last_error = "HTTP %s: %s" % (exc.code, detail)
                if _is_retryable(exc.code, detail) and attempt < self.config.retry_max:
                    time.sleep(self.config.retry_backoff_sec * (2 ** attempt))
                    continue
                raise LLMError(last_error)
            except urllib.error.URLError as exc:
                last_error = "连接失败: %s" % exc.reason
                if attempt < self.config.retry_max:
                    time.sleep(self.config.retry_backoff_sec * (2 ** attempt))
                    continue
                raise LLMError(last_error)
            except Exception as exc:
                raise LLMError("请求失败: %s" % exc)
        raise LLMError(last_error)

    @staticmethod
    def _parse_steps(raw_steps, store: PlaceStore) -> list:
        """Recursively parse raw step dicts into Step objects (supporting nested ask_and_wait branches)."""
        steps = []
        for item in raw_steps:
            if not isinstance(item, dict):
                raise LLMError("steps 元素不是对象")
            kind = str(item.get("kind", "goto")).lower()
            dwell = float(item.get("dwell_sec") or 0.0)
            if kind == "home":
                steps.append(Step(kind="home", dwell_sec=0.0))
                continue
            if kind == "speak":
                content = str(item.get("text", "")).strip()
                if content:
                    steps.append(Step(kind="speak", text=content))
                continue
            if kind == "turn":
                steps.append(Step(kind="turn", angle_deg=float(item.get("angle_deg") or 0.0)))
                continue
            if kind == "move_rel":
                steps.append(Step(kind="move_rel",
                                  direction=str(item.get("direction", "forward")).lower(),
                                  distance_m=float(item.get("distance_m") or 0.0)))
                continue
            if kind == "wait":
                steps.append(Step(kind="wait", dwell_sec=float(item.get("dwell_sec") or 0.0)))
                continue
            if kind == "ask_and_wait":
                steps.append(Step(
                    kind="ask_and_wait",
                    text=str(item.get("text", "") or ""),
                    expect_keywords=[str(k) for k in (item.get("expect_keywords") or [])],
                    timeout_sec=float(item.get("timeout_sec") or 20.0),
                    on_match=LLMClient._parse_steps(item.get("on_match") or [], store),
                    on_timeout=LLMClient._parse_steps(item.get("on_timeout") or [], store),
                ))
                continue
            name = str(item.get("place", "")).strip()
            if not name:
                raise LLMError("goto 步骤缺少 place")
            place = store.get(name)
            if place is None:
                place, options = store.resolve(name)
            else:
                options = []
            if place is None:
                if options:
                    raise LLMError("地点有歧义：%s（%s）" % (name, "、".join(p.name for p in options)))
                raise LLMError("地点库里没有：%s" % name)
            steps.append(Step(place=place, dwell_sec=dwell,
                              modifier=str(item.get("modifier", "") or "")))
        return steps

    @staticmethod
    def _to_plan(payload: Dict[str, Any], text: str, store: PlaceStore) -> Plan:
        raw_steps = payload.get("steps")
        if not isinstance(raw_steps, list) or not raw_steps:
            raise LLMError("模型没有给出 steps")
        unknown = payload.get("unknown") or []
        if unknown:
            raise LLMError("地点库里没有：" + "、".join(str(item) for item in unknown))

        steps = LLMClient._parse_steps(raw_steps, store)

        if not steps:
            raise LLMError("steps 为空")
        return Plan(steps=steps, source="llm", text=text,
                    say=str(payload.get("say", "") or ""),
                    announce=str(payload.get("announce", "") or ""))


_RETRYABLE_HTTP = (429, 500, 502, 503, 504)
_RETRYABLE_BIZ_CODES = (1302, 1305)


def _is_retryable(http_code: int, detail: str = "") -> bool:
    if http_code in _RETRYABLE_HTTP:
        return True
    for code in _RETRYABLE_BIZ_CODES:
        if str(code) in (detail or ""):
            return True
    return False


def _extract_json(content: str) -> Dict[str, Any]:
    """Parse JSON object from model output, stripping optional markdown fences."""
    if not isinstance(content, str):
        raise LLMError("模型返回内容不是字符串")
    text = content.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
        text = text.strip()
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            raise LLMError("模型输出不是 JSON：%s" % content[:200])
        try:
            payload = json.loads(text[start:end + 1])
        except json.JSONDecodeError as exc:
            raise LLMError("模型输出 JSON 解析失败：%s (%s)" % (content[:200], exc))
    if not isinstance(payload, dict):
        raise LLMError("模型输出不是 JSON 对象")
    return payload


def build_client(config: Optional[LLMConfig] = None) -> LLMClient:
    return LLMClient(config or LLMConfig.from_env())
