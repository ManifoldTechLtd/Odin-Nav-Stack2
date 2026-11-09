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

"""Multi-turn conversation session memory (bounded by max_turns and max_chars)."""

from typing import Dict, List, Optional


class Session(object):
    def __init__(self, max_turns: int = 6, max_chars: int = 1800):
        self.max_turns = max(1, int(max_turns))
        self.max_chars = max(200, int(max_chars))
        self._history: List[Dict[str, str]] = []
        self.last_user: str = ""
        self.last_intent: str = ""
        self.last_reply: str = ""
        self.last_place: str = ""

    def add_user(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        self.last_user = text
        self._history.append({"role": "user", "content": text})
        self._trim()

    def add_assistant(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        self.last_reply = text
        self._history.append({"role": "assistant", "content": text})
        self._trim()

    def note(self, intent: str, place: str = "") -> None:
        if intent:
            self.last_intent = intent
        if place:
            self.last_place = place

    def clear(self) -> None:
        self._history.clear()
        self.last_user = ""
        self.last_intent = ""
        self.last_reply = ""
        self.last_place = ""

    def messages(self, system: str, user: str,
                 extra_system: Optional[str] = None) -> List[Dict[str, str]]:
        """Build message list for LLM chat completion."""
        result: List[Dict[str, str]] = [{"role": "system", "content": system}]
        if extra_system:
            result.append({"role": "system", "content": extra_system})
        result.extend(self._history)
        result.append({"role": "user", "content": user})
        return result

    def history_text(self, max_lines: int = 6) -> str:
        """Format recent conversation history for display."""
        lines = []
        for item in self._history[-2 * max_lines:]:
            who = "用户" if item["role"] == "user" else "奥丁"
            lines.append("%s：%s" % (who, item["content"]))
        return "\n".join(lines)

    @property
    def turns(self) -> int:
        return len(self._history) // 2

    def _trim(self) -> None:
        while len(self._history) > self.max_turns * 2:
            self._history.pop(0)
        while len(self._history) > 2 and sum(len(item["content"]) for item in self._history) > self.max_chars:
            self._history.pop(0)
