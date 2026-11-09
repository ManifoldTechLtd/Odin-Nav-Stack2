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

"""Unified event bus for ASR transcripts, user interrupts, mission events, and conditional waits."""

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, Iterable, Optional


@dataclass
class Event:
    type: str                      # ASR | INTERRUPT | MISSION_EVENT | SCAN_CLEARED
    payload: Dict[str, Any] = field(default_factory=dict)
    ts: float = field(default_factory=time.time)


class EventBus(object):
    def __init__(self, maxlen: int = 100):
        self._cond = threading.Condition()
        self._queue: Deque[Event] = deque(maxlen=maxlen)
        self._interrupt_action: Optional[str] = None   # pause | cancel | preempt_temp
        self._preempt_plan: Any = None
        self._listening = False

    def publish(self, event_type: str, payload: Optional[Dict[str, Any]] = None) -> Event:
        evt = Event(type=event_type, payload=dict(payload or {}), ts=time.time())
        with self._cond:
            self._queue.append(evt)
            self._cond.notify_all()
        return evt

    def request_interrupt(self, action: str, preempt_plan: Any = None) -> None:
        """Request an interrupt on the active task (cancel / pause / preempt_temp)."""
        with self._cond:
            self._interrupt_action = action
            self._preempt_plan = preempt_plan
            self._queue.append(Event(type="INTERRUPT", payload={"action": action}))
            self._cond.notify_all()

    def pop_interrupt(self):
        """Pop and clear the pending interrupt (action, preempt_plan)."""
        with self._cond:
            action, plan = self._interrupt_action, self._preempt_plan
            self._interrupt_action = None
            self._preempt_plan = None
            return action, plan

    def has_interrupt(self) -> bool:
        with self._cond:
            return self._interrupt_action is not None

    def set_listening(self, flag: bool) -> None:
        """Mark whether the executor is currently inside an `ask_and_wait` listening window."""
        with self._cond:
            self._listening = bool(flag)

    def is_listening(self) -> bool:
        with self._cond:
            return getattr(self, "_listening", False)

    def clear_asr(self) -> None:
        """Discard stale ASR events before entering an `ask_and_wait` listening window."""
        with self._cond:
            self._queue = deque(e for e in self._queue if e.type != "ASR")

    def clear(self) -> None:
        with self._cond:
            self._queue.clear()
            self._interrupt_action = None
            self._preempt_plan = None

    def wait(self, types: Iterable[str], timeout_sec: float,
             predicate: Optional[Callable[[Event], bool]] = None) -> Optional[Event]:
        """Wait for an event matching `types` (and optional `predicate`), returning None on timeout."""
        target_types = set(types)
        deadline = time.time() + max(0.0, float(timeout_sec))
        with self._cond:
            while True:
                for idx, evt in enumerate(self._queue):
                    if evt.type in target_types and (predicate is None or predicate(evt)):
                        del self._queue[idx]
                        return evt
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cond.wait(timeout=min(remaining, 0.2))
