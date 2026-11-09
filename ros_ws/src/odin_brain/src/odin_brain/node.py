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

"""Resident ROS 1 node for odin_brain.

Topics:
    Sub  /brain/command  std_msgs/String   Natural-language command
    Sub  /brain/enable   std_msgs/Bool     Latched motion enable switch (false = dry-run)
    Pub  /brain/result   std_msgs/String   JSON result: intent/reply/ok/state/plan/mission_id/ts
    Pub  /brain/speech   std_msgs/String   Reply text for TTS / UI
    Pub  /brain/ready    std_msgs/Bool     Latched readiness flag
"""

import json
import queue
import threading
import time
from typing import Optional

from .engine import BrainEngine
from .llm_client import build_client
from .places import PlaceStore
from .planner import RulePlanner


def _as_bool(value, default=False) -> bool:
    """Normalize launch/parameter server boolean values."""
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class BrainNode(object):
    def __init__(self, places_path: str, enable_motion: bool = True,
                 use_llm: bool = True, dry_run: bool = False):
        import rospy
        from std_msgs.msg import Bool, String
        from .ros1_nav import Ros1Nav

        self._rospy = rospy
        self.store = PlaceStore(places_path)
        self.planner = RulePlanner(self.store)
        self.llm = build_client() if use_llm else None
        self.nav = Ros1Nav()
        self.motion_enabled = bool(enable_motion)
        self.base_dry_run = bool(dry_run)
        self.engine = BrainEngine(self.store, self.planner, self.llm, self.nav,
                                  dry_run=(self.base_dry_run or not self.motion_enabled),
                                  verbose=True)
        self._queue = queue.Queue(maxsize=4)

        self._result_pub = rospy.Publisher("/brain/result", String, queue_size=10)
        self._speech_pub = rospy.Publisher("/brain/speech", String, queue_size=10)
        self._ready_pub = rospy.Publisher("/brain/ready", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/brain/command", String, self._on_command, queue_size=10)
        rospy.Subscriber("/brain/enable", Bool, self._on_enable, queue_size=1)

        self._worker = threading.Thread(target=self._work, daemon=True)
        self._worker.start()

        self._ready_pub.publish(Bool(data=True))
        rospy.loginfo("odin_brain ready: places=%s(%d) llm=%s motion=%s dry_run=%s" % (
            places_path, len(self.store.places),
            self.llm.config.describe() if self.llm and self.llm.available else "rule",
            self.motion_enabled, self.base_dry_run))

    def _on_enable(self, msg) -> None:
        self.motion_enabled = bool(msg.data)
        self.engine.dry_run = self.base_dry_run or not self.motion_enabled
        self.engine.executor.dry_run = self.engine.dry_run
        self._rospy.loginfo("motion enable -> %s (dry_run=%s)" % (
            self.motion_enabled, self.engine.dry_run))

    def _on_command(self, msg) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self._publish("chat", "上一条还在处理，请稍候再说", ok=False,
                          state="BUSY", plan=[])

    def _work(self) -> None:
        while not self._rospy.is_shutdown():
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self.engine.busy:
                    fast = self.engine.handle_fast(text)
                    if fast is not None:
                        self._publish(fast.intent, fast.reply, fast.ok, fast.state,
                                      fast.plan_payload(), fast.mission_id, fast.source)
                    continue
                self.engine.handle_async(text, on_done=self._publish_result)
            except Exception as exc:
                self._publish("error", "内部错误：%s" % exc, ok=False, state="ERROR", plan=[])

    def _publish_result(self, result) -> None:
        try:
            self._publish(result.intent, result.reply, result.ok, result.state,
                          result.plan_payload(), result.mission_id, result.source)
        except Exception:
            pass

    def _publish(self, intent: str, reply: str, ok: bool = True, state: str = "",
                 plan: Optional[list] = None, mission_id: str = "", source: str = "") -> None:
        from std_msgs.msg import String
        payload = {
            "ok": bool(ok),
            "intent": intent,
            "reply": reply,
            "state": state,
            "plan": plan or [],
            "mission_id": mission_id,
            "source": source,
            "ts": time.time(),
        }
        self._result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if reply:
            self._speech_pub.publish(String(data=reply))
        self._rospy.loginfo("[result] %s %s %s" % (intent, "ok" if ok else "fail", reply[:80]))


def main(argv: Optional[list] = None) -> int:
    import argparse
    import rospy
    from .cli import default_places_path

    raw_argv = rospy.myargv(argv=argv)[1:]
    parser = argparse.ArgumentParser(prog="brain_node", description="Resident command node (ROS 1)")
    parser.add_argument("--places", default="")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--disable-motion", action="store_true")
    args = parser.parse_args(raw_argv)

    rospy.init_node("odin_brain", anonymous=False, disable_signals=False)

    places = rospy.get_param("~places", args.places) or default_places_path()
    dry_run = _as_bool(rospy.get_param("~dry_run", args.dry_run))
    no_llm = _as_bool(rospy.get_param("~no_llm", args.no_llm))
    disable_motion = _as_bool(rospy.get_param("~disable_motion", args.disable_motion))

    BrainNode(places,
              enable_motion=not disable_motion,
              use_llm=not no_llm,
              dry_run=dry_run)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
