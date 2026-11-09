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

"""Resident ROS 2 node for odin_brain subscribing to /brain/command and executing via BrainEngine."""

import argparse
import json
import queue
import threading
import time
from typing import Optional

from .engine import BrainEngine
from .llm_client import build_client
from .places import PlaceStore
from .planner import RulePlanner


class BrainNodeRos2(object):
    def __init__(self, places_path: str, enable_motion: bool = True,
                 use_llm: bool = True, dry_run: bool = False):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        from .ros2_nav import Ros2Nav

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()

        self.store = PlaceStore(places_path)
        self.planner = RulePlanner(self.store)
        self.llm = build_client() if use_llm else None
        self.nav = Ros2Nav(node_name="odin_brain")
        self._node = self.nav._node
        self.motion_enabled = bool(enable_motion)
        self.base_dry_run = bool(dry_run)
        self.engine = BrainEngine(
            self.store, self.planner, self.llm, self.nav,
            dry_run=(self.base_dry_run or not self.motion_enabled),
            verbose=True,
        )
        self._queue: "queue.Queue[str]" = queue.Queue(maxsize=4)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._result_pub = self._node.create_publisher(String, "/brain/result", 10)
        self._speech_pub = self._node.create_publisher(String, "/brain/speech", 10)
        self._ready_pub = self._node.create_publisher(Bool, "/brain/ready", latched_qos)
        self._node.create_subscription(String, "/brain/command", self._on_command, 10)
        self._node.create_subscription(Bool, "/brain/enable", self._on_enable, latched_qos)

        self._worker = threading.Thread(target=self._work, daemon=True)
        self._worker.start()

        self._ready_pub.publish(Bool(data=True))
        self._node.get_logger().info(
            "odin_brain (ROS2) ready: places=%s(%d) llm=%s motion=%s dry_run=%s" % (
                places_path, len(self.store.places),
                self.llm.config.describe() if self.llm and self.llm.available else "rule",
                self.motion_enabled, self.base_dry_run,
            )
        )

    def _on_enable(self, msg) -> None:
        self.motion_enabled = bool(msg.data)
        self.engine.dry_run = self.base_dry_run or not self.motion_enabled
        self.engine.executor.dry_run = self.engine.dry_run

    def _on_command(self, msg) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self._publish("chat", "上一条还在处理，请稍候再说", ok=False, state="BUSY", plan=[])

    def _work(self) -> None:
        while self._rclpy.ok():
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                result = self.engine.handle(text)
                self._publish(result.intent, result.reply, result.ok, result.state,
                              result.plan_payload(), result.mission_id, result.source)
            except Exception as exc:
                self._publish("error", "内部错误：%s" % exc, ok=False, state="ERROR", plan=[])

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


def main(argv: Optional[list] = None) -> int:
    from .cli import default_places_path
    parser = argparse.ArgumentParser(prog="brain_node", description="常驻指令节点（ROS2）")
    parser.add_argument("--places", default="")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--disable-motion", action="store_true")
    args, _ = parser.parse_known_args(argv)

    node = BrainNodeRos2(
        args.places or default_places_path(),
        enable_motion=not args.disable_motion,
        use_llm=not args.no_llm,
        dry_run=args.dry_run,
    )
    try:
        while node._rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
