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

"""ROS 2 (Jazzy / rclpy) navigation client adapter for /waypoint_manager/* contracts."""

import json
import math
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .models import Plan

TERMINAL_STATES = ("COMPLETED", "FAILED", "CANCELLED")


class Ros2Nav(object):
    """rclpy navigation client adapter with an internal background spin thread."""

    def __init__(self, map_frame: str = "map", base_frame: str = "base_link",
                 node_name: str = "odin_brain_nav"):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from nav_interfaces.msg import MissionEvent, MissionStatus, OperatorCommand
        from nav_interfaces.srv import PlanPath, StartMission
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self.map_frame = map_frame
        self.base_frame = base_frame
        self._node: Node = rclpy.create_node(node_name)

        self._lock = threading.Lock()
        self._status = None
        self._status_at = 0.0
        self._health_dict: Dict = {"all_ok": True, "bad": [], "note": "未收到健康话题"}
        self._events: Deque[Dict] = deque(maxlen=50)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._node.create_subscription(
            MissionStatus, "/waypoint_manager/status", self._on_status, latched_qos
        )
        self._node.create_subscription(
            MissionEvent, "/waypoint_manager/mission_feedback", self._on_event, 20
        )
        self._node.create_subscription(
            OperatorCommand, "/waypoint_manager/operator_command", self._on_command, 20
        )
        self._node.create_subscription(
            String, "/waypoint_manager/system_health", self._on_health_json, 10
        )

        self._start_cli = self._node.create_client(StartMission, "/waypoint_manager/start_mission")
        self._plan_cli = self._node.create_client(PlanPath, "/map_planner/plan")
        self._trigger_clis = {
            name: self._node.create_client(Trigger, "/waypoint_manager/%s" % name)
            for name in ("pause", "resume", "cancel", "return_home", "reset")
        }
        self._StartMission = StartMission
        self._PlanPath = PlanPath
        self._Trigger = Trigger

        self._tf_buffer = None
        self._tf_listener = None
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node, spin_thread=False)
        except Exception:
            self._tf_buffer = None

        self._stop_spin = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while not self._stop_spin.is_set() and self._rclpy.ok():
            try:
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                time.sleep(0.05)

    def shutdown(self) -> None:
        self._stop_spin.set()
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass

    def _on_status(self, msg) -> None:
        with self._lock:
            self._status = msg
            self._status_at = time.time()

    def _on_event(self, msg) -> None:
        with self._lock:
            self._events.append({
                "event": msg.event,
                "waypoint_idx": int(msg.waypoint_idx),
                "waypoint_name": msg.waypoint_name,
                "message": msg.message,
                "ts": float(msg.timestamp),
            })

    def _on_command(self, msg) -> None:
        with self._lock:
            self._events.append({
                "event": "OPERATOR_COMMAND",
                "waypoint_idx": -1,
                "waypoint_name": "",
                "message": "%s/%s %s" % (msg.command, msg.phase, msg.message),
                "ts": time.time(),
            })

    def _on_health_json(self, msg) -> None:
        """Parse JSON system health message into the normalized dictionary structure."""
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            return
        nodes = data.get("nodes") or []
        bad = []
        for item in nodes:
            if isinstance(item, dict) and item.get("status", "OK") != "OK":
                bad.append({
                    "name": str(item.get("name", "")),
                    "status": str(item.get("status", "")),
                    "cause": str(item.get("cause", "")),
                    "action": str(item.get("action", "")),
                })
        with self._lock:
            self._health_dict = {
                "all_ok": bool(data.get("all_ok", len(bad) == 0)),
                "bad": bad,
                "note": str(data.get("system_note", "")),
            }

    def snapshot(self) -> Dict:
        with self._lock:
            status = self._status
            if status is None:
                return {"ok": False, "state": "UNKNOWN", "mission_id": "", "age": float("inf")}
            return {
                "ok": True,
                "state": status.state,
                "mission_id": status.mission_id,
                "idx": int(status.current_waypoint_idx),
                "total": int(status.total_waypoints),
                "dist_m": float(status.distance_to_target_m),
                "progress": float(status.progress_percent),
                "dwell_remaining": float(status.dwell_remaining_sec),
                "v": float(status.linear_vel),
                "w": float(status.angular_vel),
                "age": time.time() - self._status_at,
            }

    def get_health(self) -> Dict:
        with self._lock:
            return dict(self._health_dict)

    def drain_events(self) -> List[Dict]:
        with self._lock:
            items = list(self._events)
            self._events.clear()
        return items

    def get_pose(self) -> Optional[Dict]:
        if self._tf_buffer is None:
            return None
        try:
            from rclpy.time import Time
            from rclpy.duration import Duration
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), Duration(seconds=0.5)
            )
            tr = transform.transform.translation
            rot = transform.transform.rotation
            siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y)
            cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return {"x": float(tr.x), "y": float(tr.y), "yaw": float(yaw)}
        except Exception:
            return None

    def wait_ready(self, timeout_sec: float = 10.0) -> Tuple[bool, str]:
        deadline = time.time() + timeout_sec
        service_ok = False
        while time.time() < deadline:
            service_ok = self._start_cli.wait_for_service(timeout_sec=0.3)
            if service_ok and self._status is not None:
                return True, ""
            time.sleep(0.1)
        missing = []
        if not service_ok:
            missing.append("服务 /waypoint_manager/start_mission")
        if self._status is None:
            missing.append("话题 /waypoint_manager/status")
        return False, "导航栈未就绪：缺少 " + "、".join(missing)

    def _call_sync(self, client, request, timeout_sec: float = 5.0):
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            raise TimeoutError("服务调用超时 (%.1fs)" % timeout_sec)
        return future.result()

    def start_mission(self, plan: Plan, loop: bool = False,
                      temporary: bool = False) -> Tuple[bool, str, str]:
        from nav_interfaces.msg import Waypoint

        waypoints = []
        for item in plan.waypoints():
            wp = Waypoint()
            wp.x = float(item["x"])
            wp.y = float(item["y"])
            wp.yaw = float(item["yaw"])
            wp.dwell_sec = float(item["dwell_sec"])
            wp.name = str(item["name"])
            waypoints.append(wp)

        if not waypoints:
            return False, "计划里没有可下发的航点", ""
        if not self._start_cli.wait_for_service(timeout_sec=2.0):
            return False, "服务 /waypoint_manager/start_mission 不可用", ""

        req = self._StartMission.Request()
        req.waypoints = waypoints
        req.loop = bool(loop)
        req.temporary = bool(temporary)
        try:
            res = self._call_sync(self._start_cli, req, timeout_sec=5.0)
        except Exception as exc:
            return False, "调用 start_mission 失败：%s" % exc, ""
        return bool(res.success), str(res.message), str(res.mission_id)

    def trigger(self, name: str, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        cli = self._trigger_clis.get(name)
        if cli is None:
            return False, "未知操作: %s" % name
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, "服务 /waypoint_manager/%s 不可用" % name
        try:
            res = self._call_sync(cli, self._Trigger.Request(), timeout_sec=timeout_sec)
        except Exception as exc:
            return False, "调用 %s 失败：%s" % (name, exc)
        return bool(res.success), str(res.message)

    def wait_mission(self, mission_id: str, timeout_sec: float,
                     on_tick: Optional[Callable[[Dict, List[Dict]], None]] = None,
                     poll_sec: float = 0.4) -> str:
        deadline = time.time() + timeout_sec
        grace_until = time.time() + 5.0
        last_key = None
        while time.time() < deadline:
            snapshot = self.snapshot()
            events = self.drain_events()
            if on_tick is not None:
                key = (snapshot.get("state"), snapshot.get("idx"), snapshot.get("dist_m"))
                if events or key != last_key:
                    on_tick(snapshot, events)
                    last_key = key
            state = snapshot.get("state", "UNKNOWN")
            same_mission = snapshot.get("mission_id", "") == mission_id
            if same_mission and state in TERMINAL_STATES:
                return state
            if (not same_mission) and state in TERMINAL_STATES and time.time() > grace_until:
                return "SUPERSEDED"
            time.sleep(poll_sec)
        return "TIMEOUT"
