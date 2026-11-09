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

"""ROS 1 (Noetic) navigation adapter using the /waypoint_manager/* contract.

Contract:
    Services  /waypoint_manager/start_mission                    nav_interfaces/StartMission
              /waypoint_manager/{pause,resume,cancel,return_home,reset} std_srvs/Trigger
    Topics    /waypoint_manager/status                           nav_interfaces/MissionStatus (latched, 2Hz)
              /waypoint_manager/mission_feedback                 nav_interfaces/MissionEvent
              /waypoint_manager/operator_command                 nav_interfaces/OperatorCommand
"""

import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .models import Plan

TERMINAL_STATES = ("COMPLETED", "FAILED", "CANCELLED")


class NavUnavailable(Exception):
    """Raised when the navigation stack is unavailable or unready."""


class Ros1Nav(object):
    """rospy navigation client (must be constructed after rospy.init_node())."""

    def __init__(self, map_frame: str = "map", base_frame: str = "base_link",
                 node_name: str = "odin_brain_nav"):
        import rospy
        from nav_interfaces.msg import MissionStatus, MissionEvent, OperatorCommand
        from nav_interfaces.srv import StartMission
        from std_srvs.srv import Trigger

        self._rospy = rospy
        self._Trigger = Trigger
        self.map_frame = map_frame
        self.base_frame = base_frame

        self._lock = threading.Lock()
        self._status = None
        self._status_at = 0.0
        self._events: Deque[Dict] = deque(maxlen=50)

        self._sub_status = rospy.Subscriber("/waypoint_manager/status", MissionStatus,
                                            self._on_status, queue_size=1)
        self._sub_event = rospy.Subscriber("/waypoint_manager/mission_feedback", MissionEvent,
                                           self._on_event, queue_size=20)
        self._sub_command = rospy.Subscriber("/waypoint_manager/operator_command",
                                             OperatorCommand, self._on_command, queue_size=20)

        self._start_proxy = rospy.ServiceProxy("/waypoint_manager/start_mission", StartMission)
        self._trigger_proxies = {
            name: rospy.ServiceProxy("/waypoint_manager/%s" % name, Trigger)
            for name in ("pause", "resume", "cancel", "return_home", "reset")
        }

        # Atomic motion services (in-place turn / straight move)
        self._turn_proxy = None
        self._move_proxy = None
        try:
            from atomic_move.srv import Turn, Move
            self._turn_proxy = rospy.ServiceProxy("/atomic_move/turn", Turn)
            self._move_proxy = rospy.ServiceProxy("/atomic_move/move", Move)
        except Exception:
            self._turn_proxy = None
            self._move_proxy = None

        # Precheck (/map_planner/plan, shadow_only=true) and system health
        self._health = None
        self._plan_proxy = None
        try:
            from nav_interfaces.srv import PlanPath
            self._plan_proxy = rospy.ServiceProxy("/map_planner/plan", PlanPath)
        except Exception:
            self._plan_proxy = None

        self._sub_health = None
        try:
            from nav_interfaces.msg import SystemHealth
            self._sub_health = rospy.Subscriber(
                "/system_health", SystemHealth, self._on_health, queue_size=1)
        except Exception:
            self._sub_health = None

        self._tf_buffer = None
        self._tf_listener = None
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        except Exception:
            self._tf_buffer = None

        self._node_name = node_name

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

    def _on_health(self, msg) -> None:
        with self._lock:
            self._health = msg

    def get_health(self) -> Dict:
        with self._lock:
            msg = self._health
        if msg is None:
            return {"all_ok": True, "bad": [], "note": "未收到健康话题"}
        bad = []
        for node in getattr(msg, "nodes", []):
            if getattr(node, "status", "OK") != "OK":
                bad.append({
                    "name": str(getattr(node, "name", "")),
                    "status": str(getattr(node, "status", "")),
                    "cause": str(getattr(node, "cause", "")),
                    "action": str(getattr(node, "action", "")),
                })
        return {
            "all_ok": bool(getattr(msg, "all_ok", len(bad) == 0)),
            "bad": bad,
            "note": str(getattr(msg, "system_note", "")),
        }

    def get_runtime_map_id(self) -> str:
        """Return runtime map ID from MAP_FILE_NAME env or /map_planner/map_id parameter."""
        import os
        env_map = os.environ.get("MAP_FILE_NAME", "").strip()
        if env_map:
            return os.path.splitext(os.path.basename(env_map))[0]
        try:
            param_map = str(self._rospy.get_param("/map_planner/map_id", "") or "").strip()
            if param_map:
                return param_map
        except Exception:
            pass
        return ""

    def precheck(self, x: float, y: float, yaw: float = 0.0) -> Tuple[bool, str, float]:
        """Validate goal reachability via /map_planner/plan (shadow_only=True)."""
        if self._plan_proxy is None:
            return True, "SKIP_NO_PROXY", 0.0
        if not self._service_ready(self._plan_proxy, timeout_sec=1.5):
            return True, "SKIP_SERVICE_DOWN", 0.0
        try:
            import math
            from geometry_msgs.msg import PoseStamped
            from nav_interfaces.srv import PlanPath
            req = PlanPath._request_class()
            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = self._rospy.Time.now()
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = math.sin(float(yaw) * 0.5)
            goal.pose.orientation.w = math.cos(float(yaw) * 0.5)
            req.goal = goal
            req.shadow_only = True
            res = self._plan_proxy(req)
            if not res.success:
                return False, str(res.message or "NO_PATH"), 0.0
            poses = getattr(res.path, "poses", []) or []
            length = 0.0
            for i in range(1, len(poses)):
                p0 = poses[i - 1].pose.position
                p1 = poses[i].pose.position
                length += math.hypot(p1.x - p0.x, p1.y - p0.y)
            return True, str(res.message or "OK"), length
        except Exception as exc:
            return True, "SKIP_ERROR:%s" % exc, 0.0

    def status_age(self) -> float:
        with self._lock:
            if self._status is None:
                return float("inf")
            return time.time() - self._status_at

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

    def drain_events(self) -> List[Dict]:
        with self._lock:
            items = list(self._events)
            self._events.clear()
        return items

    def get_pose(self) -> Optional[Dict]:
        """Look up map -> base_link transform and return {x, y, yaw}."""
        if self._tf_buffer is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, self._rospy.Time(0),
                self._rospy.Duration(0.5))
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return {"x": float(translation.x), "y": float(translation.y), "yaw": float(yaw)}

    def wait_ready(self, timeout_sec: float = 10.0) -> Tuple[bool, str]:
        deadline = time.time() + timeout_sec
        service_ok = False
        while time.time() < deadline:
            service_ok = self._service_ready(self._start_proxy, timeout_sec=0.5)
            if service_ok and self._status is not None:
                return True, ""
            time.sleep(0.1)
        missing = []
        if not service_ok:
            missing.append("服务 /waypoint_manager/start_mission")
        if self._status is None:
            missing.append("话题 /waypoint_manager/status")
        return False, "导航栈未就绪：缺少 " + "、".join(missing)

    def start_mission(self, plan: Plan, loop: bool = False,
                      temporary: bool = False) -> Tuple[bool, str, str]:
        from nav_interfaces.srv import StartMission
        from nav_interfaces.msg import Waypoint

        waypoints = []
        for item in plan.waypoints():
            waypoint = Waypoint()
            waypoint.x = float(item["x"])
            waypoint.y = float(item["y"])
            waypoint.yaw = float(item["yaw"])
            waypoint.dwell_sec = float(item["dwell_sec"])
            waypoint.name = str(item["name"])
            waypoints.append(waypoint)

        if not waypoints:
            return False, "计划里没有可下发的航点（可能只有返航步骤）", ""

        if not self._service_ready(self._start_proxy, timeout_sec=3.0):
            return False, "服务 /waypoint_manager/start_mission 不可用", ""

        request = StartMission._request_class()
        request.waypoints = waypoints
        request.loop = bool(loop)
        request.temporary = bool(temporary)
        try:
            response = self._start_proxy(request)
        except Exception as exc:
            return False, "调用 start_mission 失败：%s" % exc, ""
        return bool(response.success), str(response.message), str(response.mission_id)

    def turn(self, angle_rad: float, timeout_sec: float = 40.0) -> Tuple[bool, str]:
        """In-place rotation (radians, positive = CCW / left) via /atomic_move/turn."""
        if self._turn_proxy is None:
            return False, "atomic_move/turn 不可用"
        if not self._service_ready(self._turn_proxy, timeout_sec=3.0):
            return False, "服务 /atomic_move/turn 不可用"
        try:
            response = self._turn_proxy(float(angle_rad))
        except Exception as exc:
            return False, "调用 turn 失败：%s" % exc
        return bool(response.success), str(response.message)

    def move_straight(self, distance_m: float, timeout_sec: float = 40.0) -> Tuple[bool, str]:
        """Straight-line displacement (meters, positive = forward) via /atomic_move/move."""
        if self._move_proxy is None:
            return False, "atomic_move/move 不可用"
        if not self._service_ready(self._move_proxy, timeout_sec=3.0):
            return False, "服务 /atomic_move/move 不可用"
        try:
            response = self._move_proxy(float(distance_m))
        except Exception as exc:
            return False, "调用 move 失败：%s" % exc
        return bool(response.success), str(response.message)

    def trigger(self, name: str, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        proxy = self._trigger_proxies.get(name)
        if proxy is None:
            return False, "未知操作: %s" % name
        if not self._service_ready(proxy, timeout_sec=timeout_sec):
            return False, "服务 /waypoint_manager/%s 不可用" % name
        try:
            response = proxy(self._Trigger._request_class())
        except Exception as exc:
            return False, "调用 %s 失败：%s" % (name, exc)
        return bool(response.success), str(response.message)

    @staticmethod
    def _service_ready(proxy, timeout_sec: float) -> bool:
        try:
            proxy.wait_for_service(timeout=timeout_sec)
            return True
        except Exception:
            return False

    def wait_mission(self, mission_id: str, timeout_sec: float,
                     on_tick: Optional[Callable[[Dict, List[Dict]], None]] = None,
                     poll_sec: float = 0.4) -> str:
        """Wait for mission completion and return terminal state, TIMEOUT, or SUPERSEDED."""
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


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    import math
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
