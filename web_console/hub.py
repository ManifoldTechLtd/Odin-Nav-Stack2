"""Aggregates pose, map, safety, camera, intercom, mission and control state for the HTTP API."""

import json
import threading
import time
from typing import Callable, Optional

from web_console.camera import OnDemandImageGate
from web_console.core import ConsoleState
from web_console.intercom import AudioIntercom
from web_console.mission import MissionController
from web_console.safety import StallDetector
from web_console.teleop import NavigationAndTeleopController

API_VERSION = 2


class MonitorStateHub:
    """Central thread-safe state hub shared between ROS callbacks and the HTTP server."""

    def __init__(self, stall_detector: Optional[StallDetector] = None,
                 intercom: Optional[AudioIntercom] = None):
        self.console = ConsoleState()
        self.stall = stall_detector or StallDetector()
        self.camera = OnDemandImageGate(subscribe_fn=lambda: None, unsubscribe_fn=lambda: None, default_fps=2.0)
        self.intercom = intercom or AudioIntercom()
        self.nav = NavigationAndTeleopController()
        self.mission = MissionController()
        self._lock = threading.Lock()
        self.system_health: Optional[dict] = None
        self.started_at = time.time()

    def bind(self, call_trigger: Callable, publish_cmd_vel: Callable, publish_neupan_stop: Callable,
             publish_estop: Callable, subscribe_image: Callable, unsubscribe_image: Callable,
             start_mission: Callable, set_chassis: Callable) -> None:
        self.camera._subscribe_fn = subscribe_image
        self.camera._unsubscribe_fn = unsubscribe_image
        self.nav._call_service_fn = call_trigger
        self.nav._publish_cmd_vel_fn = publish_cmd_vel
        self.nav._publish_neupan_stop_fn = publish_neupan_stop
        self.nav._publish_estop_fn = publish_estop
        self.mission.bind(start_mission, call_trigger, set_chassis)

    def on_odom(self, x, y, vx, vy, vyaw, yaw) -> None:
        self.stall.update_odom(x, y, vx=vx, vy=vy, vyaw=vyaw, yaw=yaw)

    def on_cmd_vel(self, vx, vy, vyaw) -> None:
        self.stall.update_cmd_vel(vx, vy, vyaw)

    def on_mission_status(self, status: dict) -> None:
        self.mission.update_status(status)
        self.stall.update_mission_state(
            status["state"],
            distance_to_target_m=status.get("distance_to_target_m"),
            progress_percent=status.get("progress_percent"),
            current_waypoint_idx=status.get("current_waypoint_idx"),
            total_waypoints=status.get("total_waypoints"),
        )

    def update_system_health_json(self, raw_json: str) -> None:
        try:
            parsed = json.loads(raw_json)
            with self._lock:
                self.system_health = parsed
        except Exception:
            pass

    def get_snapshot(self) -> dict:
        with self._lock:
            health = self.system_health
        now = time.time()
        console = self.console.snapshot()
        return {
            "ok": True,
            "api_version": API_VERSION,
            "server_time": now,
            "uptime_sec": round(now - self.started_at, 1),
            "pose": console["pose"],
            "map": console["map"],
            "mission": self.mission.get_status(),
            "safety": self.stall.evaluate(),
            "camera": self.camera.get_status(),
            "intercom": self.intercom.get_status(),
            "control": self.nav.get_status(),
            "system_health": health,
        }
