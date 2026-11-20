"""Aggregates pose, map, safety, camera, intercom, mission and control state for the HTTP API."""

import json
import os
import threading
import time
from typing import Callable, Optional

from web_console.camera import OnDemandImageGate
from web_console.core import ConsoleState, load_pgm_yaml
from web_console.intercom import AudioIntercom
from web_console.mission import MissionController
from web_console.recorder import HEAVY_TYPES, BagRecorder
from web_console.safety import StallDetector
from web_console.stack import NavStack
from web_console.teleop import NavigationAndTeleopController

API_VERSION = 2
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BAG_ROOT = os.environ.get("ONS2_BAG_ROOT", os.path.expanduser("~/sw01_bag_recordings"))


class MonitorStateHub:
    """Central thread-safe state hub shared between ROS callbacks and the HTTP server."""

    def __init__(self, stall_detector: Optional[StallDetector] = None,
                 intercom: Optional[AudioIntercom] = None):
        self.console = ConsoleState()
        self.stall = stall_detector or StallDetector()
        self.camera = OnDemandImageGate(subscribe_fn=lambda: None, unsubscribe_fn=lambda: None, default_fps=8.0)
        self.intercom = intercom or AudioIntercom()
        self.nav = NavigationAndTeleopController()
        self.mission = MissionController()
        self.stack = NavStack(repo_root=REPO_ROOT)
        self.recorder = BagRecorder(bag_root=BAG_ROOT, repo_root=REPO_ROOT)
        self._list_topics: Callable[[], dict] = lambda: {}
        self._lock = threading.Lock()
        self.system_health: Optional[dict] = None
        self.started_at = time.time()
        self._ros_bound = False
        self._last_disk_map_check = 0.0
        self._loaded_disk_yaml: Optional[str] = None

    def bind(self, call_trigger: Callable, publish_cmd_vel: Callable, publish_neupan_stop: Callable,
             publish_estop: Callable, subscribe_image: Callable, unsubscribe_image: Callable,
             start_mission: Callable, set_chassis: Callable, list_topics: Optional[Callable] = None) -> None:
        if list_topics is not None:
            self._list_topics = list_topics
            self._ros_bound = True
        self.camera._subscribe_fn = subscribe_image
        self.camera._unsubscribe_fn = unsubscribe_image
        self.nav._call_service_fn = call_trigger
        self.nav._publish_cmd_vel_fn = publish_cmd_vel
        self.nav._publish_neupan_stop_fn = publish_neupan_stop
        self.nav._publish_estop_fn = publish_estop
        self.mission.bind(start_mission, call_trigger, set_chassis)

    def topic_types(self) -> dict:
        return {name: list(types) for name, types in self._list_topics().items()}

    def topics(self) -> list:
        return [{"name": n, "types": t, "heavy": any(x in HEAVY_TYPES for x in t)}
                for n, t in sorted(self.topic_types().items())]

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

    def _maybe_load_disk_map(self, now: float) -> None:
        if now - self._last_disk_map_check < 2.0:
            return
        self._last_disk_map_check = now
        yml = self.stack.active_map_yaml()
        if not yml:
            return
        if self.console.snapshot()["map"] is not None and yml == self._loaded_disk_yaml:
            return
        loaded = load_pgm_yaml(yml)
        if loaded is not None:
            self._loaded_disk_yaml = yml
            self.console.set_map(*loaded)

    def get_snapshot(self) -> dict:
        with self._lock:
            health = self.system_health
        now = time.time()
        if self._ros_bound:
            self._maybe_load_disk_map(now)
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
