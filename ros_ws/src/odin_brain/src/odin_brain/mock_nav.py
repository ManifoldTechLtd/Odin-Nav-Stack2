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

"""ROS-free mock navigation backend for local development, unit tests, and offline verification."""

import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from .models import Plan


class MockNav:
    def __init__(self, per_step_sec: float = 1.5, pose: Optional[Dict] = None):
        self.per_step_sec = per_step_sec
        self._pose = pose or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._lock = threading.Lock()
        self._state = "IDLE"
        self._mission_id = ""
        self._idx = 0
        self._total = 0
        self._started_at = 0.0
        self._events: List[Dict] = []
        self.runtime_map_id = "map_opt"
        self.occupied_boxes: List[Tuple[float, float, float, float]] = []  # (xmin, ymin, xmax, ymax)
        self.health_state: Dict = {"all_ok": True, "bad": [], "note": "OK"}
        self._saved_main: Optional[Dict] = None
        self.missions: List[List[str]] = []

    def wait_ready(self, timeout_sec: float = 10.0) -> Tuple[bool, str]:
        return True, ""

    def snapshot(self) -> Dict:
        with self._lock:
            if self._state == "NAVIGATING":
                elapsed = time.time() - self._started_at
                step_index = min(self._total - 1, int(elapsed / self.per_step_sec))
                if step_index != self._idx:
                    self._idx = step_index
                    self._events.append({
                        "event": "NAVIGATING",
                        "waypoint_idx": self._idx,
                        "waypoint_name": "",
                        "message": "mock reached waypoint %d" % (self._idx + 1),
                        "ts": time.time(),
                    })
                if elapsed >= self.per_step_sec * self._total:
                    if self._saved_main is not None:
                        # Mirror mission_manager::restorePausedMission: return to PAUSED
                        # with the main mission's ID when a temporary mission finishes.
                        self._state = "PAUSED"
                        self._mission_id = self._saved_main["mission_id"]
                        self._total = self._saved_main["total"]
                        self._idx = self._saved_main["idx"]
                        self._saved_main = None
                        self._events.append({
                            "event": "TEMPORARY_DONE",
                            "waypoint_idx": self._idx,
                            "waypoint_name": "",
                            "message": "temporary mission finished, main mission back to PAUSED",
                            "ts": time.time(),
                        })
                    else:
                        self._state = "COMPLETED"
                        self._events.append({
                            "event": "COMPLETED",
                            "waypoint_idx": self._idx,
                            "waypoint_name": "",
                            "message": "mock mission completed",
                            "ts": time.time(),
                        })
            return {
                "ok": True,
                "state": self._state,
                "mission_id": self._mission_id,
                "idx": self._idx,
                "total": self._total,
                "dist_m": 0.0,
                "progress": 100.0 if self._state == "COMPLETED" else 0.0,
                "dwell_remaining": 0.0,
                "v": 0.0,
                "w": 0.0,
                "age": 0.0,
            }

    def start_mission(self, plan: Plan, loop: bool = False,
                      temporary: bool = False) -> Tuple[bool, str, str]:
        waypoints = plan.waypoints()
        if not waypoints:
            return False, "计划里没有可下发的航点", ""
        with self._lock:
            if temporary:
                if self._state != "PAUSED":
                    return False, "temporary 只能在 PAUSED 下发起（当前 %s）" % self._state, ""
                self._saved_main = {
                    "mission_id": self._mission_id,
                    "total": self._total,
                    "idx": self._idx,
                }
            self._state = "NAVIGATING"
            self._mission_id = "mock_%d" % int(time.time() * 1000)
            self._idx = 0
            self._total = len(waypoints)
            self._started_at = time.time()
            self.missions.append([str(wp.get("name", "")) for wp in waypoints])
            self._events.append({
                "event": "NAVIGATING",
                "waypoint_idx": 0,
                "waypoint_name": waypoints[0]["name"],
                "message": "mock started %d waypoints" % len(waypoints),
                "ts": time.time(),
            })
            mission_id = self._mission_id
        return True, "mock mission started", mission_id

    def trigger(self, name: str, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        with self._lock:
            if name == "cancel":
                self._state = "CANCELLED"
            elif name == "pause":
                self._state = "PAUSED"
            elif name == "resume":
                self._state = "NAVIGATING"
                self._started_at = time.time() - self._idx * self.per_step_sec
            elif name == "return_home":
                self._state = "RETURNING_HOME"
                self._events.append({
                    "event": "RETURNING_HOME", "waypoint_idx": 0, "waypoint_name": "起点",
                    "message": "mock return home", "ts": time.time(),
                })
        return True, "mock %s ok" % name

    def wait_mission(self, mission_id: str, timeout_sec: float,
                     on_tick: Optional[Callable[[Dict, List[Dict]], None]] = None,
                     poll_sec: float = 0.4) -> str:
        deadline = time.time() + timeout_sec
        last_key = None
        while time.time() < deadline:
            snapshot = self.snapshot()
            events = self.drain_events()
            if on_tick is not None:
                key = (snapshot.get("state"), snapshot.get("idx"))
                if events or key != last_key:
                    on_tick(snapshot, events)
                    last_key = key
            if snapshot["state"] in ("COMPLETED", "FAILED", "CANCELLED"):
                return snapshot["state"]
            time.sleep(poll_sec)
        return "TIMEOUT"

    def drain_events(self) -> List[Dict]:
        with self._lock:
            items = list(self._events)
            self._events.clear()
        return items

    def get_pose(self) -> Optional[Dict]:
        return dict(self._pose)

    def get_runtime_map_id(self) -> str:
        return self.runtime_map_id

    def get_health(self) -> Dict:
        return dict(self.health_state)

    def precheck(self, x: float, y: float, yaw: float = 0.0) -> Tuple[bool, str, float]:
        for xmin, ymin, xmax, ymax in self.occupied_boxes:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return False, "GOAL_OCCUPIED", 0.0
        pose = self.get_pose() or {"x": 0.0, "y": 0.0}
        dist = math.hypot(float(x) - float(pose["x"]), float(y) - float(pose["y"]))
        return True, "OK", dist

    def turn(self, angle_rad: float, timeout_sec: float = 40.0):
        self._pose["yaw"] = float(self._pose.get("yaw", 0.0)) + float(angle_rad)
        return True, "mock turned %.2f rad" % angle_rad

    def move_straight(self, distance_m: float, timeout_sec: float = 40.0):
        return True, "mock moved %.2fm" % distance_m
