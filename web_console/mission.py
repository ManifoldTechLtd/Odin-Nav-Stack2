"""Waypoint mission state and commands (start, return home, reset)."""

import threading
from typing import Callable, Optional

MAX_WAYPOINTS = 50
TRIGGER_SERVICES = {
    "return_home": "/waypoint_manager/return_home",
    "reset": "/waypoint_manager/reset",
}


def parse_waypoints(raw):
    if not isinstance(raw, list) or not raw:
        raise ValueError("waypoints 必须是非空列表")
    if len(raw) > MAX_WAYPOINTS:
        raise ValueError("单次任务最多 {} 个点".format(MAX_WAYPOINTS))
    points = []
    for idx, item in enumerate(raw, start=1):
        try:
            points.append({
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": float(item.get("yaw", 0.0)),
                "dwell_sec": max(0.0, float(item.get("dwell_sec", 0.0))),
                "name": str(item.get("name") or "P{}".format(idx)),
            })
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("第 {} 个点无效: {}".format(idx, exc))
    return points


class MissionController:
    def __init__(self):
        self._lock = threading.Lock()
        self._status = None
        self._start_fn: Optional[Callable] = None
        self._trigger_fn: Optional[Callable[[str], tuple]] = None

    def bind(self, start_fn, trigger_fn):
        self._start_fn = start_fn
        self._trigger_fn = trigger_fn

    def update_status(self, status):
        with self._lock:
            self._status = status

    def start(self, raw_waypoints, loop=False, temporary=False):
        try:
            points = parse_waypoints(raw_waypoints)
        except ValueError as exc:
            return {"ok": False, "message": str(exc)}
        if self._start_fn is None:
            return {"ok": False, "message": "任务接口未绑定"}
        ok, msg, mission_id = self._start_fn(points, bool(loop), bool(temporary))
        return {"ok": ok, "message": msg, "mission_id": mission_id}

    def command(self, action):
        if action in TRIGGER_SERVICES and self._trigger_fn is not None:
            ok, msg = self._trigger_fn(TRIGGER_SERVICES[action])
            return {"ok": ok, "message": msg}
        return {"ok": False, "message": "未知任务指令: {}".format(action)}

    def get_status(self):
        with self._lock:
            return {"status": self._status}
