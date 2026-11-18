"""Stall (stop / creep) detection over a sliding window of odometry."""

import math
import threading
import time
from collections import deque
from typing import Optional, Tuple


class StallDetector:
    """Detects robot complete stop or creeping-in-place over a sliding time window."""

    ACTIVE_MISSION_STATES = frozenset({
        "NAVIGATING",
        "RETURNING_HOME",
        "ALIGNING",
        "RECOVERING",
        "EXECUTING",
        "MOVING",
    })

    def __init__(
        self,
        window_sec: float = 30.0,
        stall_radius_m: float = 0.20,
        cmd_active_hold_sec: float = 10.0,
        odom_stale_sec: float = 10.0,
    ):
        self.window_sec = float(window_sec)
        self.stall_radius_m = float(stall_radius_m)
        self.cmd_active_hold_sec = float(cmd_active_hold_sec)
        self.odom_stale_sec = float(odom_stale_sec)

        self._lock = threading.Lock()
        self.mission_state = "UNKNOWN"
        self.distance_to_target_m: Optional[float] = None
        self.progress_percent: Optional[float] = None
        self.current_waypoint_idx: Optional[int] = None
        self.total_waypoints: Optional[int] = None
        self.force_monitor = False

        self.poses = deque()  # entries: (t, x, y, vx, vy, vyaw)
        self.last_odom: Optional[dict] = None
        self.last_cmd_vel: Optional[dict] = None
        self.last_nonzero_cmd_at: Optional[float] = None

        self.active_since: Optional[float] = None
        self.stall_anchor: Optional[Tuple[float, float, float]] = None  # (t, x, y)
        self.muted_until: float = 0.0

    def set_config(self, window_sec: Optional[float] = None, stall_radius_m: Optional[float] = None) -> None:
        with self._lock:
            if window_sec is not None and window_sec >= 5.0:
                self.window_sec = float(window_sec)
            if stall_radius_m is not None and stall_radius_m > 0.01:
                self.stall_radius_m = float(stall_radius_m)

    def set_force_monitor(self, enabled: bool, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.force_monitor = bool(enabled)
            if self.force_monitor and self.active_since is None:
                self.active_since = now
                if self.last_odom is not None:
                    self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])
            elif not self.force_monitor and not self._is_auto_active_locked(now):
                self.active_since = None
                self.stall_anchor = None

    def acknowledge_alarm(self, mute_sec: float = 60.0, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.muted_until = now + max(0.0, float(mute_sec))

    def update_mission_state(
        self,
        state: str,
        distance_to_target_m: Optional[float] = None,
        progress_percent: Optional[float] = None,
        current_waypoint_idx: Optional[int] = None,
        total_waypoints: Optional[int] = None,
        now: Optional[float] = None,
    ) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            self.mission_state = (state or "UNKNOWN").strip().upper()
            if distance_to_target_m is not None:
                self.distance_to_target_m = float(distance_to_target_m)
            if progress_percent is not None:
                self.progress_percent = float(progress_percent)
            if current_waypoint_idx is not None:
                self.current_waypoint_idx = int(current_waypoint_idx)
            if total_waypoints is not None:
                self.total_waypoints = int(total_waypoints)
            if self._is_monitoring_active_locked(now):
                if self.active_since is None:
                    self.active_since = now
                    if self.last_odom is not None:
                        self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])
            else:
                self.active_since = None
                self.stall_anchor = None

    def update_cmd_vel(self, vx: float, vy: float, vyaw: float, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            speed = math.hypot(float(vx), float(vy))
            self.last_cmd_vel = {
                "t": now,
                "vx": float(vx),
                "vy": float(vy),
                "vyaw": float(vyaw),
            }
            if speed > 0.02 or abs(float(vyaw)) > 0.05:
                self.last_nonzero_cmd_at = now
                if self.active_since is None:
                    self.active_since = now
                    if self.last_odom is not None:
                        self.stall_anchor = (now, self.last_odom["x"], self.last_odom["y"])

    def update_odom(
        self,
        x: float,
        y: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        yaw: float = 0.0,
        now: Optional[float] = None,
    ) -> None:
        now = time.time() if now is None else float(now)
        with self._lock:
            x, y = float(x), float(y)
            vx, vy, vyaw, yaw = float(vx), float(vy), float(vyaw), float(yaw)
            self.last_odom = {
                "t": now,
                "x": x,
                "y": y,
                "yaw": yaw,
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
            }
            self.poses.append((now, x, y, vx, vy, vyaw))
            cutoff = now - max(self.window_sec * 2.0, 120.0)
            while self.poses and self.poses[0][0] < cutoff:
                self.poses.popleft()

            if self._is_monitoring_active_locked(now):
                if self.active_since is None:
                    self.active_since = now
                if self.stall_anchor is None:
                    self.stall_anchor = (now, x, y)
                else:
                    dist_from_anchor = math.hypot(x - self.stall_anchor[1], y - self.stall_anchor[2])
                    if dist_from_anchor >= self.stall_radius_m:
                        self.stall_anchor = (now, x, y)

    def _is_auto_active_locked(self, now: float) -> bool:
        if self.mission_state in self.ACTIVE_MISSION_STATES:
            return True
        if self.last_nonzero_cmd_at is not None and (now - self.last_nonzero_cmd_at) <= self.cmd_active_hold_sec:
            return True
        return False

    def _is_monitoring_active_locked(self, now: float) -> bool:
        return self.force_monitor or self._is_auto_active_locked(now)

    def evaluate(self, now: Optional[float] = None) -> dict:
        now = time.time() if now is None else float(now)
        with self._lock:
            active = self._is_monitoring_active_locked(now)
            if not active:
                self.active_since = None
                self.stall_anchor = None
                return self._build_result_locked(
                    now=now,
                    monitoring_active=False,
                    alarm=False,
                    alarm_type="NONE",
                    alarm_message="待机中（导航启动后自动监控）",
                    stall_duration_sec=0.0,
                    max_displacement_m=0.0,
                    path_length_m=0.0,
                )

            if self.active_since is None:
                self.active_since = now

            window_start = now - self.window_sec
            window_poses = [p for p in self.poses if p[0] >= window_start]

            odom_age = (now - self.last_odom["t"]) if self.last_odom else None
            if not window_poses or (odom_age is not None and odom_age > self.odom_stale_sec):
                active_dur = now - self.active_since
                is_alarm = active_dur >= self.window_sec
                return self._build_result_locked(
                    now=now,
                    monitoring_active=True,
                    alarm=is_alarm,
                    alarm_type="ODOM_STALE" if is_alarm else "NONE",
                    alarm_message=(
                        f"警告：里程计数据断流或超时已达 {active_dur:.1f}s！"
                        if is_alarm
                        else f"等待里程计数据（已监控 {active_dur:.1f}s）"
                    ),
                    stall_duration_sec=active_dur,
                    max_displacement_m=0.0,
                    path_length_m=0.0,
                )

            x0, y0 = window_poses[0][1], window_poses[0][2]
            max_disp = 0.0
            path_len = 0.0
            max_speed = 0.0
            max_yaw_rate = 0.0
            prev_x, prev_y = x0, y0

            for _, px, py, pvx, pvy, pvyaw in window_poses:
                d = math.hypot(px - x0, py - y0)
                if d > max_disp:
                    max_disp = d
                path_len += math.hypot(px - prev_x, py - prev_y)
                prev_x, prev_y = px, py
                spd = math.hypot(pvx, pvy)
                if spd > max_speed:
                    max_speed = spd
                if abs(pvyaw) > max_yaw_rate:
                    max_yaw_rate = abs(pvyaw)

            # Calculate how long the robot has stayed within stall_radius_m
            anchor_age = (now - self.stall_anchor[0]) if self.stall_anchor else (now - window_poses[0][0])
            window_span = now - window_poses[0][0]
            effective_stall_sec = max(anchor_age, window_span if max_disp < self.stall_radius_m else 0.0)
            active_duration = now - self.active_since
            stall_duration_sec = min(active_duration, effective_stall_sec)

            if active_duration >= self.window_sec and max_disp < self.stall_radius_m:
                is_creeping = (path_len >= 0.04) or (max_speed >= 0.03) or (max_yaw_rate >= 0.08)
                alarm_type = "CREEPING" if is_creeping else "STOPPED"
                label = "原地蠕动/受阻" if is_creeping else "停止不前"
                msg = (
                    f"警报：机械狗已{label} {stall_duration_sec:.1f} 秒！"
                    f"（近 {self.window_sec:.0f}s 净位移仅 {max_disp:.2f}m < 阈值 {self.stall_radius_m:.2f}m，"
                    f"累计轨迹 {path_len:.2f}m）"
                )
                return self._build_result_locked(
                    now=now,
                    monitoring_active=True,
                    alarm=True,
                    alarm_type=alarm_type,
                    alarm_message=msg,
                    stall_duration_sec=stall_duration_sec,
                    max_displacement_m=max_disp,
                    path_length_m=path_len,
                )

            return self._build_result_locked(
                now=now,
                monitoring_active=True,
                alarm=False,
                alarm_type="NONE",
                alarm_message=f"运行正常（近 {min(window_span, self.window_sec):.0f}s 净位移 {max_disp:.2f}m）",
                stall_duration_sec=stall_duration_sec if max_disp < self.stall_radius_m else 0.0,
                max_displacement_m=max_disp,
                path_length_m=path_len,
            )

    def _build_result_locked(
        self,
        now: float,
        monitoring_active: bool,
        alarm: bool,
        alarm_type: str,
        alarm_message: str,
        stall_duration_sec: float,
        max_displacement_m: float,
        path_length_m: float,
    ) -> dict:
        odom_age = (now - self.last_odom["t"]) if self.last_odom else None
        return {
            "monitoring_active": monitoring_active,
            "force_monitor": self.force_monitor,
            "mission_state": self.mission_state,
            "distance_to_target_m": self.distance_to_target_m,
            "progress_percent": self.progress_percent,
            "current_waypoint_idx": self.current_waypoint_idx,
            "total_waypoints": self.total_waypoints,
            "alarm": alarm,
            "alarm_type": alarm_type,
            "alarm_message": alarm_message,
            "buzzer_muted": now < self.muted_until,
            "muted_remaining_sec": max(0.0, round(self.muted_until - now, 1)),
            "window_sec": self.window_sec,
            "stall_radius_m": self.stall_radius_m,
            "stall_duration_sec": round(stall_duration_sec, 1),
            "max_displacement_m": round(max_displacement_m, 3),
            "path_length_m": round(path_length_m, 3),
            "odom": self.last_odom,
            "odom_age_sec": round(odom_age, 2) if odom_age is not None else None,
            "cmd_vel": self.last_cmd_vel,
        }


