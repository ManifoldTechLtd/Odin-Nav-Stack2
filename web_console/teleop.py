"""Emergency stop, navigation mission controls and manual teleop takeover."""

import math
import threading
import time
from typing import Callable, Optional, Tuple


class NavigationAndTeleopController:
    """Manages Emergency Stop, Navigation Mission Controls (Pause/Resume/Skip/Cancel), and Manual Teleop Takeover."""

    def __init__(
        self,
        call_service_fn: Optional[Callable[[str], Tuple[bool, str]]] = None,
        publish_cmd_vel_fn: Optional[Callable[[float, float, float], None]] = None,
        publish_neupan_stop_fn: Optional[Callable[[], None]] = None,
        publish_estop_fn: Optional[Callable[[bool], None]] = None,
        cmd_timeout_sec: float = 0.6,
        max_linear_vel: float = 0.6,
        max_angular_vel: float = 0.8,
    ):
        self._call_service_fn = call_service_fn or (lambda srv: (True, "noop"))
        self._publish_cmd_vel_fn = publish_cmd_vel_fn or (lambda vx, vy, vyaw: None)
        self._publish_neupan_stop_fn = publish_neupan_stop_fn or (lambda: None)
        self._publish_estop_fn = publish_estop_fn or (lambda flag: None)
        self.cmd_timeout_sec = float(cmd_timeout_sec)
        self.max_linear_vel = float(max_linear_vel)
        self.max_angular_vel = float(max_angular_vel)

        self._lock = threading.Lock()
        self.estop_locked = False
        self.teleop_active = False
        self.target_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_vel_at: float = 0.0
        self.last_action_message: str = "就绪"
        self.last_action_at: Optional[float] = None

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        try:
            v = float(val)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(v):
            return 0.0
        return max(-abs(limit), min(abs(limit), v))

    def execute_nav_command(self, action: str) -> dict:
        act = (action or "").strip().lower()
        now = time.time()

        if act == "emergency_stop":
            with self._lock:
                self.estop_locked = True
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_estop_fn(True)
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            ok, msg = self._call_service_fn("/waypoint_manager/pause")
            with self._lock:
                self.last_action_message = f"🚨 已触发紧急停止（持续锁定零速；任务暂停: {msg}）"
            return {"ok": True, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "release_emergency_stop":
            with self._lock:
                self.estop_locked = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
                self.last_action_message = "🔓 已解除急停锁定（可点击继续导航或接管手控）"
            self._publish_estop_fn(False)
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            return {"ok": True, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "pause":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            ok, msg = self._call_service_fn("/waypoint_manager/pause")
            with self._lock:
                self.last_action_message = f"⏸️ 暂停导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "resume":
            with self._lock:
                was_estop = self.estop_locked
                self.estop_locked = False
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            if was_estop:
                self._publish_estop_fn(False)
            ok, msg = self._call_service_fn("/waypoint_manager/resume")
            with self._lock:
                self.last_action_message = f"▶️ 继续导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "skip":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            # Arm skip in waypoint_goal_bridge (if running in queue mode), then skip in mission_manager
            self._call_service_fn("/waypoint/goal/skip")
            ok, msg = self._call_service_fn("/waypoint_manager/skip")
            with self._lock:
                self.last_action_message = f"⏭️ 跳过当前目标点：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        if act == "cancel":
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            self._call_service_fn("/waypoint/goal/reset")
            ok, msg = self._call_service_fn("/waypoint_manager/cancel")
            with self._lock:
                self.last_action_message = f"⏹️ 取消导航：{msg}"
            return {"ok": ok, "action": act, "message": self.last_action_message, "control": self.get_status()}

        raise ValueError(f"未知的导航控制指令: {action}")

    def set_teleop_takeover(
        self,
        active: bool,
        resume_navigation: bool = False,
        now: Optional[float] = None,
    ) -> dict:
        now = time.time() if now is None else float(now)
        if active:
            with self._lock:
                was_estop = self.estop_locked
                self.estop_locked = False
                self.teleop_active = True
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_vel_at = now
                self.last_action_at = now
            if was_estop:
                self._publish_estop_fn(False)
            self._call_service_fn("/waypoint_manager/pause")
            self._publish_neupan_stop_fn()
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            with self._lock:
                self.last_action_message = "🕹️ 已接管手控（导航自动暂停，松手自动零速保护）"
        else:
            with self._lock:
                self.teleop_active = False
                self.target_vel = (0.0, 0.0, 0.0)
                self.last_action_at = now
            self._publish_cmd_vel_fn(0.0, 0.0, 0.0)
            if resume_navigation:
                ok, msg = self._call_service_fn("/waypoint_manager/resume")
                with self._lock:
                    self.last_action_message = f"🔄 已释放手控并继续导航（{msg}）"
            else:
                with self._lock:
                    self.last_action_message = "⏸️ 已退出手控模式（导航保持暂停）"
        status = self.get_status()
        return {
            "ok": True,
            "teleop_active": status["teleop_active"],
            "message": status["last_action_message"],
            "control": status,
        }

    def set_teleop_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        now: Optional[float] = None,
    ) -> dict:
        now = time.time() if now is None else float(now)
        with self._lock:
            if self.estop_locked:
                raise RuntimeError("当前处于急停锁定状态，请先解除急停")
            if not self.teleop_active:
                raise RuntimeError("请先点击「接管控制权」进入手控模式")
            cx = self._clamp(vx, self.max_linear_vel)
            cy = self._clamp(vy, self.max_linear_vel)
            cw = self._clamp(vyaw, self.max_angular_vel)
            self.target_vel = (cx, cy, cw)
            self.last_vel_at = now
        self._publish_cmd_vel_fn(cx, cy, cw)
        return {
            "ok": True,
            "vx": cx,
            "vy": cy,
            "vyaw": cw,
        }

    def tick(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        pub_vel: Optional[Tuple[float, float, float]] = None
        with self._lock:
            if self.estop_locked:
                pub_vel = (0.0, 0.0, 0.0)
            elif self.teleop_active:
                if (now - self.last_vel_at) > self.cmd_timeout_sec:
                    self.target_vel = (0.0, 0.0, 0.0)
                pub_vel = self.target_vel
        if pub_vel is not None:
            self._publish_cmd_vel_fn(pub_vel[0], pub_vel[1], pub_vel[2])

    def get_status(self) -> dict:
        with self._lock:
            return {
                "estop_locked": self.estop_locked,
                "teleop_active": self.teleop_active,
                "target_vel": {
                    "vx": round(self.target_vel[0], 3),
                    "vy": round(self.target_vel[1], 3),
                    "vyaw": round(self.target_vel[2], 3),
                },
                "max_linear_vel": self.max_linear_vel,
                "max_angular_vel": self.max_angular_vel,
                "last_action_message": self.last_action_message,
                "last_action_at": self.last_action_at,
            }


