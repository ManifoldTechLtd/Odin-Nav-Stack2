"""Record ROS 2 bags on the SW01 host in a dedicated tmux session that survives console restarts."""

import json
import os
import re
import shlex
import shutil
import subprocess
import threading
import time
from typing import Callable, Dict, List, Optional

SESSION = "ons2_bag"
TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")
HEAVY_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
LIGHT_TOPICS = [
    "/rosout_agg", "/rosout",
    "/cmd_vel", "/odin1/odometry", "/tf", "/tf_static",
    "/scan_raw", "/scan", "/scan_360", "/scan_temporal_filter/trace", "/initial_path", "/inflated_map",
    "/move_base_simple/goal", "/neupan/goal", "/neupan/arrive", "/neupan/replan",
    "/neupan_plan", "/neupan_initial_path",
    "/neupan/control_trace", "/neupan/escape_trace",
    "/neupan/recovery_request", "/neupan/recovery_command",
    "/neupan/replan_gate_shadow",
    "/sw01_cmd_vel_bridge/cmd_sent", "/sw01_cmd_vel_bridge/control_trace",
    "/sw01_cmd_vel_bridge/sdk_linked", "/sw01_cmd_vel_bridge/control_authority",
    "/waypoint_manager/status", "/waypoint_manager/mission_feedback",
]
TOTAL_TOPICS = [
    "/waypoint_manager/status", "/waypoint_manager/mission_feedback",
    "/move_base_simple/goal", "/cmd_vel", "/cmd_vel/throttle",
    "/odin1/odometry", "/tf", "/tf_static", "/scan_raw", "/scan", "/scan_360", "/scan_temporal_filter/trace",
    "/odin1/cloud_slam", "/odin1/image/compressed", "/odin1/cloud_raw", "/imu", "/odin1/imu",
    "/initial_path", "/neupan_initial_path", "/neupan_plan", "/robot_marker",
    "/neupan/replan", "/neupan/arrive", "/neupan/stop",
    "/neupan/nav_path/goal", "/neupan/nav_path/status", "/neupan/nav_path/feedback",
    "/neupan/nav_path/result", "/neupan/nav_path/cancel",
    "/map", "/inflated_map", "/fake_map", "/sliding_map", "/map_planner/result",
    "/sw01_cmd_vel_bridge/sdk_linked", "/sw01_cmd_vel_bridge/control_authority",
    "/neupan/control_trace", "/neupan/escape_trace",
    "/neupan/recovery_request", "/neupan/recovery_command",
    "/neupan/replan_gate_shadow",
    "/sw01_cmd_vel_bridge/cmd_sent", "/sw01_cmd_vel_bridge/control_trace",
    "/rosout_agg", "/rosout",
]
PRESETS = {
    "light": {"label": "轻量 (LIGHT)", "topics": LIGHT_TOPICS},
    "total": {"label": "全量 (TOTAL)", "topics": TOTAL_TOPICS},
    "nav": {"label": "导航核心", "topics": [
        "/tf", "/tf_static", "/odin1/odometry", "/cmd_vel", "/map", "/scan",
        "/waypoint_manager/status", "/waypoint_manager/system_health", "/waypoint_manager/mission_feedback",
        "/waypoint/goal"]},
    "perception": {"label": "感知", "topics": ["/odin1/imu", "/odin1/image/compressed", "/odin1/cloud_filtered"]},
    "debug": {"label": "调试", "topics": ["/rosout", "/navigation/map_diagnostics", "/emergency_stop", "/neupan/stop"]},
}
ENV_DIR, ENV_TOPICS, ENV_STARTED = "BAG_DIR", "BAG_TOPICS", "BAG_STARTED"


def normalize_topics(raw) -> List[str]:
    parts = re.split(r"[\s,]+", raw.strip()) if isinstance(raw, str) else [
        p for item in (raw or []) for p in re.split(r"[\s,]+", str(item).strip())]
    names = []
    for part in parts:
        if not part:
            continue
        if not TOPIC_RE.match(part):
            raise ValueError("话题名无效：{}".format(part))
        if part not in names:
            names.append(part)
    if not names:
        raise ValueError("请至少选择一个话题")
    return names


def heavy_topics(topic_types: Dict[str, List[str]], names: List[str]) -> List[str]:
    return [n for n in names if any(t in HEAVY_TYPES for t in topic_types.get(n, []))]


def bag_dir_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]", "_", name or "").strip("_")
    return cleaned[:40] or "bag"


def build_record_cmd(repo_root: str, out_dir: str, topics: List[str], domain: str = "98") -> str:
    return "source /opt/ros/jazzy/setup.bash && source {} && export ROS_DOMAIN_ID={} && ros2 bag record -o {} {}".format(
        shlex.quote(os.path.join(repo_root, "ros2_ws", "install", "setup.bash")), shlex.quote(str(domain)),
        shlex.quote(out_dir), " ".join(shlex.quote(t) for t in topics))


def _default_run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=20)


def _dir_size(path: str) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


class BagRecorder:
    def __init__(self, bag_root: str, repo_root: Optional[str] = None, run: Optional[Callable] = None,
                 has_tmux: Optional[Callable[[], bool]] = None, domain: Optional[str] = None):
        self.bag_root = bag_root
        self.domain = str(domain or os.environ.get("ROS_DOMAIN_ID", "98"))
        self.repo_root = repo_root or os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self._run = run or _default_run
        self._has_tmux = has_tmux or (lambda: shutil.which("tmux") is not None)
        self._lock = threading.Lock()

    def _require_tmux(self):
        if not self._has_tmux():
            raise ValueError("未安装 tmux，无法录包")

    def _tmux(self, *args):
        return self._run(["tmux", *args])

    def _active(self) -> bool:
        try:
            return self._tmux("has-session", "-t", SESSION).returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    def _env(self, key: str) -> Optional[str]:
        res = self._tmux("show-environment", "-t", SESSION, key)
        if res.returncode != 0 or "=" not in res.stdout:
            return None
        return res.stdout.strip().split("=", 1)[1]

    def start(self, topics, name: str = "", topic_types: Optional[Dict[str, List[str]]] = None) -> dict:
        self._require_tmux()
        names = normalize_topics(topics)
        with self._lock:
            if self._active():
                raise ValueError("已有录包进行中，请先结束")
            out_dir = os.path.join(self.bag_root, "{}_{}".format(bag_dir_name(name), time.strftime("%Y%m%d_%H%M%S")))
            os.makedirs(self.bag_root, exist_ok=True)
            started = time.time()
            meta_path = out_dir + "_meta.json"
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump({"name": name, "topics": names, "dir": out_dir, "started_at": started}, f,
                          ensure_ascii=False, indent=2)
            self._tmux("new-session", "-d", "-s", SESSION, "bash --norc")
            self._tmux("set-environment", "-t", SESSION, ENV_DIR, out_dir)
            self._tmux("set-environment", "-t", SESSION, ENV_TOPICS, " ".join(names))
            self._tmux("set-environment", "-t", SESSION, ENV_STARTED, str(started))
            self._tmux("send-keys", "-t", SESSION, "-l", build_record_cmd(self.repo_root, out_dir, names, self.domain))
            self._tmux("send-keys", "-t", SESSION, "Enter")
        warnings = heavy_topics(topic_types or {}, names)
        return {"ok": True, "message": "开始录包：{} 个话题".format(len(names)), "dir": out_dir,
                "topics": names, "warnings": warnings}

    def stop(self) -> dict:
        self._require_tmux()
        with self._lock:
            if not self._active():
                raise ValueError("当前没有录包")
            out_dir = self._env(ENV_DIR)
            self._tmux("send-keys", "-t", SESSION, "C-c")
            self._wait_bag_exit(out_dir)
            self._tmux("kill-session", "-t", SESSION)
        size = _dir_size(out_dir) if out_dir and os.path.isdir(out_dir) else 0
        return {"ok": True, "message": "录包已结束", "dir": out_dir, "size_bytes": size}

    def _wait_bag_exit(self, out_dir, timeout=15.0):
        if not out_dir:
            return
        end = time.time() + timeout
        while time.time() < end:
            try:
                alive = self._run(["pgrep", "-f", "ros2 bag record -o {}".format(out_dir)]).returncode == 0
            except (OSError, subprocess.SubprocessError):
                alive = False
            if not alive:
                return
            time.sleep(0.5)

    def status(self) -> dict:
        active = self._active()
        out_dir = self._env(ENV_DIR) if active else None
        topics = (self._env(ENV_TOPICS) or "").split() if active else []
        started = float(self._env(ENV_STARTED) or 0) if active else 0.0
        return {"active": active, "tmux": self._has_tmux(), "dir": out_dir, "topics": topics,
                "started_at": started or None,
                "elapsed_sec": round(time.time() - started, 1) if active and started else 0.0,
                "size_bytes": _dir_size(out_dir) if out_dir and os.path.isdir(out_dir) else 0,
                "free_bytes": self._free_bytes()}

    def _free_bytes(self) -> int:
        path = self.bag_root
        while not os.path.isdir(path) and os.path.dirname(path) != path:
            path = os.path.dirname(path)
        return shutil.disk_usage(path).free
