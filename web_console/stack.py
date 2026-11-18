"""Start and stop the SW01 navigation stack by running the existing launcher scripts."""

import os
import shutil
import subprocess
import threading
import time
from collections import deque
from typing import Callable, Optional

MODES = ("replan_periodic", "escape", "neupan_only")
SESSION = "ons2_sw01"
EXCLUDED_MAPS = {"empty"}


def tmux_session_active(name: str) -> bool:
    try:
        return subprocess.run(["tmux", "has-session", "-t", name], capture_output=True, timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


class _Job:
    def __init__(self, kind, map_name=None, mode=None):
        self.kind = kind
        self.map = map_name
        self.mode = mode
        self.lines = deque(maxlen=500)
        self.rc = None
        self.started_at = time.time()
        self.finished_at = None

    @property
    def running(self):
        return self.rc is None

    def summary(self):
        return {"kind": self.kind, "map": self.map, "mode": self.mode, "running": self.running,
                "returncode": self.rc, "started_at": self.started_at, "finished_at": self.finished_at}


class NavStack:
    def __init__(self, repo_root: str, maps_dir: Optional[str] = None, reloc_dir: Optional[str] = None,
                 start_script: Optional[str] = None, stop_script: Optional[str] = None,
                 session_active: Optional[Callable[[], bool]] = None,
                 has_tmux: Optional[Callable[[], bool]] = None):
        self.repo_root = repo_root
        self.maps_dir = maps_dir or os.path.join(repo_root, "ros2_ws", "src", "map_planner", "maps")
        self.reloc_dir = reloc_dir or os.path.join(repo_root, "customer_file")
        self.start_script = start_script or os.path.join(repo_root, "scripts", "ons2_sw01.sh")
        self.stop_script = stop_script or os.path.join(repo_root, "scripts", "ons2_sw01_stop.sh")
        self._session_active = session_active or (lambda: tmux_session_active(SESSION))
        self._has_tmux = has_tmux or (lambda: shutil.which("tmux") is not None)
        self._lock = threading.Lock()
        self._job: Optional[_Job] = None

    def list_maps(self):
        maps = []
        if not os.path.isdir(self.maps_dir):
            return maps
        for fname in sorted(os.listdir(self.maps_dir)):
            name, ext = os.path.splitext(fname)
            if ext != ".yaml" or name in EXCLUDED_MAPS:
                continue
            if not os.path.isfile(os.path.join(self.maps_dir, name + ".pgm")):
                continue
            maps.append({"name": name, "reloc": os.path.isfile(os.path.join(self.reloc_dir, name + ".bin"))})
        return maps

    def start(self, map_name: str, navigation_mode: str) -> dict:
        self._require_tmux()
        if map_name not in {m["name"] for m in self.list_maps()}:
            raise ValueError("未知地图：{}".format(map_name))
        if navigation_mode not in MODES:
            raise ValueError("未知导航模式：{}".format(navigation_mode))
        cmd = ["bash", self.start_script, "--map={}".format(map_name), "--navigation-mode={}".format(navigation_mode)]
        with self._lock:
            self._ensure_idle()
            if self._session_active():
                raise ValueError("导航已在运行（tmux 会话 {} 已存在），请先停止".format(SESSION))
            self._job = self._spawn(_Job("start", map_name, navigation_mode), cmd)
        return {"ok": True, "message": "开始启动导航：{} / {}".format(map_name, navigation_mode)}

    def stop(self) -> dict:
        self._require_tmux()
        with self._lock:
            self._ensure_idle()
            self._job = self._spawn(_Job("stop"), ["bash", self.stop_script])
        return {"ok": True, "message": "开始停止导航"}

    def status(self) -> dict:
        job = self._job
        active = self._session_active()
        return {"state": self._state(job, active), "session_active": active, "tmux": self._has_tmux(),
                "job": job.summary() if job else None,
                "log": list(job.lines)[-200:] if job else []}

    def _require_tmux(self):
        if not self._has_tmux():
            raise ValueError("未安装 tmux，无法管理导航会话")

    def _ensure_idle(self):
        if self._job is not None and self._job.running:
            raise ValueError("已有导航任务进行中，请等待其完成")

    def _state(self, job, active):
        if job is not None and job.running:
            return "starting" if job.kind == "start" else "stopping"
        if job is not None and job.rc != 0:
            return "failed"
        return "running" if active else "stopped"

    def _spawn(self, job, cmd):
        proc = subprocess.Popen(cmd, cwd=self.repo_root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, text=True, encoding="utf-8", errors="replace",
                                bufsize=1, start_new_session=True)

        def reader():
            for line in proc.stdout:
                job.lines.append(line.rstrip("\n"))
            job.rc = proc.wait()
            job.finished_at = time.time()

        threading.Thread(target=reader, daemon=True).start()
        return job
