#!/usr/bin/env python3
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

"""Diagnostic event code (EC) logger for NeuPAN critical paths (ROS-independent, flush + fsync per entry)."""
import os
import threading
import time

_LOG_DIR = os.path.expanduser(os.environ.get("EC_LOG_DIR", "~/base_inspection/logs"))
_LOG_FILE = os.path.join(_LOG_DIR, "ec_events.log")
_lock = threading.Lock()


class EC:
    """Diagnostic code table (kept in sync with C++ ec_trace.h)."""
    # 0xx lifecycle
    NODE_START = 1
    NODE_READY = 2
    NODE_EXIT = 3
    NODE_START_FAIL = 10
    # 1xxx mode transitions and gate telemetry
    MODE_ENTER = 1000
    MODE_MPC = 1001
    MODE_ESCAPE = 1002
    MODE_PREALIGN = 1003
    WAIT_INITIAL_PATH = 1004
    PATH_INSTALL = 1005
    PATH_REJECT = 1006
    ADAPTIVE_DMAX = 1007
    GRACE_TRIGGER = 1008
    ESCAPE_WATCH = 1009
    MPC_SOLVE = 1010
    REPLAN_TRIGGER = 1011
    PLAN_DONE = 1012
    # 2xxx performance / resources
    SOLVE_SLOW = 2000
    LOOP_GAP = 2001
    CPU_HIGH = 2002
    DISK_LOW = 2003
    HEAVY_ALLOC_PRE = 2004
    # 3xxx NeuPAN
    NEUPAN_CRASH_PRE = 3000
    NEUPAN_SOLVE_FAIL = 3001
    NEUPAN_OBSTACLE_BURST = 3002
    NEUPAN_LOOP_ALIVE = 3003
    NEUPAN_FEEDBACK_FROZEN = 3004
    # 4xxx map planner
    ASTAR_FAIL = 4000
    MAP_LOAD_FAIL = 4001
    ASTAR_SLOW = 4002
    PATH_INVALID = 4003
    # 5xxx robot bridge
    SDK_CONNECT_FAIL = 5000
    CMDVEL_STALL = 5001
    CMDVEL_ABNORMAL = 5002
    # 6xxx perception / drivers
    DEVICE_OFFLINE = 6000
    CLOUD_STALL = 6001
    TF_FAIL = 6002
    # 7xxx environment / dependencies
    PY_VER_MISMATCH = 7000
    ABI_RISK = 7001
    SDK_LIB_MISSING = 7002


def ec(code: int, phase: str, detail: str = "", node: str = "neupan", also_print: bool = True) -> str:
    """Write a diagnostic event entry to the local event log (flush + fsync) and optionally stdout."""
    line = "[EC:{:05d}] 节点={} 线程={} 阶段={} 详情={}".format(
        code, node, threading.current_thread().name, phase, detail
    )
    try:
        with _lock:
            if not os.path.isdir(_LOG_DIR):
                os.makedirs(_LOG_DIR, exist_ok=True)
            with open(_LOG_FILE, "a", encoding="utf-8") as fp:
                fp.write("{:.3f} {}\n".format(time.time(), line))
                fp.flush()
                os.fsync(fp.fileno())
    except Exception:
        pass
    if also_print:
        try:
            print(line, flush=True)
        except Exception:
            pass
    return line


def tail(n: int = 20) -> list:
    """Read the last `n` entries from the diagnostic event log."""
    try:
        with open(_LOG_FILE, "r", encoding="utf-8") as fp:
            return [ln.rstrip("\n") for ln in fp.readlines()[-n:]]
    except Exception:
        return []
