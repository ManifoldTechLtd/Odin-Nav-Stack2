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

"""Deterministic planning test for sliding_map memory decay vs. A* path planning across an occluded obstacle."""
import math
import sys

import numpy as np

sys.path.insert(0, ".")
from sim_system import WorldGrid, SlidingMapSim, GlobalPathSim
import yaml

env = yaml.safe_load(open("ghost_corner/env.yaml"))
OBS = (6.6, 4.5, 0.32)     # Unknown obstacle (center_x, center_y, radius)
ROBOT = (6.6, 1.0)         # Corner pose


def lidar(robot, yaw, obs, n=200, fov=2.094, rmax=8.0):
    ang = np.linspace(-fov / 2, fov / 2, n)
    rng = np.full(n, rmax)
    for k, a in enumerate(ang):
        dx, dy = math.cos(yaw + a), math.sin(yaw + a)
        ex, ey = obs[0] - robot[0], obs[1] - robot[1]
        b = ex * dx + ey * dy
        c = ex * ex + ey * ey - obs[2] ** 2
        disc = b * b - c
        if b > 0 and disc >= 0:
            t = b - math.sqrt(disc)
            if 0.05 < t < rng[k]:
                rng[k] = t
    return ang, rng


def path_crosses_obs(wps, obs, thresh=None):
    """Return (crosses_obstacle, min_distance_to_center)."""
    thr = thresh if thresh is not None else obs[2] + 0.35
    arr = np.array([[a, b] for a, b, _ in wps])
    d = np.hypot(arr[:, 0] - obs[0], arr[:, 1] - obs[1])
    return bool((d < thr).any()), float(d.min())


print(f"{'decay':>6} | seen_1s | kept_5s | replan_res | crosses_obs | min_dist")
for decay in (3.0, 0.0):
    w = WorldGrid(env)
    f = SlidingMapSim(w, decay_sec=decay)
    g = GlobalPathSim(w, f, inflation_radius=0.35)
    # Phase 1: face north (+90 deg) and observe obstacle for 1s (10 frames)
    yaw_n = math.pi / 2
    for i in range(10):
        a, r = lidar(ROBOT, yaw_n, OBS)
        f.raytrace_update(i * 0.1, ROBOT, yaw_n, a, r)
        f.decay(i * 0.1)
    seen = int(f.mem.sum())
    # Phase 2: face south (-90 deg) for 5s (50 frames)
    yaw_s = -math.pi / 2
    for i in range(10, 60):
        a, r = lidar(ROBOT, yaw_s, OBS)
        f.raytrace_update(i * 0.1, ROBOT, yaw_s, a, r)
        f.decay(i * 0.1)
    kept = int(f.mem.sum())
    # Phase 3: replan
    wps = g.plan(ROBOT, (6.6, 10.4))
    if wps:
        ghost, dmin = path_crosses_obs(wps, OBS)
        res = f"{len(wps)}wps"
    else:
        ghost, dmin, res = False, float("nan"), "NO_PATH"
    print(f"{decay:>6} | {seen:>7} | {kept:>7} | {res:>10} | {str(ghost):>11} | "
          f"{dmin:.2f}m" if wps else
          f"{decay:>6} | {seen:>7} | {kept:>7} | {res:>10} | {str(ghost):>11} |   -")
