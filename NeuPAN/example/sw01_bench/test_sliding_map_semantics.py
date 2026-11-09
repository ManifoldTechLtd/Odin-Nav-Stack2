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

"""Unit tests for SlidingMapSim raytracing semantics:
  - T1: in-FOV clearing when a dynamic obstacle moves away
  - T2: out-of-FOV retention (decay=0) and expiration (decay=3)
  - T3: world-to-grid coordinate consistency with negative world offsets
  - T4: max-range no-return rays do not mark occupied cells
"""
import math
import sys

import numpy as np
import yaml

sys.path.insert(0, ".")
from sim_system import WorldGrid, SlidingMapSim

FAIL = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}")
    if not cond:
        FAIL.append(name)


def make_world(offset=(0.0, 0.0), size=(20, 20)):
    cfg = {"world": {"width": size[0], "height": size[1], "step_time": 0.1,
                     "sample_time": 0.1, "offset": list(offset),
                     "collision_mode": "stop", "control_mode": "auto"},
           "robot": [], "obstacle": []}
    return WorldGrid(cfg, res=0.05)


def lidar_at(robot, yaw, obs_list, n=200, fov=2.094, rmax=8.0):
    """Ideal 120-degree lidar returning (angles, ranges) for circular obstacles `obs_list=[(x, y, r), ...]`."""
    ang = np.linspace(-fov / 2, fov / 2, n)
    rng = np.full(n, rmax)
    for k, a in enumerate(ang):
        best = rmax
        dx, dy = math.cos(yaw + a), math.sin(yaw + a)
        for (ox, oy, orr) in obs_list:
            ex, ey = ox - robot[0], oy - robot[1]
            b = ex * dx + ey * dy
            c = ex * ex + ey * ey - orr * orr
            disc = b * b - c
            if b > 0 and disc >= 0:
                t = b - math.sqrt(disc)
                if 0.05 < t < best:
                    best = t
        rng[k] = best
    return ang, rng


print("T1 In-FOV real-time clearing (decay=0)")
w = make_world()
f0 = SlidingMapSim(w, decay_sec=0.0)
robot = (5.0, 5.0)
yaw = 0.0
obs = [(8.0, 5.0, 0.3)]
for step in range(10):
    a, r = lidar_at(robot, yaw, obs)
    f0.raytrace_update(step * 0.1, robot, yaw, a, r)
ix, iy = w.to_index(7.7, 5.0)
check("T1.1 surface cell marked when observed", f0.mem[ix, iy] and f0.plan_grid()[ix, iy])
obs = [(8.0, 8.0, 0.3)]
for step in range(10, 20):
    a, r = lidar_at(robot, yaw, obs)
    f0.raytrace_update(step * 0.1, robot, yaw, a, r)
still = f0.mem[ix, iy]
n_cleared = int((~f0.mem[max(0,ix-6):ix+7, max(0,iy-6):iy+7]).sum())
check("T1.2 vacated cell cleared in real time", not still,
      f"cell={still} cleared_neighbors={n_cleared}")

print("T2 Out-of-FOV retention and decay")
for decay, expect_keep in ((0.0, True), (3.0, False)):
    w2 = make_world()
    f2 = SlidingMapSim(w2, decay_sec=decay)
    obs2 = [(8.0, 5.0, 0.3)]
    for step in range(10):
        a, r = lidar_at(robot, 0.0, obs2)
        f2.raytrace_update(step * 0.1, robot, 0.0, a, r)
    ix2, iy2 = w2.to_index(7.7, 5.0)
    marked = f2.mem[ix2, iy2]
    for step in range(10, 60):
        a, r = lidar_at(robot, math.pi, obs2)
        f2.raytrace_update(step * 0.1, robot, math.pi, a, r)
        f2.decay(step * 0.1)
    keep = f2.mem[ix2, iy2]
    check(f"T2 decay={decay}: marked={marked} kept_after_5s={keep} (expected {expect_keep})",
          keep == expect_keep)

print("T3 Coordinate consistency with negative world offset")
w3 = make_world(offset=(-23.04, -10.03))
f3 = SlidingMapSim(w3, decay_sec=0.0)
obs3 = [(-20.0, -1.0, 0.25)]
rb3 = (-17.0, -1.0)
for step in range(10):
    a, r = lidar_at(rb3, math.pi, obs3)
    f3.raytrace_update(step * 0.1, rb3, math.pi, a, r)
ix3, iy3 = w3.to_index(-19.75, -1.0)
wx3, wy3 = w3.to_world(ix3, iy3)
hit = f3.plan_grid()[ix3, iy3]
near = int(f3.mem[max(0,ix3-4):ix3+5, max(0,iy3-4):iy3+5].sum())
check("T3.1 obstacle cell hit in negative-offset world", hit, f"cell=({ix3},{iy3}) world=({wx3:.2f},{wy3:.2f})")
check("T3.2 round-trip world<->index consistency",
      abs(wx3 - (-19.75)) < 0.051 and abs(wy3 - (-1.0)) < 0.051)
check("T3.3 hit cluster localized around obstacle (<25 cells)", 0 < near < 25, f"cluster={near}")

print("T4 Max-range no-return rays do not mark occupied cells")
w4 = make_world()
f4 = SlidingMapSim(w4, decay_sec=3.0)
a4 = np.linspace(-1.5708 / 2, 1.5708 / 2, 93)
r4 = np.full(93, 4.5)
f4.raytrace_update(0.0, robot, 0.0, a4, r4, footprint_radius=0.60, range_max=4.5)
check("T4 no-return rays leave memory empty", not f4.mem.any(), f"occupied={int(f4.mem.sum())}")

print()
if FAIL:
    print(f"FAILED: {len(FAIL)} checks -> {FAIL}")
    sys.exit(1)
print("ALL PASS")
