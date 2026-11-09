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

"""Unit test for GlobalMemorySim persistent coarse memory when an obstacle slides out of the 20m sliding_map window."""
import sys

import numpy as np

sys.path.insert(0, ".")
from sim_system import WorldGrid, SlidingMapSim, GlobalPathSim, GlobalMemorySim

ENV = {
    "world": {"width": 40, "height": 10, "offset": [0, -5]},
    "robot": [{"shape": {"name": "rectangle", "length": 0.96, "width": 0.55},
               "state": [1, 0, 0], "goal": [38, 0, 0]}],
    "obstacle": [{"number": 1, "distribution": {"name": "manual"},
                  "exclude_from_static": True,
                  "shape": [{"name": "circle", "radius": 0.3}],
                  "state": [[5.0, 0.0, 0]]}],
}
OBS = (5.0, 0.0)
ANG = np.linspace(-1.047, 1.047, 60)


def drive(f360, with_obstacle_view=True):
    """Drive east from x=1 to x=30, observing the obstacle at x=5 when passing x=3."""
    for x in np.arange(1.0, 30.1, 2.0):
        rng = np.full(len(ANG), 8.0)
        if with_obstacle_view and abs(x - 3.0) < 1e-6:
            rng = np.where(np.abs(ANG) < 0.15, 2.0, 8.0)
        f360.raytrace_update(x * 0.5, (x, 0.0), 0.0, ANG, rng)
        f360.decay(x * 0.5)
        f360.apply_window((x, 0.0))


def path_clearance(gp, start, goal):
    wps = gp.plan(start, goal)
    if wps is None:
        return None, None
    arr = np.array([[a, b] for a, b, _ in wps])
    d = np.hypot(arr[:, 0] - OBS[0], arr[:, 1] - OBS[1]).min()
    return d, len(wps)


def main():
    fails = 0

    # A) Sliding window without coarse global memory: obstacle is forgotten once outside the 20m window
    w = WorldGrid(ENV, res=0.05)
    f = SlidingMapSim(w, decay_sec=3.0, window_m=20.0)
    drive(f)
    gp = GlobalPathSim(w, f, inflation_radius=0.35)
    d, n = path_clearance(gp, (30.0, 0.0), (2.0, 0.0))
    print(f"A sliding window only: return path clearance to obstacle = {d:.2f}m ({n} pts)")
    if d is not None and d > 0.45:
        print("  WARN: obstacle remained in memory"); fails += 1
    else:
        print("  OK: sliding window alone forgets obstacle outside 20m window")

    # B) Sliding window + 0.2m coarse global memory: obstacle is retained and bypassed
    w2 = WorldGrid(ENV, res=0.05)
    gm = GlobalMemorySim(w2, res=0.20)
    f2 = SlidingMapSim(w2, decay_sec=3.0, window_m=20.0, global_memory=gm)
    drive(f2)
    gp2 = GlobalPathSim(w2, f2, inflation_radius=0.35)
    d2, n2 = path_clearance(gp2, (30.0, 0.0), (2.0, 0.0))
    print(f"B with 0.2m coarse global memory: return path clearance = {d2:.2f}m ({n2} pts)")
    assert d2 is not None and d2 > 0.45, "Coarse global memory failed to bypass obstacle"
    print("  OK: persistent coarse memory bypasses previously observed obstacle")

    # C) Start/goal exemption prevents GOAL_OCCUPIED when goal lies on coarse memory
    gm.stamp(2.0, 0.0); gm.stamp(2.1, 0.1)
    d3, n3 = path_clearance(gp2, (30.0, 0.0), (2.0, 0.0))
    assert d3 is not None and n3 > 2, "Goal exemption failed (GOAL_OCCUPIED)"
    print(f"C goal covered by coarse memory: still plans {n3} pts OK")

    # D) Grid cell count comparison on a 100x100m map
    w100 = {"world": {"width": 100, "height": 100, "offset": [0, 0]}, "obstacle": []}
    wg = WorldGrid(w100, res=0.05)
    fine_cells = wg.nx * wg.ny
    g100 = GlobalMemorySim(wg, res=0.20)
    coarse_cells = g100.nx * g100.ny
    print(f"D 100x100m map: fine {fine_cells/1e6:.1f}M cells vs coarse {coarse_cells/1e6:.2f}M cells"
          f" ({fine_cells/coarse_cells:.0f}x reduction) OK")

    if fails:
        sys.exit(1)
    print("\ntest_mem_slip: ALL PASS")


if __name__ == "__main__":
    main()
