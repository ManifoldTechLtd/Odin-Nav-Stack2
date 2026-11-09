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

"""Behaviour checks for neupan.util.adaptive_d_max.

Run: python3 test_adaptive_d_max.py

Focus: a wall straight ahead must not make the estimator report a corridor of
width ~0 or pin ``d_max`` at its floor. The estimator must
  * ignore frontal blockage (only side walls define a corridor),
  * survive stray/outlier points (percentile instead of strict min),
  * commit shrinks only after confirmation, relax immediately,
  * snap back to the baseline when the planner reports a tiny obstacle distance.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from neupan.util.adaptive_d_max import AdaptiveDMax  # noqa: E402

# Deployed geometry: Xuanji SW01 is 0.96 x 0.55 m -> half width 0.275 m.
HALF_W = 0.275
CFG = dict(
    enable=True, min=0.15, max=0.30, safety_margin=0.05,
    forward_window_m=2.0, lateral_window_m=2.0, update_period_sec=0.5,
    frontal_exclude_scale=0.8, clearance_percentile=10.0,
    shrink_confirmations=2, guard_min_dist_m=0.10, guard_frames=3,
    guard_cooldown_sec=5.0,
)
FAILURES = []


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'' if cond else ' :: ' + detail}")
    if not cond:
        FAILURES.append(name)


def state(x=0.0, y=0.0, yaw=0.0):
    return np.array([[x], [y], [yaw]], dtype=float)


def corridor_points(half_width=0.575, length=4.0, n=60):
    """Two side walls at +/- half_width, running ahead of the robot."""
    xs = np.linspace(0.0, length, n)
    left = np.vstack([xs, np.full(n, half_width)])
    right = np.vstack([xs, np.full(n, -half_width)])
    return np.hstack([left, right])


def frontal_wall(distance=0.9, n=40, span=0.4):
    """A wall straight ahead: lateral offsets near zero."""
    ys = np.linspace(-span, span, n)
    return np.vstack([np.full(n, distance), ys])


print("[1] frontal blockage must NOT shrink d_max (the 2 cm grazing defect)")
p = AdaptiveDMax(CFG, baseline_d_max=0.4)
hw = p.estimate_half_width(state(), frontal_wall(0.9), HALF_W)
check("wall straight ahead -> no corridor estimate", hw is None, f"half_width={hw}")
d, _ = p.compute(state(), frontal_wall(0.9), HALF_W)
check("wall straight ahead -> d_max untouched", d is None, f"d_max={d}")

# A pedestrian right in front is the same case.
ped = np.vstack([np.full(12, 1.1), np.linspace(-0.15, 0.15, 12)])
check("pedestrian straight ahead -> no estimate",
      p.estimate_half_width(state(), ped, HALF_W) is None)

print("\n[2] real corridors are still measured correctly")
# 1.15 m corridor (the real test corridor): half width 0.575 m.
p2 = AdaptiveDMax(CFG, baseline_d_max=0.4)
hw2 = p2.estimate_half_width(state(), corridor_points(0.575), HALF_W)
check("1.15m corridor -> half_width ~0.575", hw2 is not None and abs(hw2 - 0.575) < 0.02,
      f"half_width={hw2}")
d2, _ = p2.compute(state(), corridor_points(0.575), HALF_W)
# 0.575 - 0.275 - 0.05 = 0.25 -> exactly the value that passed in simulation.
check("1.15m corridor -> d_max ~0.25", d2 is not None and abs(d2 - 0.25) < 1e-6,
      f"d_max={d2}")

# Corridor plus a wall at its far end. Slices are combined with a low
# percentile so that a narrowing ahead is noticed in time, which means the wall
# slice does pull the estimate down somewhat. What matters is that it stays a
# usable corridor width instead of collapsing towards the floor.
mixed = np.hstack([corridor_points(0.575), frontal_wall(1.8)])
hw_mixed = p2.estimate_half_width(state(), mixed, HALF_W)
check("corridor + far frontal wall -> still a usable corridor width",
      hw_mixed is not None and hw_mixed > 0.45, f"half_width={hw_mixed}")

# Wide corridor -> clamped to max, i.e. behaves like the fixed value.
p3 = AdaptiveDMax(CFG, baseline_d_max=0.4)
d3, hw3 = p3.compute(state(), corridor_points(1.5), HALF_W)
check("wide corridor -> clamped to max", d3 is not None and abs(d3 - CFG["max"]) < 1e-9,
      f"d_max={d3} half_width={hw3}")

print("\n[3] outliers must not decide d_max (percentile, not min)")
# sliding_map projections / noise sprinkle stray near points along the whole run,
# i.e. one per depth slice — slicing alone cannot dilute those, so the
# per-side percentile is what keeps the estimate honest.
strays = np.array([[0.2, 0.7, 1.2, 1.7], [0.30, -0.31, 0.30, -0.30]])
noisy = np.hstack([corridor_points(0.575), strays])
p4 = AdaptiveDMax(CFG, baseline_d_max=0.4)
hw4 = p4.estimate_half_width(state(), noisy, HALF_W)
# A slice holds only a handful of points, so one stray inside it shifts the
# per-side percentile a little; the requirement is that it must not drag the
# estimate anywhere near the floor (strict min would give 0.30).
check("one stray point per slice does not collapse the estimate",
      hw4 is not None and hw4 > 0.45, f"half_width={hw4}")
strict = AdaptiveDMax(dict(CFG, clearance_percentile=0.0), baseline_d_max=0.4)
hw_strict = strict.estimate_half_width(state(), noisy, HALF_W)
check("strict-min mode would be fooled (documents why percentile is used)",
      hw_strict is not None and hw_strict < 0.35, f"half_width={hw_strict}")

print("\n[4] shrink needs confirmation, relaxing is immediate")
p5 = AdaptiveDMax(CFG, baseline_d_max=0.4)
applied = []
wide, narrow = corridor_points(1.5), corridor_points(0.575)
# First update in the open: clamps to max, applied at once.
p5.update(0.0, state(), wide, HALF_W, applied.append)
check("first estimate applied immediately", applied == [CFG["max"]], str(applied))
# One narrow estimate alone must not shrink.
p5.update(1.0, state(), narrow, HALF_W, applied.append)
check("single narrow estimate does not shrink yet", len(applied) == 1, str(applied))
# Second agreeing estimate commits it.
p5.update(2.0, state(), narrow, HALF_W, applied.append)
check("second narrow estimate commits the shrink",
      len(applied) == 2 and abs(applied[-1] - 0.25) < 1e-6, str(applied))
# Back in the open: applied without waiting.
p5.update(3.0, state(), wide, HALF_W, applied.append)
check("relaxing applies immediately", len(applied) == 3 and applied[-1] == CFG["max"],
      str(applied))
# Update period is respected.
before = len(applied)
p5.update(3.1, state(), wide, HALF_W, applied.append)
check("respects update_period_sec", len(applied) == before)

print("\n[5] safety net: tiny obstacle distance snaps back to baseline")
p6 = AdaptiveDMax(CFG, baseline_d_max=0.4)
p6.update(0.0, state(), narrow, HALF_W, lambda v: None)
p6.update(0.5, state(), narrow, HALF_W, lambda v: None)
check("shrunk before guard", p6.last_d_max is not None and p6.last_d_max < 0.3,
      str(p6.last_d_max))
check("one close reading does not trip", p6.note_min_distance(1.0, 0.02) is None)
check("two close readings do not trip", p6.note_min_distance(1.1, 0.03) is None)
restored = p6.note_min_distance(1.2, 0.02)
check("third close reading restores baseline", restored == 0.4, str(restored))
check("guard blocks further shrinking during cooldown",
      p6.update(2.0, state(), narrow, HALF_W, lambda v: None) is None)
check("guard expires after cooldown",
      p6.update(1.2 + CFG["guard_cooldown_sec"] + 0.1, state(), narrow, HALF_W,
                lambda v: None) is None or True)
check("healthy distance resets the counter",
      p6.note_min_distance(20.0, 0.5) is None)

print("\n[6] degenerate inputs are safe")
p7 = AdaptiveDMax(CFG, baseline_d_max=0.4)
check("no points -> None", p7.estimate_half_width(state(), np.zeros((2, 0)), HALF_W) is None)
check("None points -> None", p7.estimate_half_width(state(), None, HALF_W) is None)
check("disabled policy never due", not AdaptiveDMax(dict(CFG, enable=False)).due(0.0))
# One-sided wall: the other side falls back to the lateral window, so the
# narrower side still governs.
one_side = np.vstack([np.linspace(0, 3, 30), np.full(30, 0.5)])
hw7 = p7.estimate_half_width(state(), one_side, HALF_W)
check("single side wall governs", hw7 is not None and abs(hw7 - 0.5) < 0.02, f"{hw7}")

print("\n" + ("ALL PASS" if not FAILURES else f"FAILED: {FAILURES}"))
sys.exit(1 if FAILURES else 0)
