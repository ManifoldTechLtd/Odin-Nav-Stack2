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

"""Behaviour checks for neupan.util.stuck_escape.

Run: python3 test_stuck_escape.py

1. ``displacement`` mode must reproduce the gates of the deployed
   neupan_ros.py (dual gate, count-2 decay, trigger_frames, direction pick,
   exit rules) — including its two known false-positive/false-negative modes.
2. ``progress`` mode must separate the two cases the displacement gate cannot:
   rocking in place (stuck) vs. slow but advancing (not stuck).
"""
import sys
from math import pi, sin
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from neupan.util.stuck_escape import StuckEscapePolicy  # noqa: E402

# Values deployed on Xuanji SW01.
CFG = dict(
    enable=True, displacement_window_sec=1.0, displacement_threshold_m=0.05,
    progress_threshold_m=0.05, v_eps=0.15, w_eps=0.30, trigger_frames=10,
    front_obs_threshold=1.0, cone_half_width_rad=0.7, back_speed=0.2,
    rotate_speed=0.5, duration_sec=2.0, exit_obs_distance=1.0,
    side_search_radius_m=3.5, limit_cycle_consecutive_n=2,
    limit_cycle_window_sec=20.0, min_rotation_rad=0.35,
    early_replan_enable=True, early_replan_trigger_frames=5,
    early_replan_cooldown_sec=2.0,
)
DT = 0.05  # 20 Hz, the measured real control loop rate
FAILURES = []


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'' if cond else ' :: ' + detail}")
    if not cond:
        FAILURES.append(name)


def state(x=0.0, y=0.0, yaw=0.0):
    return np.array([[x], [y], [yaw]], dtype=float)


def front_wall(distance=0.9, n=40):
    ys = np.linspace(-0.4, 0.4, n)
    return np.vstack([np.full(n, distance), ys])


def straight_path(length=10.0, n=101):
    xs = np.linspace(0.0, length, n)
    return np.vstack([xs, np.zeros(n)])


def feed(policy, poses, cmds, points, path=None, points_seq=None, t0=0.0):
    """Drive the policy over a pose/command sequence; return decisions."""
    if path is not None:
        policy.set_reference_path(path)
    out = []
    for i, ((x, y, yaw), (v, w)) in enumerate(zip(poses, cmds)):
        t = t0 + i * DT
        policy.note_pose(t, x, y)
        if path is not None:
            policy.note_progress(t, x, y)
        policy.note_cmd(v, w)
        pts = points_seq[i] if points_seq is not None else points
        out.append(policy.decide(t, state(x, y, yaw), pts))
    return out


print("[1] displacement mode: reproduces the deployed gates")

# Fully stopped against a 0.9 m wall. cmd_low is true immediately, so the count
# starts on frame 0 and trigger_frames=10 fires on frame 9 (~0.5 s at 20 Hz).
p = StuckEscapePolicy(CFG, criterion="displacement")
decs = feed(p, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, front_wall(0.9))
entered = [i for i, d in enumerate(decs) if d.entered]
check("stopped at close wall enters escape", bool(entered), "never entered")
check("enters on frame 10 (cmd_low counts from frame 0)",
      bool(entered) and entered[0] == 9, f"entered at {entered}")
check("escape cmd = back 0.2 + turn 0.5",
      decs[entered[0]].cmd is not None
      and abs(float(decs[entered[0]].cmd[0, 0]) + 0.2) < 1e-9
      and abs(abs(float(decs[entered[0]].cmd[1, 0])) - 0.5) < 1e-9,
      str(decs[entered[0]].cmd))
check("early replan fires at count>=5 before the full trigger",
      any(d.request_replan for d in decs[:entered[0]]), "no early replan")

p2 = StuckEscapePolicy(CFG, criterion="displacement")
d2 = feed(p2, [(0.0, 0.0, 0.0)] * 60, [(0.0, 0.0)] * 60, np.zeros((2, 0)))
check("no obstacle -> never escapes", not any(d.entered for d in d2))

p3 = StuckEscapePolicy(CFG, criterion="displacement")
d3 = feed(p3, [(0.0, 0.0, 0.0)] * 60, [(0.0, 0.0)] * 60, front_wall(1.5))
check("front gate respects front_obs_threshold=1.0",
      not any(d.entered for d in d3))

# count-2 decay: stalled every frame, but the forward cone alternates
# near/far, so gate 2 opens only half the time -> +1/-2 -> never reaches 10.
p4 = StuckEscapePolicy(CFG, criterion="displacement")
pts_seq = [front_wall(0.9) if i % 2 else front_wall(1.5) for i in range(80)]
d4 = feed(p4, [(0.0, 0.0, 0.0)] * 80, [(0.0, 0.0)] * 80, None, points_seq=pts_seq)
check("count-2 decay prevents intermittent-gate trigger",
      not any(d.entered for d in d4))

# Direction: extra obstacle on the right -> turn left (+1).
p5 = StuckEscapePolicy(CFG, criterion="displacement")
pts = np.hstack([front_wall(0.9),
                 np.vstack([np.full(20, 0.5), np.linspace(-1.2, -0.3, 20)])])
d5 = feed(p5, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, pts)
ent5 = [d for d in d5 if d.entered]
check("turns away from the tighter side",
      bool(ent5) and ent5[0].info["direction"] > 0,
      str(ent5[0].info if ent5 else "no enter"))

# Exit needs BOTH front clearance and min_rotation_rad of yaw change.
p6 = StuckEscapePolicy(CFG, criterion="displacement")
feed(p6, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, front_wall(0.9))
check("still active right after enter", p6.active)
d_norot = p6.decide(0.60, state(0.0, 0.0, 0.0), front_wall(2.0))
check("clear front alone does not exit (rotation required)",
      not d_norot.exited and p6.active)
d_rot = p6.decide(0.65, state(0.0, 0.0, 0.5), front_wall(2.0))
check("clear front + rotation exits", d_rot.exited)
check("exit requests replan and opens grace",
      d_rot.request_replan and d_rot.grace_scales == (0.25, 0.5))
# Time-out exit path.
p6b = StuckEscapePolicy(CFG, criterion="displacement")
feed(p6b, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, front_wall(0.9))
d_to = p6b.decide(0.55 + CFG["duration_sec"] + 0.1, state(), front_wall(0.9))
check("times out after duration_sec even if still blocked",
      d_to.exited and d_to.info["time_up"])

print("\n[2] progress mode: separates rocking from slow advance")

# (a) Rocking in place as seen on the robot: lateral sway of +/-0.15 m, no
#     forward advance. Window displacement swings through 0.0-0.3 m (the bag
#     showed 0.053-0.106), so the displacement gate keeps decaying its count,
#     while arc-length progress stays ~0.
poses_rock, cmds_rock = [], []
for i in range(160):
    t = i * DT
    poses_rock.append((0.0, 0.15 * sin(2 * pi * t / 1.5), 0.0))
    cmds_rock.append((0.02, 0.45))  # |w| > w_eps, so cmd_low stays false
rock_disp = StuckEscapePolicy(CFG, criterion="displacement")
d_rock_disp = feed(rock_disp, poses_rock, cmds_rock, front_wall(0.9),
                   path=straight_path())
rock_prog = StuckEscapePolicy(CFG, criterion="progress")
d_rock_prog = feed(rock_prog, poses_rock, cmds_rock, front_wall(0.9),
                   path=straight_path())
check("rocking: displacement mode never escapes (defect #21)",
      not any(d.entered for d in d_rock_disp),
      f"max count={rock_disp.count}")
check("rocking: progress mode escapes", any(d.entered for d in d_rock_prog))

# (b) Slow but real advance at 0.11 m/s -- the measured speed of the successful
#     0.4-0.5 m corridor run (front_obs ~0.9 m there, so gate 2 is open).
poses_slow, cmds_slow = [], []
for i in range(160):
    poses_slow.append((0.11 * i * DT, 0.0, 0.0))
    cmds_slow.append((0.11, 0.02))
slow_disp = StuckEscapePolicy(CFG, criterion="displacement")
d_slow_disp = feed(slow_disp, poses_slow, cmds_slow, front_wall(0.9),
                   path=straight_path())
slow_prog = StuckEscapePolicy(CFG, criterion="progress")
d_slow_prog = feed(slow_prog, poses_slow, cmds_slow, front_wall(0.9),
                   path=straight_path())
check("slow advance: displacement mode false-positives (v_eps=0.15 > 0.11)",
      any(d.entered for d in d_slow_disp))
check("slow advance: progress mode correctly stays idle",
      not any(d.entered for d in d_slow_prog))

# (c) Aggressive displacement threshold triggers premature escape during slow crawl.
aggressive = dict(CFG, displacement_threshold_m=0.15, front_obs_threshold=1.2,
                  trigger_frames=5)
bad = StuckEscapePolicy(aggressive, criterion="displacement")
d_bad = feed(bad, poses_slow, cmds_slow, front_wall(0.9))
bad_first = next((i for i, d in enumerate(d_bad) if d.entered), None)
check("aggressive gate escapes even sooner",
      bad_first is not None and bad_first <= 9, f"first enter={bad_first}")

print("\n[2d] clearing guard: yielding to a moving obstacle must not escape")
# Robot waits while a pedestrian walks out of the way: no advance at all, but
# the forward clearance grows steadily -> escaping here interrupts a working
# yield-and-pass (simulation: dynamic_human dropped 100% -> 67% without this).
poses_yield = [(0.0, 0.0, 0.0)] * 120
cmds_yield = [(0.0, 0.0)] * 120
pts_opening = [front_wall(0.6 + 0.004 * i) for i in range(120)]  # +0.4 m over 5 s
guarded = StuckEscapePolicy(CFG, criterion="progress")
d_guard = feed(guarded, poses_yield, cmds_yield, None,
               points_seq=pts_opening, path=straight_path())
check("clearing ahead suppresses escape", not any(d.entered for d in d_guard))
# The same standstill in front of a wall that is NOT opening must still escape.
unguarded = StuckEscapePolicy(CFG, criterion="progress")
d_static = feed(unguarded, poses_yield, cmds_yield, front_wall(0.6),
                path=straight_path())
check("static blockage still escapes", any(d.entered for d in d_static))
# Guard can be switched off.
off = StuckEscapePolicy(dict(CFG, clearing_guard_enable=False),
                        criterion="progress")
d_off = feed(off, poses_yield, cmds_yield, None,
             points_seq=pts_opening, path=straight_path())
check("clearing guard is switchable", any(d.entered for d in d_off))

print("\n[2e] corridor guard: slow-but-moving in a corridor must not escape")
# 1.15 m corridor with robot legitimately crawling at low speed.
# Side walls at +/-0.575 m put front_obs at ~0.9 m (cone hits the side walls)
# and speed at 0.08 m/s (< v_eps), so both escape gates are open -- the corridor
# guard is what prevents the 12 spurious escapes seen on the robot.
def corridor_points(half_width=0.575, length=3.0, n=60):
    xs = np.linspace(0.0, length, n)
    return np.hstack([np.vstack([xs, np.full(n, half_width)]),
                      np.vstack([xs, np.full(n, -half_width)])])

poses_crawl, cmds_crawl = [], []
for i in range(160):
    poses_crawl.append((0.08 * i * DT, 0.0, 0.0))   # 0.08 m/s, genuinely moving
    cmds_crawl.append((0.08, 0.02))                 # below v_eps -> cmd_low true
guarded = StuckEscapePolicy(CFG, criterion="displacement")
d_guard = feed(guarded, poses_crawl, cmds_crawl, corridor_points())
check("corridor + crawling: guard suppresses escape",
      not any(d.entered for d in d_guard))
check("corridor detected", guarded.in_corridor)
# Same corridor, but the robot is truly stopped -> escape must still fire.
stopped = StuckEscapePolicy(CFG, criterion="displacement")
d_stop = feed(stopped, [(0.0, 0.0, 0.0)] * 200, [(0.0, 0.0)] * 200,
              corridor_points())
check("corridor + true standstill: escape still fires", any(d.entered for d in d_stop))
first = next(i for i, d in enumerate(d_stop) if d.entered)
check("standstill wait is respected (>=3s before firing)", first * DT >= 3.0,
      f"fired at {first * DT:.2f}s")
# Guard must not weaken the plain frontal-wall case (no side walls -> no corridor).
p_wall = StuckEscapePolicy(CFG, criterion="displacement")
d_wall = feed(p_wall, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, front_wall(0.9))
check("frontal wall (not a corridor) still escapes in 10 frames",
      any(d.entered for d in d_wall) and not p_wall.in_corridor)
# Asymmetric sides (wall on one side only) must not count as a corridor.
one_side = np.vstack([np.linspace(0, 3, 30), np.full(30, 0.5)])
p_one = StuckEscapePolicy(CFG, criterion="displacement")
feed(p_one, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12,
     np.hstack([front_wall(0.9), one_side]))
check("single-side wall is not a corridor", not p_one.in_corridor)
# Guard is switchable.
off = StuckEscapePolicy(dict(CFG, corridor_guard_enable=False),
                        criterion="displacement")
d_off = feed(off, poses_crawl, cmds_crawl, corridor_points())
check("corridor guard is switchable (off -> old behaviour returns)",
      any(d.entered for d in d_off))

# ---------------------------------------------------------------- [2f] Rotation clearance gate
GATE = dict(CFG, rotate_gate_enable=True, rotate_clearance_m=0.60,
            back_speed=0.2, rotate_speed=0.5)

def wall_pts(half, length=4.0, n=60):
    xs = np.linspace(-1.5, length, n)
    return np.hstack([np.vstack([xs, np.full(n, half)]),
                      np.vstack([xs, np.full(n, -half)])])

p2f = StuckEscapePolicy(GATE)
# 0.75m corridor: wall distance 0.375m < 0.60m -> rotation blocked
allowed, near = p2f.rotation_allowed(state(), wall_pts(0.375))
check("[2f] 0.75m corridor -> rotation blocked",
      allowed is False and abs(near - 0.375) < 0.02, f"allowed={allowed} near={near}")
# 1.15m corridor: wall distance 0.575m < 0.60m -> rotation blocked
allowed, _ = p2f.rotation_allowed(state(), wall_pts(0.575))
check("[2f] 1.15m corridor -> rotation still blocked (only 4cm net margin)",
      allowed is False, f"allowed={allowed}")
# 1.6m corridor: wall distance 0.8m > 0.60m -> rotation allowed
allowed, _ = p2f.rotation_allowed(state(), wall_pts(0.8))
check("[2f] 1.6m corridor -> rotation allowed", allowed is True, f"allowed={allowed}")
# Open space with distant frontal wall -> rotation allowed
far_wall = np.vstack([np.full(30, 1.2), np.linspace(-1.0, 1.0, 30)])
allowed, _ = p2f.rotation_allowed(state(), far_wall)
check("[2f] open space with distant frontal wall -> rotation allowed",
      allowed is True, f"allowed={allowed}")

# Gated escape command is straight reverse only
p2f.rotation_blocked = True
p2f.direction = 1.0
cmd = p2f._escape_cmd()
check("[2f] gated escape command is straight reverse only",
      abs(float(cmd[0, 0]) + 0.2) < 1e-9 and abs(float(cmd[1, 0])) < 1e-9, f"cmd={cmd.ravel()}")
p2f.rotation_blocked = False
cmd = p2f._escape_cmd()
check("[2f] ungated escape command still turns",
      abs(float(cmd[1, 0]) - 0.5) < 1e-9, f"cmd={cmd.ravel()}")

# Disabling the gate restores legacy rotation behavior
p2f_off = StuckEscapePolicy(dict(GATE, rotate_gate_enable=False))
allowed, near = p2f_off.rotation_allowed(state(), wall_pts(0.375))
check("[2f] gate disabled -> old behaviour restored",
      allowed is True and near is None, f"allowed={allowed} near={near}")

print("\n[3] progress mode degrades safely with no reference path")
p7 = StuckEscapePolicy(CFG, criterion="progress")
d7 = feed(p7, [(0.0, 0.0, 0.0)] * 12, [(0.0, 0.0)] * 12, front_wall(0.9))
check("no path installed -> still escapes via displacement/cmd_low",
      any(d.entered for d in d7))
# A replanned path resets progress history without breaking the policy.
p8 = StuckEscapePolicy(CFG, criterion="progress")
feed(p8, poses_rock[:40], cmds_rock[:40], front_wall(0.9), path=straight_path())
p8.set_reference_path(straight_path(length=8.0))
check("set_reference_path clears stale arc lengths",
      len(p8.progress_history) == 0)

print("\n[4] candidate-directed recovery")
RECOVERY = dict(CFG, early_replan_enable=False, rotate_gate_enable=False)
p9 = StuckEscapePolicy(RECOVERY, criterion="displacement")
d9 = []
for i in range(12):
    t = i * DT
    p9.note_pose(t, 0.0, 0.0)
    p9.note_cmd(0.0, 0.0)
    d9.append(p9.decide(t, state(), front_wall(0.9),
                        require_preferred_direction=True))
check("missing candidate holds zero instead of entering escape",
      not any(d.entered for d in d9)
      and d9[-1].info.get("recovery_wait")
      and np.allclose(d9[-1].cmd, 0.0))

left = np.vstack([np.full(12, 0.8), np.linspace(0.1, 0.4, 12)])
right = np.vstack([np.full(12, 1.2), np.linspace(-0.4, -0.1, 12)])
side_points = np.hstack([left, right])
p10 = StuckEscapePolicy(RECOVERY, criterion="displacement")
d10 = []
for i in range(12):
    t = i * DT
    p10.note_pose(t, 0.0, 0.0)
    p10.note_cmd(0.0, 0.0)
    d10.append(p10.decide(t, state(), side_points,
                          preferred_direction=1,
                          require_preferred_direction=True))
entered10 = next((d for d in d10 if d.entered), None)
check("candidate side overrides locally clearer opposite side",
      entered10 is not None and p10.direction == 1.0
      and entered10.info.get("candidate_forced"),
      f"direction={p10.direction} info={entered10.info if entered10 else None}")

unsafe_left = np.vstack([np.full(12, 0.5), np.linspace(0.1, 0.4, 12)])
p11 = StuckEscapePolicy(RECOVERY, criterion="displacement")
d11 = []
for i in range(12):
    t = i * DT
    p11.note_pose(t, 0.0, 0.0)
    p11.note_cmd(0.0, 0.0)
    d11.append(p11.decide(t, state(), np.hstack([unsafe_left, right]),
                          preferred_direction=1,
                          require_preferred_direction=True,
                          preferred_min_clearance=0.57))
check("unsafe candidate side remains in zero wait",
      not any(d.entered for d in d11)
      and d11[-1].info.get("recovery_wait_reason") == "candidate_side_unsafe"
      and np.allclose(d11[-1].cmd, 0.0))

print("\n" + ("ALL PASS" if not FAILURES else f"FAILED: {FAILURES}"))
sys.exit(1 if FAILURES else 0)
