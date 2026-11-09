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

"""SW01 NeuPAN simulation benchmark runner (irsim).

Usage:
  python benchmark_sw01.py --scene narrow_corridor --runs 1
  python benchmark_sw01.py --scene narrow_corridor --runs 3 \
      --set adjust.d_max=0.1 --set adjust.eta=6 --set receding=8
  python benchmark_sw01.py --scene open --runs 1 --delay-ms 100
"""

import argparse
import copy
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import yaml

NEUPAN_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NEUPAN_ROOT))

from neupan import neupan  # noqa: E402
from neupan.util.stuck_escape import StuckEscapePolicy  # noqa: E402
from neupan.util.adaptive_d_max import AdaptiveDMax, robot_half_width  # noqa: E402
from neupan.util.curvature_speed import CurvatureSpeedGovernor  # noqa: E402
from sim_system import WorldGrid, SlidingMapSim, GlobalPathSim, GlobalMemorySim  # noqa: E402
import irsim  # noqa: E402

REAL_CONFIG = NEUPAN_ROOT.parent / "robots" / "sw01" / "neupan" / "config.yaml"
REAL_LIDAR_OVERRIDE = {"range_max": 4.5, "angle_range": 1.5708, "number": 93}


def load_real_defaults():
    """Load stuck_escape, adaptive_d_max, and curvature_speed defaults from robots/sw01/neupan/config.yaml."""
    try:
        cfg = load_yaml(REAL_CONFIG)
    except Exception:
        return {}, {}, {}
    return (cfg.get("stuck_escape") or {}, cfg.get("adaptive_d_max") or {},
            cfg.get("curvature_speed") or {})


class _KeepOldPath(Exception):
    pass


def dist_to_polyline(pt, pts):
    ax, ay = pts[:-1, 0], pts[:-1, 1]
    dx, dy = pts[1:, 0] - ax, pts[1:, 1] - ay
    L2 = dx * dx + dy * dy
    L2[L2 < 1e-9] = 1e-9
    u = np.clip(((pt[0] - ax) * dx + (pt[1] - ay) * dy) / L2, 0, 1)
    return float(np.min(np.hypot(pt[0] - (ax + u * dx), pt[1] - (ay + u * dy))))


def path_corners(pts, thr_deg=40.0, min_seg=1.0):
    """Detect corner vertices along a reference polyline for corner overshoot metrics."""
    import math as _m
    out = []
    anchor = 0
    j = 1
    thr = _m.radians(thr_deg)
    while j < len(pts) - 1:
        if _m.hypot(*(pts[j] - pts[anchor])) < min_seg:
            j += 1
            continue
        k = j
        while k < len(pts) - 2 and _m.hypot(*(pts[k + 1] - pts[j])) < min_seg:
            k += 1
        v1 = pts[j] - pts[anchor]
        v2 = pts[k + 1] - pts[j]
        da = abs((_m.atan2(v2[1], v2[0]) - _m.atan2(v1[1], v1[0]) + _m.pi) % (2 * _m.pi) - _m.pi)
        if da > thr:
            out.append(tuple(pts[j]))
            anchor = k + 1
            j = anchor + 1
        else:
            anchor = j
            j += 1
    return out

BENCH_DIR = Path(__file__).resolve().parent
TMP_DIR = BENCH_DIR / ".tmp"
RESULTS_DIR = BENCH_DIR / "results"

# Default stuck_escape configuration aligned with Xuanji SW01
DEFAULT_ESCAPE_CFG = dict(
    enable=True,
    displacement_window_sec=1.0,
    displacement_threshold_m=0.05,
    progress_threshold_m=0.05,
    clearing_guard_enable=True,
    clearing_guard_delta_m=0.05,
    v_eps=0.15,
    w_eps=0.30,
    trigger_frames=10,
    front_obs_threshold=1.0,
    cone_half_width_rad=0.7,
    back_speed=0.2,
    rotate_speed=0.5,
    rotate_gate_enable=True,
    rotate_clearance_m=0.60,
    rotate_bounded_enable=False,
    rotate_bounded_margin_m=0.05,
    rotate_open_enable=False,
    rotate_open_clearance_m=0.57,
    rotate_sweep_check_enable=False,
    rotate_sweep_margin_m=0.05,
    rotate_sweep_step_deg=15.0,
    front_path_bypass_enable=False,
    front_path_clearance_m=0.40,
    corridor_fast_escape_enable=False,
    corridor_fast_standstill_sec=0.5,
    creep_detect_enable=False,
    creep_v_eps=0.10,
    creep_timeout_sec=1.5,
    oscillation_detect_enable=False,
    osc_window_sec=4.0,
    osc_net_disp_m=0.30,
    osc_travel_m=1.0,
    rotate_bounded_half_len=0.48,
    rotate_bounded_half_wid=0.275,
    duration_sec=2.0,
    exit_obs_distance=1.0,
    side_search_radius_m=3.5,
    limit_cycle_consecutive_n=2,
    limit_cycle_window_sec=20.0,
    min_rotation_rad=0.35,
    early_replan_enable=True,
    early_replan_trigger_frames=5,
    early_replan_cooldown_sec=2.0,
    post_escape_grace_q_theta_scale=0.25,
    post_escape_grace_q_s_scale=0.5,
    post_escape_grace_min_distance_m=0.5,
    post_escape_grace_clear_threshold_m=1.0,
    post_escape_grace_clear_dwell_sec=0.5,
    post_escape_grace_max_sec=5.0,
)


# Default adaptive_d_max configuration
DEFAULT_ADAPTIVE_CFG = dict(
    enable=False,
    min=0.10,
    max=0.30,
    safety_margin=0.05,
    forward_window_m=2.0,
    lateral_window_m=2.0,
    update_period_sec=0.5,
    frontal_exclude_scale=0.8,
    forward_slices=4,
    min_populated_slices=2,
    clearance_percentile=10.0,
    shrink_confirmations=2,
    guard_min_dist_m=0.05,
    guard_frames=3,
    guard_cooldown_sec=5.0,
)


def read_nrmp_weights(planner):
    """Read current (q_theta, q_s) from the planner."""
    try:
        layer = planner.pan.nrmp_layer
        return (float(layer.q_theta.detach().item()),
                float(layer.q_s.detach().item()))
    except Exception:
        return 0.2, 2.0


def apply_nrmp_weights(planner, q_theta, q_s):
    try:
        planner.update_adjust_parameters(q_theta=q_theta, q_s=q_s)
    except Exception as e:
        print(f"  [escape] weight update skipped: {e}")


def install_reference_path(policy, planner):
    """Install the planner's current reference path into `policy`."""
    try:
        path = planner.initial_path
    except Exception:
        return False
    if not path:
        return False
    policy.set_reference_path(path)
    return True


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_yaml(data, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)


def lidar_config(env_cfg: dict) -> dict:
    for sensor in env_cfg.get("robot", [{}])[0].get("sensors", []):
        if sensor.get("type") == "lidar2d":
            return {k: sensor.get(k) for k in ("range_max", "angle_range", "number", "noise")}
    return {}


def apply_overrides(cfg: dict, overrides: dict) -> dict:
    """Apply dot-separated path overrides (e.g. 'adjust.d_max') into a nested dictionary."""
    for dotted, value in overrides.items():
        keys = dotted.split(".")
        node = cfg
        for k in keys[:-1]:
            if k not in node or not isinstance(node[k], dict):
                node[k] = {}
            node = node[k]
        node[keys[-1]] = value
    return cfg


def parse_set_pairs(pairs):
    """Parse ['adjust.d_max=0.1', 'receding=8'] into a dictionary of overrides."""
    overrides = {}
    for pair in pairs or []:
        if "=" not in pair:
            raise ValueError(f"Invalid override format (expected key=value): {pair}")
        key, raw = pair.split("=", 1)
        raw = raw.strip()
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = raw
        overrides[key.strip()] = value
    return overrides


def lateral_errors(traj, start, goal):
    """Signed perpendicular distance sequence from `traj` points to the start-goal line."""
    start = np.asarray(start, dtype=float).reshape(2)
    goal = np.asarray(goal, dtype=float).reshape(2)
    d = goal - start
    norm = np.linalg.norm(d)
    if norm < 1e-9:
        return np.zeros(len(traj))
    n = np.array([-d[1], d[0]]) / norm
    errs = [float(np.dot((p - start), n)) for p in traj]
    return np.asarray(errs)


def omega_metrics(omegas, deadband=0.01):
    """Angular velocity oscillation metrics: sign flips, mean |omega|, and mean |d_omega|."""
    om = np.asarray(omegas, dtype=float)
    if len(om) == 0:
        return dict(omega_flips=0, omega_abs_mean=0.0, domega_mean=0.0)
    flips = 0
    for a, b in zip(om[:-1], om[1:]):
        if abs(a) < deadband or abs(b) < deadband:
            continue
        if np.sign(a) != np.sign(b):
            flips += 1
    dom = np.abs(np.diff(om)) if len(om) > 1 else np.zeros(1)
    return dict(
        omega_flips=int(flips),
        omega_abs_mean=float(np.mean(np.abs(om))),
        domega_mean=float(np.mean(dom)),
    )


def run_once(env_file: Path, planner_template: dict, overrides: dict,
             max_steps: int, delay_ms: int, warmup: int = 2,
             perception_delay_ms: int = 0, chassis_tau: float = 0.0,
             enable_escape: bool = True, goal_xy_target: list = None,
             escape_cfg: dict = None, escape_criterion: str = "displacement",
             adaptive_cfg: dict = None, global_path=None, curvature_cfg: dict = None,
             save_ani: bool = False, ani_name: str = "",
             path_validity_replan: bool = False,
             path_validity_cooldown_sec: float = 5.0,
             lidar_override: dict = None,
             sliding_map_footprint_radius_m: float = 0.60):
    """Run one simulation episode and return a metrics dictionary."""
    planner_cfg = apply_overrides(copy.deepcopy(planner_template), overrides)
    gpath_waypoints = None
    env_cfg_pre = load_yaml(env_file)
    if lidar_override:
        env_file_eff = TMP_DIR / f"env_lidar_{env_file.parent.name}.yaml"
        cfg_l = copy.deepcopy(env_cfg_pre)
        for sensor in cfg_l.get("robot", [{}])[0].get("sensors", []):
            if sensor.get("type") == "lidar2d":
                sensor.update(lidar_override)
        save_yaml(cfg_l, env_file_eff)
        env_file = env_file_eff
        env_cfg_pre = cfg_l
    if global_path is not None:
        global_path.sliding_map.reset()
        s_xy = env_cfg_pre["robot"][0]["state"][:2]
        g_xy = env_cfg_pre["robot"][0]["goal"][:2]
        gpath_waypoints = global_path.plan(s_xy, g_xy)
        if gpath_waypoints:
            planner_cfg.setdefault("ipath", {})["waypoints"] = [
                [float(a), float(b), float(c)] for a, b, c in gpath_waypoints]
    tmp_planner = TMP_DIR / "planner_run.yaml"
    save_yaml(planner_cfg, tmp_planner)

    env = irsim.make(str(env_file), save_ani=save_ani, full=False, display=save_ani)
    planner = neupan.init_from_yaml(str(tmp_planner))

    curv = CurvatureSpeedGovernor(
        curvature_cfg or {}, float(planner_cfg.get("ref_speed", 0.5)))
    curv_path = (np.array([[w[0], w[1]] for w in gpath_waypoints])
                 if gpath_waypoints else None)
    if curv_path is not None:
        curv.set_path(curv_path)
    curv_trace = []
    path_lat = []
    esc_episodes = []
    _esc_ep = None
    _prev_yaw = None
    _esc_dyaw = 0.0
    prev_ob_pos = {}
    dyn_ids = set()
    ghost_frames = 0
    ghost_total = 0
    path_validity_replans = 0
    last_path_validity_replan_t = -1e9
    frame_log = []
    enter_times = []

    env_cfg = env_cfg_pre
    start_xy = env_cfg["robot"][0]["state"][:2]
    goal_xy = env_cfg["robot"][0]["goal"][:2]
    step_time = env_cfg["world"].get("step_time", 0.1)
    delay_steps = max(0, int(round(delay_ms / 1000.0 / step_time)))
    perception_delay_steps = max(0, int(round(perception_delay_ms / 1000.0 / step_time)))

    traj = []
    omegas = []
    compute_ms = []
    min_dist = float("inf")
    action_buf = [np.zeros((2, 1))] * delay_steps
    scan_buf = []
    chassis_vel = np.zeros((2, 1))

    result = "timeout"
    steps_run = 0
    GOAL_PROXIMITY = 0.3
    ESCAPE_MAX = 8

    escape_policy = StuckEscapePolicy(
        dict(escape_cfg or DEFAULT_ESCAPE_CFG, enable=enable_escape),
        criterion=escape_criterion,
    )
    q_theta_nominal, q_s_nominal = read_nrmp_weights(planner)
    escape_replans = 0
    ref_path_installed = False

    iters_run = []
    baseline_d_max = float(planner_cfg.get("adjust", {}).get("d_max", 0.4))
    adaptive = AdaptiveDMax(adaptive_cfg or DEFAULT_ADAPTIVE_CFG,
                            baseline_d_max=baseline_d_max)
    half_width_robot = robot_half_width(planner.robot)
    guard_trips = 0
    d_max_trace = []

    for k in range(max_steps):
        steps_run = k + 1
        sim_t = k * step_time
        robot_state = env.get_robot_state()
        lidar_scan = env.get_lidar_scan()

        scan_buf.append(lidar_scan)
        if len(scan_buf) > perception_delay_steps:
            scan_used = scan_buf.pop(0)
        else:
            scan_used = scan_buf[0]
        points = planner.scan_to_point(robot_state, scan_used)

        if global_path is not None and k % 3 == 0:
            try:
                _sc = scan_used
                _n = len(_sc["ranges"])
                _ang = np.linspace(_sc["angle_min"], _sc["angle_max"], _n)
                global_path.sliding_map.raytrace_update(
                    sim_t, (float(robot_state[0, 0]), float(robot_state[1, 0])),
                    float(robot_state[2, 0]), _ang, np.asarray(_sc["ranges"], dtype=float),
                    footprint_radius=sliding_map_footprint_radius_m,
                    range_max=float(_sc.get("range_max", 8.0)))
            except Exception:
                pass
            global_path.sliding_map.decay(sim_t)
            global_path.sliding_map.apply_window(
                (float(robot_state[0, 0]), float(robot_state[1, 0])))
            if curv_path is not None and len(curv_path) > 2:
                try:
                    for o in getattr(env, "obstacle_list", []) or []:
                        op = np.asarray(o.state[:2], dtype=float).flatten()
                        if dist_to_polyline(op, curv_path) < 0.40:
                            ghost_frames += 1
                            break
                except Exception:
                    pass
            ghost_total += 1
            if (path_validity_replan and curv_path is not None and len(curv_path) > 2
                    and sim_t - last_path_validity_replan_t >= path_validity_cooldown_sec):
                occ_now = global_path._occ()
                ix = ((curv_path[:, 0] - global_path.world.x0) / global_path.world.res).astype(int)
                iy = ((curv_path[:, 1] - global_path.world.y0) / global_path.world.res).astype(int)
                valid = ((ix >= 0) & (ix < global_path.world.nx)
                         & (iy >= 0) & (iy < global_path.world.ny))
                dxp = curv_path[:, 0] - float(robot_state[0, 0])
                dyp = curv_path[:, 1] - float(robot_state[1, 0])
                ahead = dxp * math.cos(float(robot_state[2, 0])) + dyp * math.sin(float(robot_state[2, 0])) > 0.3
                if np.any(valid & ahead & occ_now[np.clip(ix, 0, global_path.world.nx-1), np.clip(iy, 0, global_path.world.ny-1)]):
                    wps = global_path.plan((float(robot_state[0, 0]), float(robot_state[1, 0])), goal_xy_target)
                    last_path_validity_replan_t = sim_t
                    if wps:
                        planner.reset(reset_velocity=True)
                        planner.set_initial_path([np.array([a,b,c,1.0]).reshape(4,1) for a,b,c in wps])
                        curv_path = np.array([[a,b] for a,b,_ in wps])
                        curv.set_path(curv_path)
                        ref_path_installed = False
                        path_validity_replans += 1
                        print(f"  [path-validity] replan @ {sim_t:.1f}s (dynamic obstacle intersects path)")

        if adaptive.enable:
            try:
                md = planner.min_distance
                md = float(md.detach().cpu()) if hasattr(md, "detach") else float(md)
            except Exception:
                md = None
            restored = adaptive.note_min_distance(sim_t, md)
            if restored is not None:
                planner.update_adjust_parameters(d_max=restored)
                guard_trips += 1
                print(f"  [adaptive] guard tripped (min_dist={md:.3f}) "
                      f"-> d_max restored to {restored}")
            applied = adaptive.update(
                sim_t, robot_state, points, half_width_robot,
                lambda dm: planner.update_adjust_parameters(d_max=dm),
            )
            if applied is not None:
                d_max_trace.append(applied)

        if curv.enable and curv_path is not None:
            rs = curv.update(float(robot_state[0, 0]), float(robot_state[1, 0]))
            planner.set_reference_speed(rs)
            curv_trace.append((sim_t, rs, curv.last_kappa))

        t0 = time.perf_counter()
        action, info = planner(robot_state, points)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if k >= warmup:
            compute_ms.append(dt_ms)
            it = getattr(getattr(planner, "pan", None), "perf_iters", None)
            if it:
                iters_run.append(int(it))

        if delay_steps > 0:
            action_buf.append(action)
            action_out = action_buf.pop(0)
        else:
            action_out = action

        if chassis_tau > 1e-6:
            alpha = step_time / chassis_tau
            chassis_vel = chassis_vel + (action_out - chassis_vel) * alpha
            action_applied = chassis_vel
        else:
            action_applied = action_out

        if enable_escape:
            state_xy = robot_state[0:2].flatten()
            if not ref_path_installed and escape_criterion == "progress":
                if install_reference_path(escape_policy, planner):
                    ref_path_installed = True
            escape_policy.note_pose(sim_t, state_xy[0], state_xy[1])
            if ref_path_installed:
                escape_policy.note_progress(sim_t, state_xy[0], state_xy[1])
            escape_policy.note_cmd(float(action_applied[0, 0]),
                                   float(action_applied[1, 0]))
            escape_policy.note_rotation(sim_t, float(action_applied[1, 0]))
            escape_policy.note_motion(sim_t, float(action_applied[0, 0]))

            dist_to_goal = (np.linalg.norm(state_xy - goal_xy_target)
                            if goal_xy_target else float("inf"))
            near_goal = dist_to_goal <= GOAL_PROXIMITY
            budget_left = escape_policy.escape_count < ESCAPE_MAX

            if near_goal or not budget_left:
                decision = None
            else:
                decision = escape_policy.decide(sim_t, robot_state, points)

            restore = escape_policy.update_grace(sim_t, robot_state, points)
            if restore is not None:
                apply_nrmp_weights(planner, q_theta_nominal, q_s_nominal)

            if decision is not None and os.environ.get("BENCH_ESCAPE_DEBUG"):
                if k % 25 == 0:
                    d = decision.info
                    print(f"  [dbg] k={k} {d.get('metric_name')}={d.get('metric')} "
                          f"stalled={d.get('stalled')} front={d.get('front_obs')} "
                          f"count={d.get('count')} active={escape_policy.active}")
            if decision is not None:
                if decision.entered:
                    d = decision.info
                    enter_times.append(sim_t)
                    print(f"  [escape #{escape_policy.escape_count}] ENTER @ "
                          f"({state_xy[0]:.2f},{state_xy[1]:.2f}) "
                          f"{d.get('metric_name')}={d.get('metric')} "
                          f"front={d.get('front_obs'):.2f} dir={d.get('direction'):+.0f}")
                if decision.grace_scales is not None:
                    qt, qs = decision.grace_scales
                    apply_nrmp_weights(planner, q_theta_nominal * qt,
                                       q_s_nominal * qs)
                if decision.request_replan:
                    escape_replans += 1
                    try:
                        planner.reset(reset_velocity=True)
                        _cur = (float(robot_state[0, 0]), float(robot_state[1, 0]))
                        wps = (global_path.plan(_cur, goal_xy_target)
                               if global_path is not None else None)
                        if global_path is not None and not wps:
                            print("  [replan] A* NO_PATH -> keeping existing path")
                            wps = None
                            raise _KeepOldPath()
                        if wps:
                            planner.set_initial_path([
                                np.array([a, b, c, 1.0]).reshape(4, 1)
                                for a, b, c in wps])
                            curv_path = np.array([[a, b] for a, b, _ in wps])
                            curv.set_path(curv_path)
                        else:
                            planner.set_initial_path_from_state(robot_state)
                        ref_path_installed = False
                    except _KeepOldPath:
                        pass
                    except Exception as e:
                        print(f"  [escape] replan failed: {e}")
                if decision.cmd is not None:
                    action_applied = decision.cmd

        traj.append(robot_state[0:2].flatten().copy())
        omegas.append(float(action_applied[1, 0]))

        if curv_path is not None and len(curv_path) > 2:
            path_lat.append((sim_t, dist_to_polyline(
                robot_state[0:2].flatten(), curv_path)))
        _yaw = float(robot_state[2, 0])
        if enable_escape:
            if escape_policy.active:
                if _esc_ep is None:
                    _esc_ep = [sim_t, sim_t, _yaw]
                    _esc_dyaw = 0.0
                else:
                    if _prev_yaw is not None:
                        d = _yaw - _prev_yaw
                        while d > np.pi: d -= 2 * np.pi
                        while d < -np.pi: d += 2 * np.pi
                        _esc_dyaw += d
                _esc_ep[1] = sim_t
            elif _esc_ep is not None:
                esc_episodes.append((_esc_ep[0], _esc_ep[1], np.degrees(_esc_dyaw)))
                _esc_ep = None
        _prev_yaw = _yaw

        md_now = float("nan")
        try:
            md = planner.min_distance
            md = float(md.detach().cpu()) if hasattr(md, "detach") else float(md)
            min_dist = min(min_dist, md)
            md_now = md
        except Exception:
            pass
        frame_log.append((sim_t, float(action_applied[0, 0]), float(action_applied[1, 0]),
                          md_now, bool(escape_policy.active if enable_escape else False),
                          float(robot_state[0, 0]), float(robot_state[1, 0]),
                          float(robot_state[2, 0])))

        if info.get("arrive"):
            result = "success"
            break
        if info.get("stop"):
            result = "collision_stop"
            break

        env.step(action_applied)
        if save_ani:
            env.render(interval=0.001)

    if save_ani and ani_name:
        env.end(ani_name=ani_name)
        _gif = BENCH_DIR.parents[1] / "animation" / (ani_name + ".gif")
        if _gif.exists():
            print(f"  [ani] GIF -> {_gif}")
    else:
        env.end()

    if os.environ.get("BENCH_SAVE_TRAJ") and traj:
        try:
            td = TMP_DIR / f"traj_{int(time.time()*1000)%10**8}.npz"
            fl = np.array(frame_log) if frame_log else np.zeros((0, 5))
            np.savez(td, traj=np.array(traj),
                     path=curv_path if curv_path is not None else np.zeros((0, 2)),
                     omegas=np.array(omegas),
                     frames=fl, enter_times=np.array(enter_times),
                     env_file=str(env_file))
            print(f"  [traj] saved {td}")
        except Exception:
            pass
    lat = lateral_errors(traj, start_xy, goal_xy) if traj else np.zeros(1)
    om = omega_metrics(omegas)
    if _esc_ep is not None:
        esc_episodes.append((_esc_ep[0], _esc_ep[1], np.degrees(_esc_dyaw)))
    pl = [e for _, e in path_lat]
    corner_ov = None
    if path_lat and curv_path is not None and len(curv_path) > 2:
        corners = path_corners(curv_path)
        tt = np.array([t for t, _ in path_lat])
        ee = np.array(pl)
        tr = np.array(traj) if traj else np.zeros((1, 2))
        ovs = []
        for cxy in corners:
            d = np.hypot(tr[:, 0] - cxy[0], tr[:, 1] - cxy[1])
            i_c = int(np.argmin(d))
            t_c = i_c * (path_lat[1][0] - path_lat[0][0] if len(path_lat) > 1 else 0.1)
            m = (tt >= t_c) & (tt <= t_c + 4.0)
            if m.sum():
                ovs.append(float(ee[m].max()))
        corner_ov = round(max(ovs), 3) if ovs else None
    collision = bool(min_dist < 0.01)
    if collision and result != "success":
        result = "collision"

    return dict(
        result=result,
        success=result == "success",
        collision=collision,
        delay_ms=delay_ms,
        perception_delay_ms=perception_delay_ms,
        chassis_tau=chassis_tau,
        lidar=lidar_config(env_cfg_pre),
        sliding_map_footprint_radius_m=sliding_map_footprint_radius_m,
        escape_count=escape_policy.escape_count,
        escape_replans=escape_replans,
        iters_mean=round(float(np.mean(iters_run)), 2) if iters_run else None,
        iters_max=max(iters_run) if iters_run else None,
        adaptive_guard_trips=guard_trips,
        d_max_min=round(min(d_max_trace), 3) if d_max_trace else None,
        d_max_max=round(max(d_max_trace), 3) if d_max_trace else None,
        steps=steps_run,
        sim_time_s=round(steps_run * step_time, 2),
        lat_err_mean=round(float(np.mean(np.abs(lat))), 4),
        lat_err_max=round(float(np.max(np.abs(lat))), 4),
        path_lat_mean=round(float(np.mean(pl)), 3) if pl else None,
        path_lat_max=round(float(np.max(pl)), 3) if pl else None,
        corner_overshoot_max=corner_ov,
        escape_rot_deg=[round(a, 1) for _, _, a in esc_episodes],
        curv_v_min=round(min(v for _, v, _ in curv_trace), 3) if curv_trace else None,
        ghost_ratio=round(ghost_frames / ghost_total, 3) if ghost_total else None,
        unknown_obs=len(getattr(global_path.world, "unknown", []) if global_path else []),
        path_validity_replans=path_validity_replans,
        min_dist=round(min_dist, 4) if np.isfinite(min_dist) else None,
        compute_ms_median=round(float(np.median(compute_ms)), 2) if compute_ms else None,
        compute_ms_p99=round(float(np.percentile(compute_ms, 99)), 2) if compute_ms else None,
        **om,
    )


def aggregate(runs):
    """Aggregate metrics across multiple runs."""
    n = len(runs)
    succ = sum(1 for r in runs if r["success"])
    coll = sum(1 for r in runs if r["collision"])
    def _avg(key, filt=None):
        vals = [r[key] for r in runs if r.get(key) is not None and (filt is None or filt(r))]
        return round(float(np.mean(vals)), 4) if vals else None
    return dict(
        runs=n,
        success_rate=round(succ / n, 3),
        collision_rate=round(coll / n, 3),
        sim_time_s_avg=_avg("sim_time_s", lambda r: r["success"]),
        lat_err_mean_avg=_avg("lat_err_mean"),
        lat_err_max_avg=_avg("lat_err_max"),
        omega_flips_avg=_avg("omega_flips"),
        min_dist_min=round(min([r["min_dist"] for r in runs if r["min_dist"] is not None]), 4)
        if any(r["min_dist"] is not None for r in runs) else None,
        iters_mean_avg=_avg("iters_mean"),
        compute_ms_median_avg=_avg("compute_ms_median"),
        compute_ms_p99_max=round(max([r["compute_ms_p99"] for r in runs if r["compute_ms_p99"] is not None]), 2)
        if any(r["compute_ms_p99"] is not None for r in runs) else None,
    )


def main():
    ap = argparse.ArgumentParser(description="SW01 NeuPAN simulation benchmark")
    ap.add_argument("--scene", default=None, choices=[
        "open", "wide_corridor", "narrow_corridor", "dynamic_human",
        "narrow_to_wide", "wide_to_narrow", "head_on", "human_block", "office",
        "narrow_entry", "narrow_long", "narrow_075", "narrow_blocked",
        "corner90", "corridor110", "corridor130", "narrow09_entry", "deadend_T"])
    ap.add_argument("--env-file", default="",
                    help="Custom env.yaml path (its parent directory must also contain planner.yaml)")
    ap.add_argument("--global-path", action="store_true",
                    help="Enable A* global path pipeline (WorldGrid + SlidingMapSim + GlobalPathSim)")
    ap.add_argument("--sliding_map-window", type=float, default=0.0,
                    help="sliding_map sliding window side length in meters (0 = full map)")
    ap.add_argument("--global-memory", action="store_true",
                    help="Enable coarse 0.2m persistent global memory layer")
    ap.add_argument("--sliding_map-decay", type=float, default=3.0,
                    help="sliding_map cell decay time in seconds (0 = persistent)")
    ap.add_argument("--sliding_map-footprint-radius", type=float, default=0.60,
                    help="sliding_map self-footprint filter radius in meters")
    ap.add_argument("--inflation-radius", type=float, default=0.35,
                    help="A* lethal inflation radius in meters")
    ap.add_argument("--dyn-inflation", type=float, default=0.0,
                    help="Independent inflation radius for dynamic memory obstacles (0 = same as static)")
    ap.add_argument("--curvature", action="store_true",
                    help="Enable curvature feedforward speed governor")
    ap.add_argument("--curvature-set", action="append", default=[],
                    help="Override curvature_speed parameter, e.g. --curvature-set a_lat_max=0.5")
    ap.add_argument("--path-validity-replan", action="store_true",
                    help="Trigger A* replan when dynamic sliding_map occupancy intersects the active path")
    ap.add_argument("--path-validity-cooldown", type=float, default=5.0,
                    help="Cooldown in seconds for path-validity replanning")
    ap.add_argument("--save-ani", action="store_true",
                    help="Record GIF animation for run 0")
    ap.add_argument("--real-defaults", action="store_true",
                    help="Load escape/adaptive/curvature defaults from robots/sw01/neupan/config.yaml")
    ap.add_argument("--runs", type=int, default=3, help="Number of trials per parameter set")
    ap.add_argument("--set", action="append", default=[], help="Parameter override key=value")
    ap.add_argument("--delay-ms", type=int, default=0, help="Action delay injection (ms)")
    ap.add_argument("--perception-delay-ms", type=int, default=0, help="Perception delay injection (ms)")
    ap.add_argument("--chassis-tau", type=float, default=0.0, help="First-order chassis time constant (s)")
    ap.add_argument("--real-lidar", action="store_true",
                    help="Align simulated lidar with real /scan (90 deg FOV / 4.5m / 93 rays)")
    ap.add_argument("--max-steps", type=int, default=900)
    ap.add_argument("--tag", default="", help="Result directory tag")
    ap.add_argument("--no-escape", action="store_true", help="Disable stuck_escape")
    ap.add_argument("--escape-criterion", default="from-config",
                    choices=["displacement", "progress", "from-config"],
                    help="Stall criterion")
    ap.add_argument("--adaptive-d-max", action="store_true",
                    help="Enable adaptive_d_max")
    ap.add_argument("--adaptive-set", action="append", default=[],
                    help="Override adaptive_d_max parameter, e.g. --adaptive-set max=0.15")
    ap.add_argument("--escape-set", action="append", default=[],
                    help="Override stuck_escape parameter, e.g. --escape-set trigger_frames=5")
    args = ap.parse_args()
    if not args.scene and not args.env_file:
        raise SystemExit("Either --scene or --env-file is required")

    overrides = parse_set_pairs(args.set)
    real_esc, real_adp, real_curv = ({}, {}, {}) if not args.real_defaults else load_real_defaults()
    escape_cfg = dict(DEFAULT_ESCAPE_CFG)
    escape_cfg.update({k: v for k, v in real_esc.items() if k not in ("enable", "criterion")})
    for pair in args.escape_set:
        if "=" not in pair:
            raise SystemExit(f"--escape-set requires key=value, got: {pair}")
        key, raw = pair.split("=", 1)
        key = key.strip()
        if key not in escape_cfg:
            raise SystemExit(f"Unknown stuck_escape parameter: {key}")
        escape_cfg[key] = yaml.safe_load(raw)
    adaptive_cfg = dict(DEFAULT_ADAPTIVE_CFG)
    adaptive_cfg.update({k: v for k, v in real_adp.items() if k != "enable"})
    adaptive_cfg["enable"] = bool(args.adaptive_d_max or real_adp.get("enable", False))
    curvature_cfg = dict(real_curv)
    curvature_cfg["enable"] = bool(args.curvature)
    for pair in args.curvature_set:
        if "=" not in pair:
            raise SystemExit(f"--curvature-set requires key=value, got: {pair}")
        k, raw = pair.split("=", 1)
        curvature_cfg[k.strip()] = yaml.safe_load(raw)
    if args.escape_criterion == "from-config":
        args.escape_criterion = str(real_esc.get("criterion", "displacement"))
    for pair in args.adaptive_set:
        if "=" not in pair:
            raise SystemExit(f"--adaptive-set requires key=value, got: {pair}")
        key, raw = pair.split("=", 1)
        key = key.strip()
        if key not in adaptive_cfg:
            raise SystemExit(f"Unknown adaptive_d_max parameter: {key}")
        adaptive_cfg[key] = yaml.safe_load(raw)
    if args.env_file:
        env_file = Path(args.env_file)
        scene_dir = env_file.parent
        scene_name = scene_dir.name
    else:
        scene_dir = BENCH_DIR / args.scene
        env_file = scene_dir / "env.yaml"
        scene_name = args.scene
    planner_template = load_yaml(scene_dir / "planner.yaml")
    lidar_override = dict(REAL_LIDAR_OVERRIDE) if args.real_lidar else None

    tag = args.tag or ("baseline" if not overrides else "_".join(
        f"{k.split('.')[-1]}={v}" for k, v in overrides.items()))
    if args.delay_ms:
        tag += f"__delay{args.delay_ms}ms"
    if args.perception_delay_ms:
        tag += f"__percep{args.perception_delay_ms}ms"
    if args.chassis_tau > 0:
        tag += f"__tau{args.chassis_tau}"
    if args.escape_criterion != "displacement":
        tag += f"__esc-{args.escape_criterion}"
    if adaptive_cfg.get("enable"):
        tag += "__adaptive"
    if args.global_path:
        tag += "__gpath"
    if args.curvature:
        tag += "__curv"
    if args.real_lidar:
        tag += "__real-lidar"
    out_dir = RESULTS_DIR / f"{scene_name}__{tag}__{time.strftime('%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[benchmark] scene={args.scene} overrides={overrides or 'baseline'} "
          f"delay={args.delay_ms}ms perception_delay={args.perception_delay_ms}ms "
          f"chassis_tau={args.chassis_tau}s escape={not args.no_escape} runs={args.runs} -> {out_dir}")

    env_cfg = load_yaml(env_file)
    goal_xy = env_cfg["robot"][0]["goal"][:2]
    effective_lidar = dict(lidar_config(env_cfg), **(lidar_override or {}))
    print(f"[benchmark] lidar={effective_lidar}")

    global_path = None
    if args.global_path:
        _w = WorldGrid(env_cfg, res=0.05)
        _gm = GlobalMemorySim(_w) if args.global_memory else None
        _f = SlidingMapSim(_w, decay_sec=args.sliding_map_decay,
                        window_m=(args.sliding_map_window if args.sliding_map_window > 0 else None),
                        global_memory=_gm)
        global_path = GlobalPathSim(_w, _f, inflation_radius=args.inflation_radius,
                                    dyn_inflation=(args.dyn_inflation
                                                   if args.dyn_inflation > 0 else None))
        print(f"[global-path] grid {_w.nx}x{_w.ny} occ_ratio {_w.grid.mean():.2f} "
              f"dyn_obs {len(_w.dynamic)} sliding_map_decay={args.sliding_map_decay}s "
              f"inflation={args.inflation_radius}")

    runs = []
    for i in range(args.runs):
        metrics = run_once(env_file, planner_template, overrides,
                           args.max_steps, args.delay_ms,
                           perception_delay_ms=args.perception_delay_ms,
                           chassis_tau=args.chassis_tau,
                           enable_escape=not args.no_escape,
                           goal_xy_target=goal_xy,
                           escape_cfg=escape_cfg,
                           escape_criterion=args.escape_criterion,
                           adaptive_cfg=adaptive_cfg,
                           global_path=global_path,
                           curvature_cfg=curvature_cfg,
                           save_ani=(args.save_ani and i == 0),
                           ani_name=f"{scene_name}__{tag}",
                           path_validity_replan=args.path_validity_replan,
                           path_validity_cooldown_sec=args.path_validity_cooldown,
                           lidar_override=lidar_override,
                           sliding_map_footprint_radius_m=args.sliding_map_footprint_radius)
        metrics["run_index"] = i
        runs.append(metrics)
        print(f"  run#{i}: {metrics['result']} steps={metrics['steps']} "
              f"min_dist={metrics['min_dist']} lat_err={metrics['lat_err_mean']:.3f} "
              f"flips={metrics['omega_flips']} escapes={metrics['escape_count']} "
              f"iters={metrics['iters_mean']} "
              f"compute={metrics['compute_ms_median']}ms")

    agg = aggregate(runs)
    summary = dict(scene=scene_name, overrides=overrides, delay_ms=args.delay_ms,
                   perception_delay_ms=args.perception_delay_ms,
                   chassis_tau=args.chassis_tau,
                   escape_criterion=args.escape_criterion,
                   real_defaults=bool(args.real_defaults),
                   real_lidar=bool(args.real_lidar),
                   lidar=effective_lidar,
                   global_path=bool(args.global_path),
                   global_memory=bool(args.global_memory),
                   path_validity_replan=bool(args.path_validity_replan),
                   path_validity_cooldown=args.path_validity_cooldown,
                   sliding_map_decay=args.sliding_map_decay,
                   sliding_map_window=args.sliding_map_window,
                   sliding_map_footprint_radius=args.sliding_map_footprint_radius,
                   inflation_radius=args.inflation_radius,
                   curvature=curvature_cfg if args.curvature else None,
                   escape_cfg={k: escape_cfg[k] for k in ("rotate_clearance_m", "rotate_speed", "duration_sec", "back_speed", "trigger_frames", "corridor_guard_enable") if k in escape_cfg},
                   aggregate=agg, runs=runs)
    with open(out_dir / "results.jsonl", "w") as f:
        for r in runs:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(f"\n[aggregate] success={agg['success_rate']*100:.0f}% "
          f"collision={agg['collision_rate']*100:.0f}% "
          f"sim_time={agg['sim_time_s_avg']}s lat_err={agg['lat_err_mean_avg']}m "
          f"flips={agg['omega_flips_avg']} compute={agg['compute_ms_median_avg']}ms")
    print(f"[done] Results saved: {out_dir}")


if __name__ == "__main__":
    main()
