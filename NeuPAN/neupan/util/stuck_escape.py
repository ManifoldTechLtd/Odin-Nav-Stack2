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

"""
Platform-independent stuck-escape policy.

This is the single source of truth for the "back up + turn + replan" escape
behaviour. It is deliberately free of ROS and of irsim so that both the real
robot node (``neupan/ros/neupan_ros.py``) and the simulation benchmark
(``example/sw01_bench/benchmark_sw01.py``) can drive the *same* decision code.

Before this module existed the benchmark carried a hand-written simplification
(fixed 0.03 m displacement gate, fixed 5-step phases, no forward-cone check),
so simulation results about escape tuning did not transfer to the robot at all.

Two stall criteria are supported:

``displacement`` (historical behaviour)
    Stalled when the Euclidean displacement over the sliding window drops
    below ``displacement_threshold_m``. Cheap, but a robot that rocks left and
    right on the spot keeps producing displacement, so a shaking-but-not-
    advancing robot is never judged stuck. Raising the threshold to catch that
    case also catches "slow but genuinely progressing", which is why the real
    robot could not use a larger value.

``progress`` (recommended)
    Stalled when the advance *along the reference path* (arc length) over the
    window drops below ``progress_threshold_m``. Rocking in place advances no
    arc length, while slow forward motion does, so the two cases separate.
"""

from collections import deque
from math import pi, sin, cos, radians, degrees

import numpy as np

INF = float("inf")


class EscapeDecision:
    """Result of one policy step.

    Attributes:
        cmd: ``(2, 1)`` array to publish instead of the MPC output, or ``None``
            to let the caller use its own command.
        entered / exited: state-machine edges, for logging by the caller.
        request_replan: the caller should trigger a fresh global plan.
        grace_scales: ``(q_theta_scale, q_s_scale)`` to apply, ``None`` when
            unchanged. ``(1.0, 1.0)`` means "restore the nominal values".
        info: diagnostic values (front_obs, stall metric, count, ...).
    """

    __slots__ = ("cmd", "entered", "exited", "request_replan", "grace_scales", "info")

    def __init__(self, cmd=None, entered=False, exited=False,
                 request_replan=False, grace_scales=None, info=None):
        self.cmd = cmd
        self.entered = entered
        self.exited = exited
        self.request_replan = request_replan
        self.grace_scales = grace_scales
        self.info = info or {}


class StuckEscapePolicy:
    """State machine: idle -> counting -> active (-> post-escape grace)."""

    def __init__(self, cfg=None, criterion=None):
        cfg = cfg or {}
        self.enable = bool(cfg.get("enable", False))
        self.criterion = criterion or cfg.get("criterion", "displacement")
        if self.criterion not in ("displacement", "progress"):
            raise ValueError(f"unknown stall criterion: {self.criterion}")

        self.v_eps = float(cfg.get("v_eps", 0.02))
        self.w_eps = float(cfg.get("w_eps", 0.05))
        self.trigger_frames = int(cfg.get("trigger_frames", 15))
        self.front_obs_threshold = float(cfg.get("front_obs_threshold", 0.6))
        self.cone_half_width = float(cfg.get("cone_half_width_rad", 0.524))
        self.back_speed = float(cfg.get("back_speed", 0.2))
        self.rotate_speed = float(cfg.get("rotate_speed", 0.8))
        # Rotation gate. Turning in place sweeps the robot's circumscribed
        # circle, so it needs that much free space in *every* direction — a
        # criterion that needs no corridor detection and is unaffected by yaw.
        self.rotate_gate_enable = bool(cfg.get("rotate_gate_enable", True))
        # Default = circumradius of the 0.96 x 0.55 m robot (0.553 m) + 5 cm.
        self.rotate_clearance_m = float(cfg.get("rotate_clearance_m", 0.60))
        self.rotation_blocked = False
        self.rotation_block_dist = None
        self.duration = float(cfg.get("duration_sec", 1.5))
        self.exit_obs_distance = float(cfg.get("exit_obs_distance", 1.0))
        self.side_radius = float(cfg.get("side_search_radius_m", 2.0))
        self.min_rotation = float(cfg.get("min_rotation_rad", 1.05))
        # Optional lateral strafe escape mode (replaces reverse + turn with pure lateral velocity vy).
        self.strafe_enable = bool(cfg.get("strafe_enable", False))
        self.strafe_speed = float(cfg.get("strafe_speed", 0.25))
        self.strafe_duration = float(cfg.get("strafe_duration_sec", 3.0))
        self.strafe_direction_sign = float(cfg.get("strafe_direction_sign", 1.0))
        self.strafe_side_margin_m = float(cfg.get("strafe_side_margin_m", 0.10))

        self.window_sec = float(cfg.get("displacement_window_sec", 1.0))
        self.displacement_threshold = float(
            cfg.get("displacement_threshold_m", 0.05)
        )
        # Arc-length advance required over the window in ``progress`` mode.
        self.progress_threshold = float(cfg.get("progress_threshold_m", 0.05))
        # Rotation exemption: exempts progress stall check when fresh odometry shows active yaw rotation.
        self.last_yaw_rate = 0.0
        self._last_yaw_t = None
        self.last_rotation_exempt = False
        # Oscillation and low-speed creep stall detectors
        self.oscillation_detect_enable = bool(cfg.get("oscillation_detect_enable", False))
        self.osc_window_sec = float(cfg.get("osc_window_sec", 4.0))
        self.osc_net_disp_m = float(cfg.get("osc_net_disp_m", 0.30))
        self.osc_travel_m = float(cfg.get("osc_travel_m", 1.0))
        self.oscillation_active = False
        self.creep_detect_enable = bool(cfg.get("creep_detect_enable", False))
        self.creep_v_eps = float(cfg.get("creep_v_eps", 0.10))
        self.creep_timeout_sec = float(cfg.get("creep_timeout_sec", 1.5))
        self._motion_history = deque(maxlen=300)
        self._creep_since = None
        self.creep_active = False
        # Bounded corridor rotation and sweep clearance checks
        self.rotate_bounded_enable = bool(cfg.get("rotate_bounded_enable", False))
        self.rotate_sweep_enable = bool(cfg.get("rotate_sweep_check_enable", False))
        self.rotate_sweep_margin_m = float(cfg.get("rotate_sweep_margin_m", 0.05))
        self.rotate_sweep_step_deg = float(cfg.get("rotate_sweep_step_deg", 15.0))
        self._sweep_cap = None
        self.front_path_bypass_enable = bool(cfg.get("front_path_bypass_enable", False))
        self.front_path_clearance_m = float(cfg.get("front_path_clearance_m", 0.40))
        self.front_path_bypassed = False
        self.rotate_open_enable = bool(cfg.get("rotate_open_enable", False))
        self.rotate_open_clearance_m = float(cfg.get("rotate_open_clearance_m", 0.57))
        self.open_rot_active = False
        self.rotate_bounded_margin_m = float(cfg.get("rotate_bounded_margin_m", 0.05))
        self.rotate_bounded_half_len = float(cfg.get("rotate_bounded_half_len", 0.48))   # SW01 L=0.96/2
        self.rotate_bounded_half_wid = float(cfg.get("rotate_bounded_half_wid", 0.275))  # SW01 W=0.55/2
        self.bounded_rot_active = False
        self.theta_max_deg = None
        self.last_half_width = None
        self._bounded_required = 0.0
        # "Blockage is clearing" guard: when the forward clearance is growing,
        # the robot is yielding to something that is moving out of the way
        # (a pedestrian crossing, a door swinging open).
        self.clearing_guard_enable = bool(cfg.get("clearing_guard_enable", True))
        self.clearing_guard_delta_m = float(cfg.get("clearing_guard_delta_m", 0.05))
        # "Inside a corridor" guard: recognized by near-symmetric walls on both sides;
        # while in a corridor, escape only fires if the robot is truly motionless.
        self.corridor_guard_enable = bool(cfg.get("corridor_guard_enable", True))
        self.corridor_side_max_m = float(cfg.get("corridor_side_max_m", 1.2))
        self.corridor_forward_window_m = float(
            cfg.get("corridor_forward_window_m", 2.0)
        )
        self.corridor_slices = int(cfg.get("corridor_slices", 4))
        self.corridor_min_slices = int(cfg.get("corridor_min_slices", 2))
        self.corridor_symmetry_ratio = float(cfg.get("corridor_symmetry_ratio", 2.0))
        self.corridor_standstill_disp_m = float(
            cfg.get("corridor_standstill_disp_m", 0.02)
        )
        self.corridor_standstill_sec = float(cfg.get("corridor_standstill_sec", 3.0))
        self.corridor_fast_escape_enable = bool(cfg.get("corridor_fast_escape_enable", False))
        self.corridor_fast_standstill_sec = float(cfg.get("corridor_fast_standstill_sec", 0.5))

        self.limit_cycle_n = int(cfg.get("limit_cycle_consecutive_n", 3))
        self.limit_cycle_sec = float(cfg.get("limit_cycle_window_sec", 10.0))

        self.early_replan_enable = bool(cfg.get("early_replan_enable", True))
        self.early_replan_trigger_frames = int(
            cfg.get("early_replan_trigger_frames", 5)
        )
        self.early_replan_cooldown = float(
            cfg.get("early_replan_cooldown_sec", 2.0)
        )

        self.grace_max = float(cfg.get("post_escape_grace_max_sec", 5.0))
        self.grace_clear_threshold = float(
            cfg.get("post_escape_grace_clear_threshold_m", 1.5)
        )
        self.grace_clear_dwell = float(
            cfg.get("post_escape_grace_clear_dwell_sec", 0.5)
        )
        self.grace_q_theta_scale = float(
            cfg.get("post_escape_grace_q_theta_scale", 0.25)
        )
        self.grace_q_s_scale = float(cfg.get("post_escape_grace_q_s_scale", 0.5))
        self.grace_min_distance = float(
            cfg.get("post_escape_grace_min_distance_m", 1.0)
        )

        # Path-block grace: scales q_theta/q_s when the reference path is blocked by live obstacle points
        self.pb_enable = bool(cfg.get("path_block_grace_enable", False))
        self.pb_dist = float(cfg.get("path_block_dist_m", 0.45))
        self.pb_frames = int(cfg.get("path_block_frames", 5))
        self.pb_clear_frames = int(cfg.get("path_block_clear_frames", 10))

        # ---- runtime state ----
        self.pose_history = deque(maxlen=400)      # (t, x, y)
        self.front_obs_history = deque(maxlen=400)  # (t, front clearance)
        self._corridor_standstill_since = None
        self.in_corridor = False
        self.progress_history = deque(maxlen=400)  # (t, arc_length)
        self.count = 0
        self.active = False
        self.start_time = None
        self.start_yaw = None
        self.direction = 1.0
        self.dir_history = deque(maxlen=8)         # (t, dir)
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.has_cmd = False
        self.last_early_replan_t = -INF
        self.escape_count = 0

        self.grace_active = False
        self.grace_start_t = None
        self.grace_start_xy = None
        self.grace_clear_since = None

        self.pb_active = False
        self.pb_count = 0
        self.pb_clear_count = 0

        self._path_xy = None       # (2, n) reference path samples
        self._path_s = None        # (n,) cumulative arc length

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def note_cmd(self, v, w):
        """Record the command that was actually published this cycle."""
        self.last_cmd_v = float(v)
        self.last_cmd_w = float(w)
        self.has_cmd = True

    def note_rotation(self, t, yaw_rate):
        """Record actual chassis yaw rate (from odometry) for rotation exemption."""
        self.last_yaw_rate = float(yaw_rate)
        self._last_yaw_t = float(t)

    def note_motion(self, t, forward_v):
        """Record actual forward velocity (from odometry or simulation)."""
        self._motion_history.append((float(t), float(forward_v)))

    def creep_stuck(self, now):
        """Return True when forward speed stays below `creep_v_eps` for `creep_timeout_sec`."""
        if not self.creep_detect_enable or not self._motion_history:
            self._creep_since = None
            return False
        t, v = self._motion_history[-1]
        if now - t > 0.3 or abs(v) >= self.creep_v_eps:
            self._creep_since = None
            return False
        if self._creep_since is None:
            self._creep_since = now
        return (now - self._creep_since) >= self.creep_timeout_sec

    def oscillating(self, now):
        """Return True when net displacement over `osc_window_sec` is below `osc_net_disp_m` while cumulative travel exceeds `osc_travel_m`."""
        if not self.oscillation_detect_enable or len(self.pose_history) < 8:
            return False
        t0 = now - self.osc_window_sec
        pts = [(x, y) for (t, x, y) in self.pose_history if t >= t0]
        if len(pts) < 8 or (now - self.pose_history[-1][0]) > 0.3:
            return False
        arr = np.asarray(pts)
        net = float(np.hypot(arr[-1, 0] - arr[0, 0], arr[-1, 1] - arr[0, 1]))
        travel = float(np.sum(np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))))
        return net < self.osc_net_disp_m and travel > self.osc_travel_m

    def rotation_exempt(self, t):
        """True when the chassis is genuinely rotating on fresh odometry."""
        if self._last_yaw_t is None:
            return False
        if float(t) - self._last_yaw_t > 0.3:
            return False
        return abs(self.last_yaw_rate) >= self.w_eps

    def corridor_half_width_open(self, state, points, exclude_lat=0.25):
        """Estimate corridor half-width for bounded rotation, excluding points with `|lat| < exclude_lat`."""
        if state is None or points is None or np.size(points) == 0:
            return None
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        c, s_ = np.cos(theta), np.sin(theta)
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        local = np.array([[c, s_], [-s_, c]]) @ rel
        fwd, lat = local[0], local[1]
        in_win = ((fwd >= 0.0) & (fwd <= self.corridor_forward_window_m)
                  & (np.abs(lat) <= self.corridor_side_max_m))
        if not np.any(in_win):
            return None
        fwd, lat = fwd[in_win], lat[in_win]
        edges = np.linspace(0.0, self.corridor_forward_window_m, self.corridor_slices + 1)
        l_sl = r_sl = 0
        l_min = r_min = INF
        for lo, hi in zip(edges[:-1], edges[1:]):
            sl = (fwd >= lo) & (fwd < hi)
            l = lat[sl & (lat > exclude_lat)]
            r = -lat[sl & (lat < -exclude_lat)]
            if l.size:
                l_sl += 1
                l_min = min(l_min, float(l.min()))
            if r.size:
                r_sl += 1
                r_min = min(r_min, float(r.min()))
        if l_sl < self.corridor_min_slices or r_sl < self.corridor_min_slices:
            return None
        if not (np.isfinite(l_min) and np.isfinite(r_min)):
            return None
        return min(l_min, r_min)

    def bounded_theta_max(self, half_width):
        """Compute maximum safe heading angle (rad) inside a corridor of `half_width`."""
        budget = half_width - self.rotate_bounded_margin_m
        if budget <= self.rotate_bounded_half_wid:
            return 0.0
        best = 0.0
        for deg in range(5, 95, 5):
            th = radians(deg)
            f = (self.rotate_bounded_half_len * abs(sin(th))
                 + self.rotate_bounded_half_wid * abs(cos(th)))
            if f <= budget:
                best = th
            else:
                break
        return best

    def sweep_max_angle(self, state, points, direction, theta_limit):
        """Compute maximum continuous rotation angle (rad) along `direction` keeping all four footprint corners at least `rotate_sweep_margin_m` from obstacle points."""
        if points is None or np.size(points) == 0 or theta_limit <= 0:
            return max(theta_limit, 0.0)
        x, y, yaw0 = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        d2 = (rel * rel).sum(axis=0)
        rr = (self.rotate_bounded_half_len ** 2 + self.rotate_bounded_half_wid ** 2) ** 0.5
        near = rel[:, d2 < (rr + self.rotate_sweep_margin_m + 0.25) ** 2]
        if near.shape[1] == 0:
            return theta_limit
        hl, hw = self.rotate_bounded_half_len, self.rotate_bounded_half_wid
        corners = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
        step = radians(self.rotate_sweep_step_deg)
        best = 0.0
        th = step
        while th <= theta_limit + 1e-9:
            a = yaw0 + direction * th
            ca, sa = cos(a), sin(a)
            clear = True
            for (cx, cy) in corners:
                wx, wy = ca * cx - sa * cy, sa * cx + ca * cy
                if float(np.min(np.hypot(near[0] - wx, near[1] - wy))) < self.rotate_sweep_margin_m:
                    clear = False
                    break
            if not clear:
                break
            best = th
            th += step
        return best

    def note_pose(self, t, x, y):
        self.pose_history.append((float(t), float(x), float(y)))

    def set_reference_path(self, path):
        """Install the reference path used by the ``progress`` criterion.

        ``path`` may be a list of ``(3, 1)``/``(2, 1)`` arrays or an ``(n, 2)``
        array. Passing a new path clears the progress history, because arc
        length is only comparable within one path.
        """
        xy = self._as_xy(path)
        self._path_xy = xy
        if xy is None:
            self._path_s = None
        else:
            seg = np.linalg.norm(np.diff(xy, axis=1), axis=0)
            self._path_s = np.concatenate([[0.0], np.cumsum(seg)])
        self.progress_history.clear()
        self.front_obs_history.clear()

    @staticmethod
    def _as_xy(path):
        if path is None:
            return None
        if isinstance(path, np.ndarray):
            arr = path
            if arr.ndim != 2 or arr.size == 0:
                return None
            xy = arr[:2, :] if arr.shape[0] in (2, 3) else arr[:, :2].T
        else:
            pts = [np.asarray(p, dtype=float).flatten()[:2] for p in path]
            if len(pts) < 2:
                return None
            xy = np.asarray(pts, dtype=float).T
        return xy if xy.shape[1] >= 2 else None

    def note_progress(self, t, x, y):
        """Project the pose onto the reference path and store its arc length."""
        if self._path_xy is None or self._path_s is None:
            return
        d = np.linalg.norm(self._path_xy - np.array([[x], [y]]), axis=0)
        self.progress_history.append((float(t), float(self._path_s[int(np.argmin(d))])))

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------
    def _window_delta(self, history, signed):
        """Change of a quantity across the sliding window.

        ``None`` until the history spans (most of) the window, so a robot that
        just started moving is not judged prematurely.
        """
        if not history:
            return None
        t_now = history[-1][0]
        cutoff = t_now - self.window_sec
        earliest = None
        for entry in history:
            if entry[0] >= cutoff:
                earliest = entry
                break
        if earliest is None or (t_now - earliest[0]) < 0.8 * self.window_sec:
            return None
        if signed:
            return float(history[-1][1] - earliest[1])
        _, x0, y0 = earliest
        _, x1, y1 = history[-1]
        return float(((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5)

    def displacement_in_window(self):
        return self._window_delta(self.pose_history, signed=False)

    def progress_in_window(self):
        return self._window_delta(self.progress_history, signed=True)

    def min_obs_distance_in_cone(self, state, points, half_width=None):
        """Nearest obstacle inside the forward cone, in the robot frame."""
        if points is None or state is None or np.size(points) == 0:
            return INF
        half_width = self.cone_half_width if half_width is None else half_width
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        ang = np.arctan2(rel[1, :], rel[0, :]) - theta
        ang = (ang + pi) % (2 * pi) - pi
        in_cone = np.abs(ang) < half_width
        if not np.any(in_cone):
            return INF
        return float(np.linalg.norm(rel[:, in_cone], axis=0).min())

    def nearest_obstacle_distance(self, state, points):
        """Distance from the robot centre to the nearest point, all directions.

        Rotation sweeps the whole circumscribed circle, so unlike the forward
        cone this must not be restricted to any sector.
        """
        if points is None or state is None or np.size(points) == 0:
            return INF
        rel = np.asarray(points)[:2, :] - np.array(
            [[float(state[0, 0])], [float(state[1, 0])]]
        )
        return float(np.linalg.norm(rel, axis=0).min())

    def rotation_allowed(self, state, points):
        """``(allowed, nearest_distance)`` for turning in place right here."""
        if not self.rotate_gate_enable:
            return True, None
        nearest = self.nearest_obstacle_distance(state, points)
        return nearest >= self.rotate_clearance_m, nearest

    def side_clearance(self, state, points, side):
        """``(min_distance, count)`` in the forward half-cone on one side."""
        if points is None or state is None or np.size(points) == 0:
            return INF, 0
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        ang = np.arctan2(rel[1, :], rel[0, :]) - theta
        ang = (ang + pi) % (2 * pi) - pi
        if side > 0:
            angle_mask = (ang > 0) & (ang < self.cone_half_width)
        else:
            angle_mask = (ang < 0) & (ang > -self.cone_half_width)
        dist = np.linalg.norm(rel, axis=0)
        mask = angle_mask & (dist < self.side_radius)
        count = int(np.sum(mask))
        if count == 0:
            return INF, 0
        return float(np.min(dist[mask])), count

    def strafe_side_clearance(self, state, points, direction):
        """Return lateral clearance margin (m) beyond robot half-width in the strafe direction (`None` if no points in band)."""
        if state is None or points is None or np.size(points) == 0:
            return None
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        c, s_ = np.cos(theta), np.sin(theta)
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        local = np.array([[c, s_], [-s_, c]]) @ rel  # row 0 forward, row 1 lateral
        fwd, lat = local[0], local[1]
        band = ((np.abs(fwd) <= self.rotate_bounded_half_len + 0.10)
                & (lat * direction > 0.0))
        if not np.any(band):
            return None
        return float(np.min(np.abs(lat[band])) - self.rotate_bounded_half_wid)

    def front_obs_trend(self):
        """Change of forward clearance across the window (``None`` if unknown).

        Positive means the way ahead is opening up.
        """
        return self._window_delta(self.front_obs_history, signed=True)

    def blockage_clearing(self):
        """True when the forward clearance is growing enough to just wait."""
        if not self.clearing_guard_enable:
            return False
        trend = self.front_obs_trend()
        return trend is not None and trend > self.clearing_guard_delta_m

    def in_corridor_now(self, state, points):
        """True when walls run **alongside** the robot on both sides.

        Detected by depth slicing, the same way
        :mod:`neupan.util.adaptive_d_max` measures corridors: a corridor wall
        appears in several forward slices, while a wall across the path only
        occupies the single slice at its own distance. Checking left/right
        clearance alone is not enough — a frontal wall is symmetric too, and
        would be mistaken for a corridor (which would disarm escape exactly
        when it is needed).
        """
        self.last_half_width = None
        if not self.corridor_guard_enable or state is None:
            return False
        if points is None or np.size(points) == 0:
            return False

        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        c, s_ = np.cos(theta), np.sin(theta)
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        local = np.array([[c, s_], [-s_, c]]) @ rel  # row 0 forward, row 1 lateral

        in_window = (
            (local[0] >= 0.0)
            & (local[0] <= self.corridor_forward_window_m)
            & (np.abs(local[1]) <= self.corridor_side_max_m)
        )
        if not np.any(in_window):
            return False

        forward = local[0, in_window]
        lateral = local[1, in_window]
        edges = np.linspace(0.0, self.corridor_forward_window_m,
                            self.corridor_slices + 1)

        left_slices = right_slices = 0
        left_min = right_min = INF
        for lo, hi in zip(edges[:-1], edges[1:]):
            sl = (forward >= lo) & (forward < hi)
            if not np.any(sl):
                continue
            vals = lateral[sl]
            left = vals[vals > 0]
            right = -vals[vals < 0]
            if left.size:
                left_slices += 1
                left_min = min(left_min, float(left.min()))
            if right.size:
                right_slices += 1
                right_min = min(right_min, float(right.min()))

        if np.isfinite(left_min) and np.isfinite(right_min):
            self.last_half_width = min(left_min, right_min)
        if left_slices < self.corridor_min_slices or right_slices < self.corridor_min_slices:
            return False
        if not (np.isfinite(left_min) and np.isfinite(right_min)):
            return False
        near, far = sorted((left_min, right_min))
        if near <= 1e-6:
            return True
        return (far / near) <= self.corridor_symmetry_ratio

    def _corridor_truly_stuck(self, now):
        """Inside a corridor, only a sustained standstill counts as stuck."""
        disp = self.displacement_in_window()
        if disp is None or disp >= self.corridor_standstill_disp_m:
            self._corridor_standstill_since = None
            return False
        if self._corridor_standstill_since is None:
            self._corridor_standstill_since = now
        required_sec = (self.corridor_fast_standstill_sec
                        if self.corridor_fast_escape_enable
                        else self.corridor_standstill_sec)
        return (now - self._corridor_standstill_since) >= required_sec

    def is_stalled(self, now=None):
        """``(stalled, metric_value, metric_name)`` for the active criterion.

        In ``displacement`` mode a low command counts as stalled on its own,
        which can flag a genuinely-but-slowly advancing robot in a narrow corridor.

        In ``progress`` mode the arc-length advance is the authority: a small
        command that still moves the robot along the path is not stalled, while
        rocking in place is, no matter how large the command or the raw
        displacement. ``cmd_low`` is only consulted as a fallback while no
        progress measurement is available yet.
        """
        if now is None:
            now = self._last_yaw_t if self._last_yaw_t is not None else 0.0
        cmd_low = (
            self.has_cmd
            and abs(self.last_cmd_v) < self.v_eps
            and abs(self.last_cmd_w) < self.w_eps
        )
        if self.criterion == "progress":
            value = self.progress_in_window()
            if value is not None:
                stalled = value < self.progress_threshold
                self.last_rotation_exempt = stalled and self.rotation_exempt(now)
                if self.last_rotation_exempt:
                    return False, value, "progress"
                return stalled, value, "progress"
            if self._path_xy is not None:
                # A path is installed but its window is not full yet (the first
                # ~0.8 s after start or after a replan). Wait for it instead of
                # falling back to cmd_low, which would false-positive on a
                # slow-but-advancing robot exactly during that warm-up.
                return False, None, "progress"
            # No reference path at all: behave like displacement mode.
            disp = self.displacement_in_window()
            stalled = (disp is not None and disp < self.displacement_threshold)
            return (stalled or cmd_low), disp, "disp"
        value = self.displacement_in_window()
        stalled = value is not None and value < self.displacement_threshold
        return (stalled or cmd_low), value, "disp"

    def front_obstacle_blocks_reference_path(self, state, points):
        """Return True when close forward-cone obstacle points lie within `front_path_clearance_m` of the reference path."""
        if self._path_xy is None or len(self._path_xy) < 2:
            return True
        if state is None or points is None or np.size(points) == 0:
            return True
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        ang = np.arctan2(rel[1, :], rel[0, :]) - theta
        ang = (ang + pi) % (2 * pi) - pi
        dist = np.linalg.norm(rel, axis=0)
        mask = (np.abs(ang) <= self.cone_half_width) & (dist < self.front_obs_threshold)
        pts = np.asarray(points)[:2, mask].T
        if len(pts) == 0:
            return True
        path = self._path_xy
        ax, ay = path[:-1, 0], path[:-1, 1]
        dx, dy = path[1:, 0] - ax, path[1:, 1] - ay
        L2 = dx * dx + dy * dy
        L2[L2 < 1e-9] = 1e-9
        for px, py in pts:
            u = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0, 1)
            if float(np.min(np.hypot(px - (ax + u * dx), py - (ay + u * dy)))) < self.front_path_clearance_m:
                return True
        return False

    # ------------------------------------------------------------------
    # Main entry
    # ------------------------------------------------------------------
    def decide(self, now, state, points, preferred_direction=None,
               require_preferred_direction=False,
               preferred_min_clearance=0.57):
        """Advance the state machine once. See :class:`EscapeDecision`."""
        if not self.enable or state is None:
            return EscapeDecision()

        front_obs = self.min_obs_distance_in_cone(state, points)
        if np.isfinite(front_obs):
            self.front_obs_history.append((float(now), float(front_obs)))

        # Re-evaluated every frame: the robot may back out into open space and
        # then be free to turn.
        allowed, nearest = self.rotation_allowed(state, points)
        self.rotation_blocked = not allowed
        self.rotation_block_dist = nearest

        self._sweep_cap = None
        if (self.rotate_sweep_enable and self.active and self.direction
                and points is not None and np.size(points) > 0):
            self._sweep_cap = self.sweep_max_angle(
                state, points, self.direction, self.min_rotation)
            if self._sweep_cap < radians(5.0) and not self.rotation_blocked:
                self.rotation_blocked = True

        self.bounded_rot_active = False
        hw_open = (self.corridor_half_width_open(state, points)
                   if (self.rotation_blocked and self.active
                       and (self.rotate_bounded_enable or self.rotate_open_enable))
                   else None)
        self.open_rot_active = False
        if hw_open is not None and self.rotate_bounded_enable:
            hw_eff = hw_open
            if self.rotation_block_dist is not None:
                hw_eff = min(hw_open, self.rotation_block_dist)
            th = self.bounded_theta_max(hw_eff)
            if self._sweep_cap is not None:
                th = min(th, self._sweep_cap)
            self.theta_max_deg = round(degrees(th), 1)
            if th >= radians(5.0):
                done = 0.0 if self.start_yaw is None else abs(
                    (float(state[2, 0]) - self.start_yaw + pi) % (2 * pi) - pi)
                if done < th:
                    self.bounded_rot_active = True
                self._bounded_required = th
        elif hw_open is None and self.rotate_open_enable:
            if (self.rotation_block_dist is not None
                    and self.rotation_block_dist >= self.rotate_open_clearance_m
                    and (self._sweep_cap is None
                         or self._sweep_cap >= radians(5.0))):
                self.open_rot_active = True

        if self.active:
            strafe_side_min = None
            if (self.strafe_enable and points is not None
                    and np.size(points) > 0 and self.direction):
                strafe_side_min = self.strafe_side_clearance(
                    state, points, self.direction)
            return self._step_active(now, state, front_obs, strafe_side_min)

        if not self.has_cmd:
            return EscapeDecision(info={
                "front_obs": front_obs,
                "nearest_obs": nearest,
                "rotation_blocked": self.rotation_blocked,
            })

        stalled, metric, metric_name = self.is_stalled(now)
        has_close_front = front_obs < self.front_obs_threshold
        self.front_path_bypassed = False
        if (has_close_front and self.front_path_bypass_enable
                and not self.front_obstacle_blocks_reference_path(state, points)):
            has_close_front = False
            self.front_path_bypassed = True

        # In a corridor, "slow" is the correct behaviour, not a stall: hold the
        # count unless the robot has genuinely stopped for several seconds.
        self.in_corridor = self.in_corridor_now(state, points)
        corridor_hold = False
        if self.in_corridor and not self._corridor_truly_stuck(now):
            stalled = False
            corridor_hold = True

        clearing = self.blockage_clearing()
        creep = (not self.in_corridor and has_close_front and self.creep_stuck(now))
        self.creep_active = creep
        osc = (not self.in_corridor and has_close_front and self.oscillating(now))
        self.oscillation_active = osc
        if clearing and not (creep or osc):
            stalled = False
        if creep or osc:
            stalled = True
        if stalled and has_close_front:
            self.count += 1
        else:
            self.count = max(0, self.count - 2)

        info = {
            "front_obs": front_obs,
            "front_path_bypassed": self.front_path_bypassed,
            "metric": metric,
            "metric_name": metric_name,
            "stalled": stalled,
            "clearing": clearing,
            "creep_active": self.creep_active,
            "oscillation_active": self.oscillation_active,
            "in_corridor": self.in_corridor,
            "corridor_hold": corridor_hold,
            "count": self.count,
            "rotation_blocked": self.rotation_blocked,
            "bounded_rot": self.bounded_rot_active,
            "open_rot": self.open_rot_active,
            "sweep_cap_deg": (round(degrees(self._sweep_cap), 1)
                              if self._sweep_cap is not None else None),
            "theta_max_deg": self.theta_max_deg,
            "half_width": hw_open if hw_open is not None else self.last_half_width,
            "nearest_obs": self.rotation_block_dist,
        }

        request_replan = False
        if (self.early_replan_enable
                and self.count >= self.early_replan_trigger_frames
                and now - self.last_early_replan_t >= self.early_replan_cooldown):
            self.last_early_replan_t = now
            request_replan = True

        if self.count < self.trigger_frames:
            return EscapeDecision(request_replan=request_replan, info=info)

        forced_direction = None
        if require_preferred_direction:
            if preferred_direction not in (-1, -1.0, 1, 1.0):
                info.update(recovery_wait=True,
                            recovery_wait_reason="candidate_unavailable")
                return EscapeDecision(cmd=np.zeros((2, 1)), info=info)
            side_min, side_count = self.side_clearance(
                state, points, preferred_direction
            )
            info.update(candidate_side=int(preferred_direction),
                        candidate_side_min=side_min,
                        candidate_side_count=side_count)
            if side_min < preferred_min_clearance:
                info.update(recovery_wait=True,
                            recovery_wait_reason="candidate_side_unsafe")
                return EscapeDecision(cmd=np.zeros((2, 1)), info=info)
            forced_direction = float(preferred_direction)

        return self._enter(now, state, points, front_obs, info, request_replan,
                           forced_direction)

    def _enter(self, now, state, points, front_obs, info, request_replan,
               forced_direction=None):
        left_min, left_count = self.side_clearance(state, points, +1)
        right_min, right_count = self.side_clearance(state, points, -1)
        natural_dir = 1.0 if left_min >= right_min else -1.0

        recent = [d for (t, d) in self.dir_history
                  if now - t <= self.limit_cycle_sec]
        forced = False
        if forced_direction is not None:
            self.direction = forced_direction
        else:
            forced = (
                len(recent) >= self.limit_cycle_n
                and all(d == natural_dir for d in recent[-self.limit_cycle_n:])
            )
            self.direction = -natural_dir if forced else natural_dir
        self.dir_history.append((now, self.direction))

        self.active = True
        self.start_time = now
        self.start_yaw = float(state[2, 0])
        self._corridor_standstill_since = None
        self.escape_count += 1
        info.update(left_min=left_min, left_count=left_count,
                    right_min=right_min, right_count=right_count,
                    direction=self.direction, limit_cycle_flip=forced,
                    candidate_forced=forced_direction is not None,
                    escape_count=self.escape_count,
                    strafe=self.strafe_enable)
        return EscapeDecision(
            cmd=self._escape_cmd(), entered=True,
            request_replan=request_replan, info=info,
        )

    def _escape_cmd(self):
        """Reverse command; the turn is dropped when rotation is gated off."""
        if self.strafe_enable:
            vy = self.strafe_direction_sign * self.direction * self.strafe_speed
            return np.array([[0.0], [0.0], [vy]], dtype=float)
        omega = (0.0 if (self.rotation_blocked
                         and not self.bounded_rot_active
                         and not self.open_rot_active)
                 else self.direction * self.rotate_speed)
        return np.array([[-self.back_speed], [omega]], dtype=float)

    def _step_active(self, now, state, front_obs, strafe_side_min=None):
        elapsed = now - (self.start_time if self.start_time is not None else now)
        if self.start_yaw is None:
            rotation_done = 0.0
        else:
            dyaw = float(state[2, 0]) - self.start_yaw
            rotation_done = abs((dyaw + pi) % (2 * pi) - pi)

        strafe_side_blocked = (
            strafe_side_min is not None
            and strafe_side_min < self.strafe_side_margin_m
        )
        time_up = (elapsed > (self.strafe_duration if self.strafe_enable
                              else self.duration)
                   or strafe_side_blocked)
        if self.strafe_enable:
            required_rotation = 0.0
        elif self.open_rot_active:
            required_rotation = self.min_rotation
            if self._sweep_cap is not None:
                required_rotation = min(required_rotation, self._sweep_cap)
        elif self.rotation_blocked:
            required_rotation = (self._bounded_required
                                 if (self.rotate_bounded_enable and self._bounded_required > 0)
                                 else 0.0)
        else:
            required_rotation = self.min_rotation
            if self._sweep_cap is not None:
                required_rotation = min(required_rotation, self._sweep_cap)
        cleared = (
            front_obs > self.exit_obs_distance
            and rotation_done >= required_rotation
        )
        info = {
            "front_obs": front_obs,
            "elapsed": elapsed,
            "rotation_done": rotation_done,
            "time_up": time_up,
            "cleared": cleared,
            "direction": self.direction,
            "strafe": self.strafe_enable,
            "strafe_side_min": strafe_side_min,
            "strafe_side_blocked": strafe_side_blocked,
            "rotation_blocked": self.rotation_blocked,
            "bounded_rot": self.bounded_rot_active,
            "open_rot": self.open_rot_active,
            "theta_max_deg": self.theta_max_deg,
            "nearest_obs": self.rotation_block_dist,
        }
        if not (time_up or cleared):
            return EscapeDecision(cmd=self._escape_cmd(), info=info)

        # ---- exit: hand control back and open the grace window ----
        self.active = False
        self.start_time = None
        self.start_yaw = None
        self.count = 0
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.grace_active = True
        self.grace_start_t = now
        self.grace_start_xy = (float(state[0, 0]), float(state[1, 0]))
        self.grace_clear_since = None
        return EscapeDecision(
            exited=True, request_replan=True,
            grace_scales=(self.grace_q_theta_scale, self.grace_q_s_scale),
            info=info,
        )

    def update_grace(self, now, state, points):
        """Call once per cycle while a grace window may be open.

        Returns ``(1.0, 1.0)`` exactly once, when the nominal weights should be
        restored; ``None`` otherwise.
        """
        if not self.grace_active or state is None:
            return None
        front_obs = self.min_obs_distance_in_cone(state, points)
        if front_obs > self.grace_clear_threshold:
            if self.grace_clear_since is None:
                self.grace_clear_since = now
            clear_for = now - self.grace_clear_since
        else:
            self.grace_clear_since = None
            clear_for = 0.0

        if self.grace_start_xy is not None:
            sx, sy = self.grace_start_xy
            moved = ((float(state[0, 0]) - sx) ** 2
                     + (float(state[1, 0]) - sy) ** 2) ** 0.5
        else:
            moved = 0.0

        elapsed = now - (self.grace_start_t if self.grace_start_t is not None else now)
        cleared = (moved >= self.grace_min_distance
                   and clear_for >= self.grace_clear_dwell)
        if not (elapsed >= self.grace_max or cleared):
            return None

        self.grace_active = False
        self.grace_start_t = None
        self.grace_start_xy = None
        self.grace_clear_since = None
        return (1.0, 1.0)

    def note_path_block(self, blocked):
        """Update hysteresis state for path-block grace (`pb_frames` to enter, `pb_clear_frames` to exit)."""
        if not self.pb_enable:
            self.pb_active = False
            return False
        if blocked:
            self.pb_count += 1
            self.pb_clear_count = 0
            if self.pb_count >= self.pb_frames:
                self.pb_active = True
        else:
            self.pb_count = 0
            self.pb_clear_count += 1
            if self.pb_clear_count >= self.pb_clear_frames:
                self.pb_active = False
        return self.pb_active
