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
Platform-independent adaptive ``d_max`` policy.

``d_max`` is the width of the NRMP obstacle penalty band: obstacle points closer
than ``d_max`` push the robot away. A value that is comfortable in the open is
harmful in a tight passage, because both walls sit inside the band at once and
the robot gets squeezed from both sides — it either oscillates or, as measured on
the robot, deadlocks at the entrance with ``min_dist`` pinned at ``d_max``.

This estimates the local corridor half-width from the current obstacle points
and derives a ``d_max`` that fits: ``half_width - robot_half_width - margin``,
clamped to ``[min, max]``.

Design notes:

* **Only side walls may shrink d_max.** The first version took the minimum
  lateral clearance over every point in a forward window, so a wall or a
  pedestrian *straight ahead* (lateral offset ~0) collapsed the estimate to zero
  and pinned ``d_max`` at its floor — the robot then grazed obstacles at 2 cm.
  Frontal blockage is escape/replan territory, not a reason to disarm the
  penalty band, so points within the robot's own width are now excluded.
* **A single stray point must not decide d_max.** sliding_map projections and
  outliers put occasional points very close; the clearance is therefore taken as
  a low percentile instead of the strict minimum.
* **Shrinking is committed slowly, relaxing immediately.** A transient narrow
  estimate should not open the band; consecutive agreeing estimates are required
  before lowering ``d_max``, while raising it back is applied at once.
* **A hard safety net exists.** If the planner keeps reporting a very small
  obstacle distance, the policy snaps ``d_max`` back to the caller's baseline and
  stays there for a cooldown, so a bad estimate can never keep the robot glued
  to a wall.

Like :mod:`neupan.util.stuck_escape`, this lives outside the ROS layer so the
robot node and the simulation benchmark run the exact same policy.
"""

import numpy as np


class AdaptiveDMax:
    """Estimate corridor half-width and derive a fitting ``d_max``."""

    def __init__(self, cfg=None, baseline_d_max=None):
        cfg = cfg or {}
        self.enable = bool(cfg.get("enable", False))
        # Lower bound. It must stay below the tightest passage the robot has to
        # fit through — a floor above the geometric clearance deadlocks that
        # passage. Within that limit, higher is safer.
        self.min = float(cfg.get("min", 0.10))
        self.max = float(cfg.get("max", 0.25))
        self.safety_margin = float(cfg.get("safety_margin", 0.05))
        self.forward_window_m = float(cfg.get("forward_window_m", 2.0))
        self.lateral_window_m = float(cfg.get("lateral_window_m", 2.0))
        self.update_period_sec = float(cfg.get("update_period_sec", 0.5))

        # Points whose lateral offset is within this multiple of the robot's
        # half-width are "straight ahead" and cannot be corridor walls.
        self.frontal_exclude_scale = float(cfg.get("frontal_exclude_scale", 0.8))
        # The forward window is measured in this many depth slices; a corridor
        # shows walls in all of them, a wall across the path in only one.
        self.forward_slices = int(cfg.get("forward_slices", 4))
        # Minimum number of slices that must contain side points for the
        # estimate to be trusted at all.
        self.min_populated_slices = int(cfg.get("min_populated_slices", 2))
        # How the per-slice widths are combined. A low percentile keeps
        # frontal-wall immunity (one narrow slice out of four barely moves it)
        # while reacting to a narrowing seen in half the slices.
        self.slice_percentile = float(cfg.get("slice_percentile", 25.0))
        # Percentile (0-100) of lateral clearance used per side; 0 = strict min.
        self.clearance_percentile = float(cfg.get("clearance_percentile", 10.0))
        # Consecutive agreeing estimates required before lowering d_max.
        self.shrink_confirmations = int(cfg.get("shrink_confirmations", 2))
        # Safety net: obstacle distance below this for that many checks snaps
        # d_max back to the baseline for cooldown_sec.
        # Guard threshold must sit below the smallest distance a legitimate
        # tight passage produces (0.8 m corridor bottoms out near 0.075 m),
        # while still catching the 0.021 m grazing seen on the robot.
        self.guard_min_dist_m = float(cfg.get("guard_min_dist_m", 0.05))
        self.guard_frames = int(cfg.get("guard_frames", 3))
        self.guard_cooldown_sec = float(cfg.get("guard_cooldown_sec", 5.0))

        self.baseline_d_max = (float(baseline_d_max)
                               if baseline_d_max is not None else None)

        self._last_update_t = None
        self._pending_shrink = 0
        self._guard_hits = 0
        self._guard_until_t = None
        self.last_half_width = None
        self.last_d_max = None
        self.guard_trips = 0

    # ------------------------------------------------------------------
    def set_baseline(self, d_max):
        """Remember the configured (non-adaptive) ``d_max`` for the safety net."""
        self.baseline_d_max = float(d_max)

    def due(self, now):
        """True when the estimate is stale enough to refresh."""
        if not self.enable:
            return False
        return (self._last_update_t is None
                or now - self._last_update_t >= self.update_period_sec)

    def guard_active(self, now):
        return self._guard_until_t is not None and now < self._guard_until_t

    def note_min_distance(self, now, min_dist):
        """Feed the planner's reported obstacle distance to the safety net.

        Returns the baseline ``d_max`` exactly once when the net trips, so the
        caller can push it into the planner and log it; ``None`` otherwise.
        """
        if not self.enable or min_dist is None or not np.isfinite(min_dist):
            return None
        if min_dist < self.guard_min_dist_m:
            self._guard_hits += 1
        else:
            self._guard_hits = 0
        if self._guard_hits < self.guard_frames or self.guard_active(now):
            return None
        self._guard_hits = 0
        self._guard_until_t = now + self.guard_cooldown_sec
        self._pending_shrink = 0
        self.guard_trips += 1
        self.last_d_max = self.baseline_d_max
        return self.baseline_d_max

    # ------------------------------------------------------------------
    def _local_points(self, state, points):
        """Obstacle points in the robot frame: row 0 forward, row 1 lateral."""
        if points is None or state is None or np.size(points) == 0:
            return None
        x, y, theta = (float(state[0, 0]), float(state[1, 0]), float(state[2, 0]))
        c, s = np.cos(theta), np.sin(theta)
        rel = np.asarray(points)[:2, :] - np.array([[x], [y]])
        return np.array([[c, s], [-s, c]]) @ rel

    def estimate_half_width(self, state, points, robot_half_width):
        """Corridor half-width from *side* points only, robot frame.

        The forward window is cut into slices and each slice is measured
        separately, then the slices are combined with a median. This is what
        separates a corridor from a wall across the path:

        * a corridor shows side points in **every** slice, so all slices report
          the same narrow clearance and the median is narrow;
        * a wall across the path only produces points in the one slice at its
          own distance (the slices in front of it are empty), so the median
          stays wide and ``d_max`` is left alone.

        Excluding points by lateral offset alone is not enough — a frontal wall
        is wide, and its outer points sit at lateral offsets that look just like
        a corridor wall.

        Returns ``None`` when too few slices contain usable points.
        """
        local = self._local_points(state, points)
        if local is None:
            return None

        in_window = (
            (local[0] >= -self.safety_margin)
            & (local[0] <= self.forward_window_m)
            & (np.abs(local[1]) <= self.lateral_window_m)
        )
        # Points within the robot's own width are directly in the way; they
        # belong to escape/replan, never to a corridor measurement.
        is_side = np.abs(local[1]) > robot_half_width * self.frontal_exclude_scale
        mask = in_window & is_side
        if not np.any(mask):
            return None

        forward = local[0, mask]
        lateral = local[1, mask]
        edges = np.linspace(-self.safety_margin, self.forward_window_m,
                            self.forward_slices + 1)

        slice_widths = []
        populated = 0
        for lo, hi in zip(edges[:-1], edges[1:]):
            in_slice = (forward >= lo) & (forward < hi)
            if not np.any(in_slice):
                # No wall seen at this depth: as far as this slice knows the
                # surroundings are open.
                slice_widths.append(self.lateral_window_m)
                continue
            populated += 1
            vals = lateral[in_slice]
            left = vals[vals > 0]
            right = -vals[vals < 0]
            slice_widths.append(min(self._clearance(left), self._clearance(right)))

        if populated < self.min_populated_slices:
            return None
        return float(np.percentile(slice_widths, self.slice_percentile))

    def _clearance(self, side_vals):
        """Lateral clearance on one side; empty side counts as open."""
        if side_vals.size == 0:
            return self.lateral_window_m
        if self.clearance_percentile <= 0:
            return float(side_vals.min())
        return float(np.percentile(side_vals, self.clearance_percentile))

    def compute(self, state, points, robot_half_width):
        """``(d_max, half_width)``; ``(None, None)`` when not estimable."""
        half_width = self.estimate_half_width(state, points, robot_half_width)
        if half_width is None or not np.isfinite(half_width):
            return None, None
        fitted = half_width - float(robot_half_width) - self.safety_margin
        d_max = float(np.clip(fitted, self.min, self.max))
        return d_max, half_width

    def update(self, now, state, points, robot_half_width, apply_fn):
        """Refresh ``d_max`` through ``apply_fn(d_max)`` when due.

        Returns the applied value, or ``None`` if nothing was applied (not due,
        not estimable, safety net active, or a shrink still awaiting
        confirmation).
        """
        if not self.due(now):
            return None
        self._last_update_t = now

        if self.guard_active(now):
            return None

        d_max, half_width = self.compute(state, points, robot_half_width)
        if d_max is None:
            return None
        self.last_half_width = half_width

        current = self.last_d_max if self.last_d_max is not None else self.max
        if d_max < current - 1e-9:
            # Shrinking narrows the safety band: require agreement first.
            self._pending_shrink += 1
            if self._pending_shrink < self.shrink_confirmations:
                return None
        self._pending_shrink = 0

        self.last_d_max = d_max
        apply_fn(d_max)
        return d_max


def robot_half_width(robot):
    """Best-effort half-width of a NeuPAN robot footprint."""
    if getattr(robot, "shape", None) == "rectangle" and getattr(robot, "width", None):
        return float(robot.width) / 2.0
    verts = getattr(robot, "vertices", None)
    if verts is not None:
        return float((verts[1, :].max() - verts[1, :].min()) / 2.0)
    return 0.2
