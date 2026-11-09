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

"""Static ground-band confirmation for fixed-index sensor clouds (no ROS)."""

import numpy as np


class LowFlickerFilter:
    def __init__(self, enabled=False, plane_normal=(0.0, 0.0, 0.0),
                 plane_offset_m=float('nan'), radius_m=1.2,
                 height_min_m=0.05, height_max_m=0.25, min_hits=2,
                 ground_tolerance_m=0.03, max_frame_gap_sec=0.30):
        values = (radius_m, height_min_m, height_max_m,
                  ground_tolerance_m, max_frame_gap_sec)
        if not np.isfinite(values).all():
            raise ValueError('low_flicker thresholds must be finite')
        if (radius_m <= 0 or ground_tolerance_m <= 0
                or height_min_m <= ground_tolerance_m
                or height_max_m <= height_min_m or max_frame_gap_sec < 0):
            raise ValueError('invalid low_flicker radius/height/time thresholds')
        if isinstance(min_hits, bool) or not 2 <= min_hits <= 255 or int(min_hits) != min_hits:
            raise ValueError('low_flicker_min_hits must be an integer in [2, 255]')
        normal = np.asarray(plane_normal, dtype=np.float64)
        self.enabled = bool(enabled)
        if self.enabled:
            if (normal.shape != (3,) or not np.isfinite(normal).all()
                    or normal[2] <= 0 or not np.isfinite(plane_offset_m)):
                raise ValueError('low_flicker requires a calibrated upward plane')
            norm = np.linalg.norm(normal)
            if not np.isfinite(norm) or norm <= 0:
                raise ValueError('low_flicker plane normal must be nonzero')
            normal = normal / norm
            plane_offset_m = float(plane_offset_m) / norm
        self.normal = normal
        self.offset = float(plane_offset_m)
        self.radius_sq = float(radius_m) ** 2
        self.height_min = float(height_min_m)
        self.height_max = float(height_max_m)
        self.min_hits = int(min_hits)
        self.ground_tolerance = float(ground_tolerance_m)
        self.max_gap_ns = int(max_frame_gap_sec * 1e9)
        self.reset()

    def reset(self):
        self._counts = None
        self._previous_ground = None
        self._stamp_ns = None

    def process(self, xyz, stamp_ns=None):
        xyz = np.asarray(xyz)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('xyz must have shape (N, 3)')
        keep = np.ones(len(xyz), dtype=bool)
        stats = dict(candidate_points=0, removed_points=0,
                     confirmed_points=0, warmup=True, reset_reason='')
        if not self.enabled:
            self.reset()
            return keep, stats

        reason = ''
        if self._counts is not None and len(self._counts) != len(xyz):
            reason = 'point_count_change'
        elif stamp_ns is not None and self._stamp_ns is not None:
            delta = int(stamp_ns) - self._stamp_ns
            if delta <= 0:
                reason = 'non_monotonic_stamp'
            elif self.max_gap_ns > 0 and delta > self.max_gap_ns:
                reason = 'frame_gap'
        if reason:
            self.reset()
        warmup = self._counts is None
        if warmup:
            self._counts = np.zeros(len(xyz), dtype=np.uint8)
            self._previous_ground = np.zeros(len(xyz), dtype=bool)

        with np.errstate(invalid='ignore', over='ignore'):
            x, y, z = xyz.T
            height = (x * self.normal[0] + y * self.normal[1]
                      + z * self.normal[2] + self.offset)
            horizontal_sq = x * x + y * y
            valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                     & (horizontal_sq + z * z > 0.05 ** 2))
            band = (valid & (x >= 0) & (horizontal_sq <= self.radius_sq)
                    & (height >= self.height_min) & (height <= self.height_max))
        # Confirm only ground-to-low transitions, matching the offline evaluator.
        active = band & (self._previous_ground | (self._counts > 0))
        counts = np.minimum(self._counts.astype(np.uint16) + 1, self.min_hits)
        self._counts = np.where(active, counts, 0).astype(np.uint8)
        remove = active & (self._counts < self.min_hits)
        keep[remove] = False
        self._previous_ground = valid & (np.abs(height) < self.ground_tolerance)
        self._stamp_ns = None if stamp_ns is None else int(stamp_ns)
        stats.update(candidate_points=int(np.count_nonzero(band)),
                     removed_points=int(np.count_nonzero(remove)),
                     confirmed_points=int(np.count_nonzero(
                         active & (self._counts >= self.min_hits))),
                     warmup=warmup, reset_reason=reason)
        return keep, stats
