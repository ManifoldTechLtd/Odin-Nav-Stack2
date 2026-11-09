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

"""Curvature feedforward reference speed governor shared between simulation and robot nodes.

Computes the maximum path curvature `kappa` within a forward lookahead window
(`lookahead_sec * current_speed`, minimum 1.0m) and scales `ref_speed` to
`clamp(sqrt(a_lat_max / kappa), v_min, ref_speed_base)` with first-order low-pass filtering.
"""

from math import atan2, pi, sqrt

import numpy as np


class CurvatureSpeedGovernor:

    def __init__(self, cfg: dict, ref_speed_base: float):
        cfg = cfg or {}
        self.enable = bool(cfg.get("enable", False))
        self.a_lat_max = float(cfg.get("a_lat_max", 0.5))
        self.v_min = float(cfg.get("v_min", 0.3))
        self.lookahead_sec = float(cfg.get("lookahead_sec", 1.0))
        self.lp_alpha = float(cfg.get("lp_alpha", 0.2))
        self.ref_speed_base = float(ref_speed_base)
        self.ref_speed_eff = self.ref_speed_base
        self.last_kappa = 0.0
        self._path = None

    def set_path(self, path_xy):
        """Install reference path as an (N, 2) array in the robot pose frame."""
        if path_xy is None or len(path_xy) < 3:
            self._path = None
            return
        self._path = np.asarray(path_xy, dtype=float)

    def update(self, mx: float, my: float) -> float:
        """Compute and return the curvature-limited reference speed at (mx, my)."""
        if not self.enable or self._path is None:
            self.ref_speed_eff = self.ref_speed_base
            return self.ref_speed_eff
        pts = self._path
        d = np.hypot(pts[:, 0] - mx, pts[:, 1] - my)
        i0 = int(np.argmin(d))
        v_now = max(self.ref_speed_eff, 0.1)
        look = max(1.0, v_now * self.lookahead_sec)
        kappa_max = 0.0
        arc = 0.0
        seg = 0.0
        head_prev = None
        j = i0
        while j + 1 < len(pts):
            step = float(np.hypot(*(pts[j + 1] - pts[j])))
            arc += step
            if arc > look + 0.5:
                break
            seg += step
            if seg >= 0.3:
                h = atan2(pts[j + 1][1] - pts[j][1], pts[j + 1][0] - pts[j][0])
                if head_prev is not None and seg > 1e-6:
                    dh = abs((h - head_prev + pi) % (2 * pi) - pi)
                    kappa_max = max(kappa_max, dh / seg)
                head_prev = h
                seg = 0.0
            j += 1
        self.last_kappa = kappa_max
        if kappa_max > 1e-3:
            v_target = min(self.ref_speed_base, sqrt(self.a_lat_max / kappa_max))
            v_target = max(self.v_min, v_target)
        else:
            v_target = self.ref_speed_base
        self.ref_speed_eff = (
            (1.0 - self.lp_alpha) * self.ref_speed_eff
            + self.lp_alpha * v_target
        )
        return self.ref_speed_eff
