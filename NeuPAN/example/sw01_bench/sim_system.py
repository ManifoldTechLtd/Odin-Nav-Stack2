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

"""Simulation system layer reproducing the real-robot navigation pipeline in sw01_bench:
  - WorldGrid: static occupancy grid rasterized from irsim env.yaml
  - SlidingMapSim: dynamic obstacle occupancy memory with cell_decay_sec decay and sliding window
  - GlobalMemorySim: coarse persistent global obstacle memory
  - astar_plan / GlobalPathSim: A* global planning with inflation, line-of-sight smoothing, and densification
"""

import heapq
import math

import numpy as np
import yaml


class WorldGrid:
    """Rasterize static obstacles from irsim env.yaml into an occupancy grid."""

    def __init__(self, env_cfg: dict, res: float = 0.05):
        world = env_cfg["world"]
        self.res = res
        off = world.get("offset", [0, 0]) or [0, 0]
        self.x0, self.y0 = float(off[0]), float(off[1])
        self.nx = int(math.ceil(float(world["width"]) / res))
        self.ny = int(math.ceil(float(world["height"]) / res))
        self.grid = np.zeros((self.nx, self.ny), dtype=bool)
        self.dynamic = []
        self.unknown = []
        for ob in env_cfg.get("obstacle", []) or []:
            if ob.get("exclude_from_static"):
                for sh, st in zip(ob.get("shape", []), ob.get("state", [])):
                    self.unknown.append((sh, st))
                continue
            is_dyn = bool(ob.get("behavior"))
            shapes = ob.get("shape", [])
            if isinstance(shapes, dict):
                shapes = [shapes]
            states = ob.get("state", [])
            if is_dyn:
                r = 0.3
                for sh in shapes:
                    if sh.get("name") == "circle":
                        r = float(sh.get("radius", 0.3))
                self.dynamic.append(r)
                continue
            for sh, st in zip(shapes, states if len(states) == len(shapes) else states[:1] * len(shapes)):
                self._fill_shape(sh, st)

    # ------------------------------------------------------------------
    def _fill_shape(self, sh, st):
        name = sh.get("name")
        cx, cy = float(st[0]), float(st[1])
        yaw = float(st[2]) if len(st) > 2 else 0.0
        if name == "circle":
            r = float(sh["radius"])
            self._fill_poly(_disc(cx, cy, r))
        elif name == "rectangle":
            L, W = float(sh.get("length", 0.5)), float(sh.get("width", 0.5))
            self._fill_poly(_box(cx, cy, yaw, L, W))
        elif name == "polygon":
            verts = np.asarray(sh.get("vertices", []), dtype=float)
            if len(verts) >= 3:
                self._fill_poly(verts)

    def _fill_poly(self, verts):
        xs, ys = verts[:, 0], verts[:, 1]
        i0 = max(0, int((xs.min() - self.x0) / self.res) - 1)
        i1 = min(self.nx - 1, int((xs.max() - self.x0) / self.res) + 1)
        j0 = max(0, int((ys.min() - self.y0) / self.res) - 1)
        j1 = min(self.ny - 1, int((ys.max() - self.y0) / self.res) + 1)
        if i1 <= i0 or j1 <= j0:
            return
        gx = self.x0 + (np.arange(i0, i1 + 1) + 0.5) * self.res
        gy = self.y0 + (np.arange(j0, j1 + 1) + 0.5) * self.res
        PX, PY = np.meshgrid(gx, gy, indexing="ij")
        self.grid[i0:i1 + 1, j0:j1 + 1] |= _points_in_poly(PX.ravel(), PY.ravel(), verts).reshape(PX.shape)

    # ------------------------------------------------------------------
    def to_index(self, x, y):
        return int((x - self.x0) / self.res), int((y - self.y0) / self.res)

    def to_world(self, ix, iy):
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def inflate_radius(self, mask: np.ndarray, radius_m: float) -> np.ndarray:
        """Inflate an arbitrary boolean mask by `radius_m`."""
        cells = max(1, int(math.ceil(radius_m / self.res)))
        yy, xx = np.mgrid[-cells:cells + 1, -cells:cells + 1]
        disk = (np.hypot(xx, yy) * self.res) <= radius_m
        out = np.zeros_like(mask)
        for ix, iy in np.argwhere(mask):
            i0, i1 = max(0, ix - cells), min(self.nx, ix + cells + 1)
            j0, j1 = max(0, iy - cells), min(self.ny, iy + cells + 1)
            d = disk[(i0 - (ix - cells)):(i1 - (ix - cells)), (j0 - (iy - cells)):(j1 - (iy - cells))]
            out[i0:i1, j0:j1] |= d
        return out

    def inflate(self, radius_m: float, extra: np.ndarray = None) -> np.ndarray:
        """Inflate static occupancy (plus optional `extra` mask) by `radius_m`."""
        base = self.grid if extra is None else (self.grid | extra)
        cells = max(1, int(math.ceil(radius_m / self.res)))
        yy, xx = np.mgrid[-cells:cells + 1, -cells:cells + 1]
        disk = (np.hypot(xx, yy) * self.res) <= radius_m
        out = np.zeros_like(base)
        idx = np.argwhere(base)
        for ix, iy in idx:
            i0, i1 = max(0, ix - cells), min(self.nx, ix + cells + 1)
            j0, j1 = max(0, iy - cells), min(self.ny, iy + cells + 1)
            d = disk[(i0 - (ix - cells)):(i1 - (ix - cells)), (j0 - (iy - cells)):(j1 - (iy - cells))]
            out[i0:i1, j0:j1] |= d
        return out


def _disc(cx, cy, r, n=24):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a)]


def _box(cx, cy, yaw, L, W):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.array([[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, -W / 2], [-L / 2, W / 2]])
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([cx, cy])


def _points_in_poly(px, py, verts):
    """Vectorized ray-casting point-in-polygon check."""
    inside = np.zeros(px.shape, dtype=bool)
    n = len(verts)
    for i in range(n):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % n]
        cond = ((y1 > py) != (y2 > py))
        with np.errstate(divide="ignore", invalid="ignore"):
            xin = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (px < xin)
    return inside


class GlobalMemorySim:
    """Coarse persistent global obstacle memory layer (default 0.20m resolution)."""

    def __init__(self, world: WorldGrid, res: float = 0.20):
        self.world = world
        self.res = res
        self.nx = max(1, int(math.ceil((world.x0 + world.nx * world.res - world.x0) / res)))
        self.nx = max(1, int(math.ceil(world.nx * world.res / res)))
        self.ny = max(1, int(math.ceil(world.ny * world.res / res)))
        self.grid = np.zeros((self.nx, self.ny), dtype=bool)

    def stamp(self, x, y):
        # Only stamp hits that are not already occupied in the static prior map.
        fx, fy = self.world.to_index(float(x), float(y))
        if 0 <= fx < self.world.nx and 0 <= fy < self.world.ny and self.world.grid[fx, fy]:
            return
        ix = int((float(x) - self.world.x0) / self.res)
        iy = int((float(y) - self.world.y0) / self.res)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            self.grid[ix, iy] = True

    def to_fine(self) -> np.ndarray:
        """Upsample coarse memory back to fine grid coordinates (world.nx x world.ny bool)."""
        fine = np.zeros((self.world.nx, self.world.ny), dtype=bool)
        ix, iy = np.nonzero(self.grid)
        if len(ix) == 0:
            return fine
        scale = self.res / self.world.res
        for i, j in zip(ix, iy):
            x0 = int(i * scale); x1 = min(self.world.nx, int((i + 1) * scale) + 1)
            y0 = int(j * scale); y1 = min(self.world.ny, int((j + 1) * scale) + 1)
            fine[x0:x1, y0:y1] = True
        return fine


class SlidingMapSim:
    """Simulation replica of the sliding_map node: raytraced occupancy memory with decay and optional sliding window."""

    def __init__(self, world: WorldGrid, decay_sec: float = 6.0,
                 window_m: float = None, global_memory=None):
        self.world = world
        self.decay_sec = decay_sec
        self.window_m = window_m
        self.global_memory = global_memory
        self.mem = np.zeros((world.nx, world.ny), dtype=bool)
        self.mem_t = np.full((world.nx, world.ny), -1e9)

    def reset(self):
        """Reset fine and coarse memory grids between runs."""
        self.mem[:] = False
        self.mem_t[:] = -1e9
        if self.global_memory is not None:
            self.global_memory.grid[:] = False

    def apply_window(self, robot_xy):
        """Clear memory cells outside the sliding square window centered on `robot_xy`."""
        if self.window_m is None:
            return
        w = self.world
        half = self.window_m / 2.0
        xs = w.x0 + (np.arange(w.nx) + 0.5) * w.res
        ys = w.y0 + (np.arange(w.ny) + 0.5) * w.res
        out_x = np.abs(xs - float(robot_xy[0])) > half
        out_y = np.abs(ys - float(robot_xy[1])) > half
        mask = out_x[:, None] | out_y[None, :]
        self.mem[mask] = False
        self.mem_t[mask] = -1e9

    def raytrace_update(self, t: float, robot_xy, robot_yaw: float,
                        angles, ranges, footprint_radius: float = 0.35,
                        range_max: float = 8.0):
        """Raytrace free space along each beam and mark hit endpoints as occupied (ignoring self-footprint hits)."""
        w = self.world
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        oix, oiy = w.to_index(rx, ry)
        for a, r in zip(angles, ranges):
            finite = np.isfinite(r) and r >= 0.05
            has_hit = finite and r < range_max
            rng = float(r) if finite else range_max
            if has_hit and rng < footprint_radius:
                has_hit = False
            free_limit = max(rng - 1e-3, 0.0) if has_hit else min(rng, range_max)
            ca, sa = math.cos(robot_yaw + a), math.sin(robot_yaw + a)
            nstep = int(free_limit / (w.res * 0.5))
            for si in range(1, nstep + 1):
                d = si * w.res * 0.5
                ix, iy = w.to_index(rx + d * ca, ry + d * sa)
                if 0 <= ix < w.nx and 0 <= iy < w.ny:
                    self.mem[ix, iy] = False
            if has_hit:
                hx, hy = rx + rng * math.cos(robot_yaw + a), ry + rng * math.sin(robot_yaw + a)
                if self.global_memory is not None:
                    self.global_memory.stamp(hx, hy)
                ix, iy = w.to_index(hx, hy)
                if 0 <= ix < w.nx and 0 <= iy < w.ny:
                    self.mem[ix, iy] = True
                    self.mem_t[ix, iy] = t

    def observe(self, t: float, dyn_positions, radius: float = 0.3):
        """Stamp dynamic obstacle discs directly into memory."""
        cells = max(1, int(math.ceil(radius / self.world.res)))
        for (x, y) in dyn_positions:
            ix, iy = self.world.to_index(x, y)
            i0, i1 = max(0, ix - cells), min(self.world.nx, ix + cells + 1)
            j0, j1 = max(0, iy - cells), min(self.world.ny, iy + cells + 1)
            if i1 <= i0 or j1 <= j0:
                continue
            yy, xx = np.mgrid[j0:j1, i0:i1]
            d = np.hypot((xx + 0.5) * self.world.res + self.world.x0 - x,
                         (yy + 0.5) * self.world.res + self.world.y0 - y)
            m = d <= radius
            self.mem[i0:i1, j0:j1] |= m
            self.mem_t[i0:i1, j0:j1][m] = t

    def decay(self, t: float):
        if self.decay_sec and self.decay_sec > 0:
            self.mem &= (t - self.mem_t) <= self.decay_sec

    def plan_grid(self) -> np.ndarray:
        return self.mem


def astar_plan(grid_occ: np.ndarray, start_ij, goal_ij):
    """8-connected A* search (`grid_occ=True` is blocked). Returns list of `(ix, iy)` or `None`."""
    nx, ny = grid_occ.shape
    si, sj = start_ij
    gi, gj = goal_ij
    if grid_occ[si, sj] or grid_occ[gi, gj]:
        return None
    SQ2 = math.sqrt(2)
    neigh = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
             (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]
    g = {start_ij: 0.0}
    parent = {start_ij: None}
    openq = [(0.0, start_ij)]
    closed = set()
    while openq:
        f, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal_ij:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        ci, cj = cur
        for di, dj, w in neigh:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or grid_occ[ni, nj]:
                continue
            if di and dj and (grid_occ[ni, cj] or grid_occ[ci, nj]):
                continue
            ncur = (ni, nj)
            ng = g[cur] + w
            if ncur in closed or ng >= g.get(ncur, float("inf")):
                continue
            g[ncur] = ng
            parent[ncur] = cur
            h = math.hypot(ni - gi, nj - gj)
            heapq.heappush(openq, (ng + h, ncur))
    return None


def _los_clear(grid_occ, a, b, step=0.5):
    """Line-of-sight check between world coordinates `a` and `b`."""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / step))
    for k in range(n + 1):
        x = a[0] + (b[0] - a[0]) * k / n
        y = a[1] + (b[1] - a[1]) * k / n
        if _los_occ(grid_occ, x, y, _los_meta):
            return False
    return True


_los_meta = {}


def _los_occ(grid_occ, x, y, meta):
    res, x0, y0 = meta["res"], meta["x0"], meta["y0"]
    ix, iy = int((x - x0) / res), int((y - y0) / res)
    if not (0 <= ix < grid_occ.shape[0] and 0 <= iy < grid_occ.shape[1]):
        return True
    if grid_occ[ix, iy]:
        return True
    dist = meta.get("dist")
    if dist is not None and dist[ix, iy] < meta.get("los_safe", 0.35):
        return True
    return False


class GlobalPathSim:
    """Global path planner: A* on inflated grid + line-of-sight smoothing + 0.1m densification."""

    def __init__(self, world: WorldGrid, sliding_map: SlidingMapSim,
                 inflation_radius: float = 0.35, res: float = 0.05,
                 dyn_inflation: float = None):
        self.world = world
        self.sliding_map = sliding_map
        self.inflation = inflation_radius
        self.dyn_inflation = dyn_inflation if dyn_inflation else inflation_radius
        self.los_safe = 0.35
        self.smooth_res = res

    def _occ(self, for_planning: bool = True):
        mem = self.sliding_map.plan_grid()
        base = self.world.inflate(self.inflation)
        if mem.any() and abs(self.dyn_inflation - self.inflation) > 1e-6:
            base = base | self.world.inflate_radius(mem, self.dyn_inflation)
        else:
            base = self.world.inflate(self.inflation, extra=mem)
        gm = getattr(self.sliding_map, "global_memory", None)
        if for_planning and gm is not None and gm.grid.any():
            base = base | self.world.inflate_radius(gm.to_fine(), self.inflation)
        return base

    def plan(self, start_xy, goal_xy):
        w = self.world
        gm = getattr(self.sliding_map, "global_memory", None)
        occ_fine = self._occ(for_planning=False)
        if gm is not None and gm.grid.any():
            occ_coarse = self.world.inflate_radius(gm.to_fine(), self.inflation)
            exempt = np.zeros_like(occ_fine)
            rr = max(1, int((self.inflation + 0.3) / w.res))
            yy, xx = np.mgrid[-rr:rr + 1, -rr:rr + 1]
            disk = (xx * xx + yy * yy) <= rr * rr
            for pxy in (start_xy, goal_xy):
                i, j = w.to_index(*pxy)
                i0, i1 = max(0, i - rr), min(w.nx, i + rr + 1)
                j0, j1 = max(0, j - rr), min(w.ny, j + rr + 1)
                exempt[i0:i1, j0:j1] |= disk[(i0 - (i - rr)):(i1 - (i - rr)),
                                             (j0 - (j - rr)):(j1 - (j - rr))]
            occ = occ_fine | (occ_coarse & ~exempt)
        else:
            occ = occ_fine
        _los_meta.update(res=w.res, x0=w.x0, y0=w.y0)
        if self.los_safe > 0:
            from scipy import ndimage
            _los_meta["dist"] = ndimage.distance_transform_edt(~occ) * w.res
            _los_meta["los_safe"] = self.los_safe
        else:
            _los_meta.pop("dist", None)
        s_ij = _nearest_free(occ_fine, w.to_index(*start_xy))
        g_ij = _nearest_free(occ_fine, w.to_index(*goal_xy))
        if s_ij is None or g_ij is None:
            return None
        path = astar_plan(occ, s_ij, g_ij)
        if not path:
            return None
        pts = [w.to_world(i, j) for i, j in path]
        pts = _smooth_los(pts, occ, w)
        return _densify(pts, 0.1)


def _nearest_free(occ, ij, max_r=20):
    i, j = ij
    if 0 <= i < occ.shape[0] and 0 <= j < occ.shape[1] and not occ[i, j]:
        return ij
    best, bd = None, 1e9
    for di in range(-max_r, max_r + 1):
        for dj in range(-max_r, max_r + 1):
            ni, nj = i + di, j + dj
            if 0 <= ni < occ.shape[0] and 0 <= nj < occ.shape[1] and not occ[ni, nj]:
                d = di * di + dj * dj
                if d < bd:
                    bd, best = d, (ni, nj)
    return best


def _smooth_los(pts, occ, world, max_iter=200):
    """Iterative line-of-sight shortcutting."""
    _los_meta.update(res=world.res, x0=world.x0, y0=world.y0)
    pts = list(pts)
    it = 0
    i = 0
    while i < len(pts) - 2 and it < max_iter:
        it += 1
        if _los_clear(occ, pts[i], pts[-1]):
            pts = pts[:i + 1] + [pts[-1]]
            break
        j = len(pts) - 1
        while j > i + 1:
            if _los_clear(occ, pts[i], pts[j]):
                pts = pts[:i + 1] + pts[j:]
                break
            j -= max(1, (j - i) // 2)
        i += 1
    return pts


def _densify(pts, step=0.1):
    """Densify polyline to `step` spacing with segment tangent headings `[[x, y, theta], ...]`."""
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / step))
        th = math.atan2(b[1] - a[1], b[0] - a[0])
        for k in range(n):
            out.append([a[0] + (b[0] - a[0]) * k / n,
                        a[1] + (b[1] - a[1]) * k / n, th])
    th = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
    out.append([pts[-1][0], pts[-1][1], th])
    return out
