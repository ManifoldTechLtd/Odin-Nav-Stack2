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

"""Convert a cropped region of a PGM + YAML occupancy map into an ir-sim benchmark scene.

Pipeline:
  occupancy grid -> coarse cell grid -> greedy rectangle merging -> ir-sim rectangle obstacles + planner.yaml

Usage:
  python3 scripts/map_processing/pgm_to_irsim_world.py \
    --pgm inspection_map_save/office/map_opt.pgm \
    --map-yaml inspection_map_save/office/map_opt.yaml \
    --crop -14,-31,20,24 --start 1.0,0.6,0 --goal 18,22,0 \
    --out NeuPAN/example/sw01_bench/office_real [--cell 0.10] [--preview]
"""
import argparse
import shutil
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
REAL_PLANNER = REPO / "robots/sw01/neupan/planner.yaml"

ROBOT = {
    "kinematics": {"name": "diff"},
    "shape": {"name": "rectangle", "length": 0.96, "width": 0.55},
    "vel_min": [-1.0, -3.14], "vel_max": [1.0, 3.14],
    "goal_threshold": 0.25,
    "plot": {"show_trail": True, "show_goal": False, "show_trajectory": False},
    "sensors": [{"type": "lidar2d", "range_min": 0, "range_max": 8,
                 "angle_range": 2.094, "number": 200, "noise": False}],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--crop", required=True, help="x,y,w,h (map frame, meters)")
    ap.add_argument("--start", required=True, help="x,y,theta (map frame)")
    ap.add_argument("--goal", required=True, help="x,y,theta (map frame)")
    ap.add_argument("--out", required=True, help="Output scene directory")
    ap.add_argument("--cell", type=float, default=0.10, help="Rectangle merging grid size (meters)")
    ap.add_argument("--min-cells", type=int, default=2, help="Discard fragments smaller than min_cells")
    ap.add_argument("--unknown-as-obstacle", action="store_true")
    ap.add_argument("--preview", action="store_true", help="Write preview.png")
    args = ap.parse_args()

    my = yaml.safe_load(open(args.map_yaml))
    res = float(my["resolution"])
    ox, oy = float(my["origin"][0]), float(my["origin"][1])
    img = np.array(Image.open(args.pgm))
    H, W = img.shape
    occupied = img < 100
    if args.unknown_as_obstacle:
        occupied |= (img > 150) & (img < 230)

    cx, cy, cw, ch = [float(v) for v in args.crop.split(",")]
    # Map coordinates -> pixel indices: col=(x-ox)/res, row=(oy+H*res-y)/res
    def to_px(x, y):
        return int((x - ox) / res), int((oy + H * res - y) / res)
    c0, r1 = to_px(cx, cy)
    c1, r0 = to_px(cx + cw, cy + ch)
    r0, r1 = sorted((max(0, min(H - 1, r0)), max(0, min(H - 1, r1))))
    c0, c1 = sorted((max(0, min(W - 1, c0)), max(0, min(W - 1, c1))))
    sub = occupied[r0:r1 + 1, c0:c1 + 1]
    y_top = oy + (H - r0) * res
    y_bot = oy + (H - r1 - 1) * res
    x_left = ox + c0 * res
    x_right = ox + (c1 + 1) * res
    print(f"crop pixels [{r0}:{r1},{c0}:{c1}] -> world x[{x_left:.2f},{x_right:.2f}] "
          f"y[{y_bot:.2f},{y_top:.2f}] occupied ratio {sub.mean():.3f}")

    # Coarse grid + greedy rectangle merging
    cell = args.cell
    ncx = int(np.ceil(sub.shape[1] * res / cell))
    ncy = int(np.ceil(sub.shape[0] * res / cell))
    blk = np.zeros((ncy, ncx), dtype=bool)
    for j in range(ncy):
        for i in range(ncx):
            r_a = int(j * cell / res); r_b = min(sub.shape[0], int((j + 1) * cell / res))
            c_a = int(i * cell / res); c_b = min(sub.shape[1], int((i + 1) * cell / res))
            if r_b > r_a and c_b > c_a:
                blk[j, i] = sub[r_a:r_b, c_a:c_b].mean() > 0.35
    rects = []
    used = np.zeros_like(blk)
    for j in range(ncy):
        i = 0
        while i < ncx:
            if blk[j, i] and not used[j, i]:
                i2 = i
                while i2 + 1 < ncx and blk[j, i2 + 1] and not used[j, i2 + 1]:
                    i2 += 1
                j2 = j
                while j2 + 1 < ncy and all(blk[j2 + 1, k] and not used[j2 + 1, k] for k in range(i, i2 + 1)):
                    j2 += 1
                used[j:j2 + 1, i:i2 + 1] = True
                n_cells = (i2 - i + 1) * (j2 - j + 1)
                if n_cells >= args.min_cells:
                    wx0 = ox + (c0 + i) * res
                    wx1 = ox + (c0 + i2 + 1) * res
                    wy0 = oy + (H - (r0 + j2 + 1)) * res
                    wy1 = oy + (H - (r0 + j)) * res
                    rects.append(((wx0 + wx1) / 2, (wy0 + wy1) / 2, wx1 - wx0, wy1 - wy0))
                i = i2 + 1
            else:
                i += 1
    print(f"Merged into {len(rects)} rectangle obstacles")

    sx, sy, sth = [float(v) for v in args.start.split(",")]
    gx, gy, gth = [float(v) for v in args.goal.split(",")]
    world = {"width": round(x_right - x_left, 3), "height": round(y_top - y_bot, 3),
             "step_time": 0.1, "sample_time": 0.1,
             "offset": [round(x_left, 3), round(y_bot, 3)],
             "collision_mode": "stop", "control_mode": "auto"}
    robot = dict(ROBOT)
    robot["state"] = [sx, sy, sth]
    robot["goal"] = [gx, gy, gth]
    shapes = [{"name": "rectangle", "length": round(l, 3), "width": round(w, 3)}
              for _, _, l, w in rects]
    states = [[round(a, 3), round(b, 3), 0] for a, b, _, _ in rects]
    env = {"world": world, "robot": [robot],
           "obstacle": [{"number": len(shapes), "distribution": {"name": "manual"},
                         "shape": shapes, "state": states}]}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "env.yaml", "w") as f:
        f.write(f"# Generated by pgm_to_irsim_world.py\n"
                f"# Source: {args.pgm} crop={args.crop}\n")
        yaml.safe_dump(env, f, allow_unicode=True, sort_keys=False)
    shutil.copy(REAL_PLANNER, out / "planner.yaml")
    print(f"Wrote {out}/env.yaml + planner.yaml")

    if args.preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(sub, origin="lower", extent=[x_left, x_right, y_bot, y_top],
                  cmap="gray_r")
        for a, b, l, w in rects:
            ax.add_patch(plt.Rectangle((a - l / 2, b - w / 2), l, w,
                                       fill=False, ec="red", lw=0.4))
        ax.plot(sx, sy, "go", ms=10)
        ax.plot(gx, gy, "r*", ms=14)
        ax.set_aspect("equal")
        fig.savefig(out / "preview.png", dpi=140)
        print(f"preview -> {out}/preview.png")


if __name__ == "__main__":
    main()
