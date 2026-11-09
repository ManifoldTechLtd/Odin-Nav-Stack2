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

"""Interactive matplotlib visualization runner for SW01 NeuPAN benchmark scenes."""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

NEUPAN_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NEUPAN_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from neupan import neupan  # noqa: E402
import irsim  # noqa: E402
from benchmark_sw01 import load_yaml, save_yaml, apply_overrides, parse_set_pairs, TMP_DIR, BENCH_DIR  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="SW01 real-time visualization")
    ap.add_argument("--scene", default="narrow_corridor",
                    choices=["open", "wide_corridor", "narrow_corridor", "dynamic_human"])
    ap.add_argument("--set", action="append", default=[], help="Parameter override key=value")
    ap.add_argument("--delay-ms", type=int, default=0, help="cmd_vel delay injection (ms)")
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    ap.add_argument("--max-steps", type=int, default=900)
    args = ap.parse_args()

    overrides = parse_set_pairs(args.set)
    scene_dir = BENCH_DIR / args.scene
    env_file = scene_dir / "env.yaml"

    planner_cfg = apply_overrides(load_yaml(scene_dir / "planner.yaml"), overrides)
    tmp_planner = TMP_DIR / "planner_view.yaml"
    save_yaml(planner_cfg, tmp_planner)

    env = irsim.make(str(env_file), save_ani=False, full=False, display=True)
    planner = neupan.init_from_yaml(str(tmp_planner))

    env_cfg = load_yaml(env_file)
    step_time = env_cfg["world"].get("step_time", 0.1)
    delay_steps = max(0, int(round(args.delay_ms / 1000.0 / step_time)))
    action_buf = [np.zeros((2, 1))] * delay_steps

    print(f"[view] scene={args.scene} overrides={overrides or 'baseline'} "
          f"delay={args.delay_ms}ms speed={args.speed}x")
    print("       (green=DUNE points, red=NRMP active points, red line=opt trajectory, blue line=ref path)")

    for k in range(args.max_steps):
        robot_state = env.get_robot_state()
        lidar_scan = env.get_lidar_scan()
        points = planner.scan_to_point(robot_state, lidar_scan)

        t0 = time.perf_counter()
        action, info = planner(robot_state, points)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        if delay_steps > 0:
            action_buf.append(action)
            action_out = action_buf.pop(0)
        else:
            action_out = action

        if k % 10 == 0:
            try:
                md = planner.min_distance
                md = float(md.detach().cpu()) if hasattr(md, "detach") else float(md)
            except Exception:
                md = float("nan")
            x, y, th = robot_state.flatten()[:3]
            v, w = action_out.flatten()
            print(f"  step={k:4d} pos=({x:6.2f},{y:6.2f},{th:+.2f}) "
                  f"cmd=(v={v:+.3f}, w={w:+.3f}) min_dist={md:.3f} compute={dt_ms:.1f}ms")

        if info.get("arrive"):
            print(f"[view] Reached goal @ step={k}")
            break
        if info.get("stop"):
            print(f"[view] Stopped (min_distance threshold) @ step={k}")
            break

        env.draw_points(planner.dune_points, s=25, c="g", refresh=True)
        env.draw_points(planner.nrmp_points, s=13, c="r", refresh=True)
        env.draw_trajectory(planner.opt_trajectory, "r", refresh=True)
        env.draw_trajectory(planner.ref_trajectory, "b", refresh=True)
        env.step(action_out)
        time.sleep(max(0.0, step_time / max(args.speed, 1e-3) - dt_ms / 1000.0))

    env.end()
    print("[view] Done")


if __name__ == "__main__":
    main()
