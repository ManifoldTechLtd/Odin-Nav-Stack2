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

"""Record GIF animations for SW01 benchmark scenes."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml
from neupan import neupan
import irsim
import numpy as np
import os
import shutil
from benchmark_sw01 import load_yaml, save_yaml, apply_overrides, parse_set_pairs, TMP_DIR, BENCH_DIR, NEUPAN_ROOT


def record(scene, out_gif, overrides=None, max_steps=600, delay_ms=0):
    scene_dir = BENCH_DIR / scene
    cfg = apply_overrides(load_yaml(scene_dir / "planner.yaml"), overrides or {})
    ck = Path(NEUPAN_ROOT) / "example/dune_train/model/sw01/model_5000.pth"
    if not ck.exists():
        cfg["pan"]["dune_checkpoint"] = "example/dune_train/model/sw01/model_500.pth"
    tmp = TMP_DIR / "planner_record.yaml"
    save_yaml(cfg, tmp)

    env = irsim.make(str(scene_dir / "env.yaml"), save_ani=True, full=False, display=True)
    planner = neupan.init_from_yaml(str(tmp))
    env_cfg = load_yaml(scene_dir / "env.yaml")
    step_time = env_cfg["world"].get("step_time", 0.1)
    delay_steps = max(0, int(round(delay_ms / 1000.0 / step_time)))
    action_buf = [np.zeros((2, 1))] * delay_steps

    for k in range(max_steps):
        st = env.get_robot_state(); scan = env.get_lidar_scan()
        pts = planner.scan_to_point(st, scan)
        action, info = planner(st, pts)
        if delay_steps > 0:
            action_buf.append(action); action_out = action_buf.pop(0)
        else:
            action_out = action
        if info["arrive"]:
            print(f"arrive @ {k}")
            break
        if info["stop"]:
            print(f"stop @ {k}")
            break
        env.draw_points(planner.dune_points, s=25, c="g", refresh=True)
        env.draw_points(planner.nrmp_points, s=13, c="r", refresh=True)
        env.draw_trajectory(planner.opt_trajectory, "r", refresh=True)
        env.draw_trajectory(planner.ref_trajectory, "b", refresh=True)
        env.step(action_out)
        env.render(interval=0.001)

    ani_name = os.path.splitext(os.path.basename(out_gif))[0]
    env.end(ani_name=ani_name)
    default_gif = os.path.join(str(BENCH_DIR), "animation", ani_name + ".gif")
    if os.path.exists(default_gif):
        os.makedirs(os.path.dirname(out_gif), exist_ok=True)
        shutil.move(default_gif, out_gif)
        print(f"GIF saved: {out_gif}")
    else:
        alt = os.path.join("animation", ani_name + ".gif")
        if os.path.exists(alt):
            os.makedirs(os.path.dirname(out_gif), exist_ok=True)
            shutil.move(alt, out_gif)
            print(f"GIF saved: {out_gif}")
        else:
            print(f"WARN: animation not found ({default_gif} or {alt})")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="wide_corridor")
    ap.add_argument("--out", default="wc_fix.gif")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--delay-ms", type=int, default=0)
    a = ap.parse_args()
    record(a.scene, a.out, parse_set_pairs(a.set), a.max_steps, a.delay_ms)
