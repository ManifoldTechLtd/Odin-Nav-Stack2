#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

"""Coordinate-descent parameter sweep for pure MPC (N0: stuck_escape=False, adaptive_d_max=False)."""
import copy
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import benchmark_sw01 as B
import sweep_master as SM

SCENES = [
    "corridor110", "corridor130", "narrow09_entry",
    "ped_block_open", "ped_block_corridor", "office_open_pedestrian",
    "open", "dynamic_human",
]

DELAY_MS = 100
PERCEPTION_MS = 200
CHASSIS_TAU = 0.15
MAX_STEPS = 1200


def patched_evaluate(self, scene, overrides, delay_ms, perception_ms=0, chassis_tau=0.0):
    """Evaluate a parameter set in pure MPC mode (escape and adaptive_d_max disabled) with A* global path."""
    scene_dir = B.BENCH_DIR / scene
    env_file = scene_dir / "env.yaml"
    template = B.load_yaml(scene_dir / "planner.yaml")

    gp = _get_global_path(scene)
    try:
        m = B.run_once(env_file, template, overrides,
                       self.args.max_steps, delay_ms,
                       perception_delay_ms=perception_ms, chassis_tau=chassis_tau,
                       enable_escape=False, adaptive_cfg={"enable": False},
                       global_path=gp,
                       save_ani=False,
                       lidar_override=B.REAL_LIDAR_OVERRIDE)
    except Exception as e:  # noqa: BLE001
        print(f"  [evaluate] {scene} overrides={overrides} exception: {e}")
        return {"success_rate": 0.0, "collision_rate": 0.0, "timeout_rate": 1.0,
                "sim_time_s_avg": None, "omega_flips_avg": None,
                "lat_err_mean_avg": None, "n_runs": 1, "error": str(e)}
    m["run_index"] = 0
    agg = B.aggregate([m])
    agg["n_runs"] = 1
    return agg


_GP_CACHE = {}


def _get_global_path(scene):
    """Return cached GlobalPathSim instance for `scene`."""
    if scene not in _GP_CACHE:
        env_cfg = B.load_yaml(B.BENCH_DIR / scene / "env.yaml")
        _w = B.WorldGrid(env_cfg, res=0.05)
        _f = B.SlidingMapSim(_w, decay_sec=3.0, window_m=20.0)
        _GP_CACHE[scene] = B.GlobalPathSim(_w, _f)
    return _GP_CACHE[scene]


def save_best_gifs(self):
    """Re-run the best parameter configuration per scene and save GIF animations."""
    gif_dir = self.out_dir / "gif"
    gif_dir.mkdir(parents=True, exist_ok=True)
    print("\n[h0] Generating best-parameter GIFs per scene ...")
    for scene, rec in self.best.items():
        over = rec.get("overrides", {})
        try:
            scene_dir = B.BENCH_DIR / scene
            template = B.load_yaml(scene_dir / "planner.yaml")
            delay_ms = int(self.args.delays.split(",")[0])
            B.run_once(scene_dir / "env.yaml", template, over,
                       self.args.max_steps, delay_ms,
                       perception_delay_ms=self.args.perception_delay_ms,
                       chassis_tau=self.args.chassis_tau,
                       enable_escape=False, adaptive_cfg={"enable": False},
                       global_path=_get_global_path(scene),
                       save_ani=True, ani_name=f"h0_N0_{scene}_best",
                       lidar_override=B.REAL_LIDAR_OVERRIDE)
            print(f"  [gif] {scene} ok")
        except Exception as e:  # noqa: BLE001
            print(f"  [gif] {scene} failed: {e}")


def main():
    SM.PARAM_L1 = {
        "adjust.d_max": [0.08, 0.12, 0.18, 0.25, 0.35],
        "adjust.eta":   [6, 8, 10, 14],
        "receding":     [8, 10],
    }
    SM.PARAM_L2 = {
        "adjust.q_s":   [0.8, 1.2, 1.5, 2.0],
        "adjust.p_w":   [0.5, 0.8, 1.0],
        "adjust.ro_obs": [300, 400, 500],
    }
    SM.PARAM_L3 = {
        "adjust.d_min":    [0.02, 0.05],
        "adjust.q_theta":  [1.0, 1.4],
    }

    SM.SweepMaster.evaluate = patched_evaluate
    SM.SweepMaster.save_best_gifs = save_best_gifs

    orig_write_report = SM.SweepMaster._write_report

    def write_report_with_gif(self, scenes, delays, elapsed):
        orig_write_report(self, scenes, delays, elapsed)
        self.save_best_gifs()

    SM.SweepMaster._write_report = write_report_with_gif

    argv = ["sweep_master.py", "--scenes", ",".join(SCENES),
            "--delays", str(DELAY_MS),
            "--perception-delay-ms", str(PERCEPTION_MS),
            "--chassis-tau", str(CHASSIS_TAU), "--real-lidar",
            "--max-steps", str(MAX_STEPS)] + sys.argv[1:]
    sys.argv = argv
    print(f"[h0] N0 pure MPC parameter sweep\n  scenes: {SCENES}\n  delay: {DELAY_MS}ms "
          f"(perception {PERCEPTION_MS}ms / chassis tau {CHASSIS_TAU}s)\n"
          f"  lidar: {B.REAL_LIDAR_OVERRIDE}\n  args: {sys.argv}")
    SM.main()


if __name__ == "__main__":
    main()
