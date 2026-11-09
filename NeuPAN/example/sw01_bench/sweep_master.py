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

"""Coordinate-descent parameter optimization framework for SW01 NeuPAN."""

import argparse
import itertools
import json
import time
from pathlib import Path

import numpy as np

import benchmark_sw01 as B


# Hierarchical coordinate-descent parameter search space
PARAM_L1 = {
    "adjust.d_max": [0.05, 0.08, 0.1, 0.15, 0.2, 0.3, 0.4],
    "adjust.eta":   [4, 6, 8, 10, 12, 15],
    "receding":     [8, 10, 12],
}
PARAM_L2 = {
    "adjust.p_w":   [0.5, 0.8, 1.0, 1.2],
    "adjust.q_s":   [0.8, 1.0, 1.2, 1.5],
    "adjust.ro_obs": [300, 350, 400, 500],
}
PARAM_L3 = {
    "adjust.d_min": [0.02, 0.05],
    "adjust.q_theta": [1.0, 1.2],
}

DEFAULT_SCENES = ["open", "wide_corridor", "narrow_corridor",
                  "narrow_to_wide", "wide_to_narrow",
                  "dynamic_human", "head_on", "human_block", "office"]
DYNAMIC_SCENES = {"dynamic_human", "head_on", "human_block", "office"}


class SweepMaster:
    def __init__(self, args):
        self.args = args
        self.out_dir = B.RESULTS_DIR / "sweep_master" / time.strftime("%Y%m%d_%H%M%S")
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.results_file = self.out_dir / "results.jsonl"
        self.best_file = self.out_dir / "best_params.json"
        self.report_file = self.out_dir / "report.md"
        self.best = {}          # scene -> best record
        self.eval_count = 0
        self.no_improve = 0

    def evaluate(self, scene, overrides, delay_ms, perception_ms=0, chassis_tau=0.0):
        """Evaluate a parameter configuration on `scene` across `runs` trials and return aggregated metrics."""
        runs = 3 if scene in DYNAMIC_SCENES else 1
        if self.args.fast:
            runs = 1
        scene_dir = B.BENCH_DIR / scene
        env_file = scene_dir / "env.yaml"
        template = B.load_yaml(scene_dir / "planner.yaml")

        run_metrics = []
        for i in range(runs):
            m = B.run_once(
                env_file, template, overrides, self.args.max_steps, delay_ms,
                perception_delay_ms=perception_ms, chassis_tau=chassis_tau,
                lidar_override=(B.REAL_LIDAR_OVERRIDE if self.args.real_lidar else None))
            m["run_index"] = i
            run_metrics.append(m)
        agg = B.aggregate(run_metrics)
        agg["n_runs"] = runs
        return agg

    def score(self, agg):
        """Compute scalar score (hard zero-collision constraint, prioritizing success rate, completion time, and smoothness)."""
        if agg["collision_rate"] > 0:
            return -1.0
        s = agg["success_rate"] * 100.0
        s -= (agg["sim_time_s_avg"] or 100) * 0.1
        s -= (agg["omega_flips_avg"] or 0) * 0.05
        s -= (agg["lat_err_mean_avg"] or 0) * 10.0
        return s

    def is_better(self, new_score, new_agg, old_score, old_agg):
        if new_agg["collision_rate"] > 0:
            return False
        if old_agg["collision_rate"] > 0:
            return True
        return new_score > old_score

    def coordinate_descent(self, scene, delay_ms=0, perception_ms=0, chassis_tau=0.0):
        """Run coordinate descent across L1, L2, and L3 parameter layers for `scene`."""
        print(f"\n[sweep] {scene} (delay={delay_ms}ms) coordinate descent")
        best_over = {}
        best_agg = self.evaluate(scene, best_over, delay_ms, perception_ms, chassis_tau)
        best_score = self.score(best_agg)
        self._log(scene, best_over, delay_ms, perception_ms, chassis_tau, best_agg, best_score, "baseline")
        print(f"  baseline: succ={best_agg['success_rate']*100:.0f}% score={best_score:.1f}")

        for layer, space in [("L1", PARAM_L1), ("L2", PARAM_L2), ("L3", PARAM_L3)]:
            improved = True
            while improved:
                improved = False
                for param, values in space.items():
                    cur = best_over.get(param)
                    for v in values:
                        if v == cur:
                            continue
                        cand = dict(best_over); cand[param] = v
                        agg = self.evaluate(scene, cand, delay_ms, perception_ms, chassis_tau)
                        sc = self.score(agg)
                        self.eval_count += 1
                        self._log(scene, cand, delay_ms, perception_ms, chassis_tau, agg, sc, layer)
                        if self.is_better(sc, agg, best_score, best_agg):
                            best_over, best_agg, best_score = cand, agg, sc
                            improved = True
                            self.no_improve = 0
                            print(f"  [{layer}] {param}={v}: succ={agg['success_rate']*100:.0f}% "
                                  f"coll={agg['collision_rate']*100:.0f}% score={sc:.1f} <- new best")
                        else:
                            self.no_improve += 1
        return best_over, best_agg, best_score

    def _log(self, scene, overrides, delay_ms, perception_ms, chassis_tau, agg, score, phase):
        lidar = B.lidar_config(B.load_yaml(B.BENCH_DIR / scene / "env.yaml"))
        if self.args.real_lidar:
            lidar.update(B.REAL_LIDAR_OVERRIDE)
        rec = dict(scene=scene, overrides=overrides, delay_ms=delay_ms,
                   perception_ms=perception_ms, chassis_tau=chassis_tau,
                   lidar=lidar, phase=phase, score=round(score, 3), **agg)
        with open(self.results_file, "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def _update_best(self, scene, over, agg, score):
        cur = self.best.get(scene)
        if cur is None or score > cur["score"]:
            lidar = B.lidar_config(B.load_yaml(B.BENCH_DIR / scene / "env.yaml"))
            if self.args.real_lidar:
                lidar.update(B.REAL_LIDAR_OVERRIDE)
            self.best[scene] = dict(
                overrides=over, score=score,
                success_rate=agg["success_rate"],
                collision_rate=agg["collision_rate"],
                sim_time=agg["sim_time_s_avg"],
                delay_ms=int(self.args.delays.split(",")[0]),
                perception_delay_ms=self.args.perception_delay_ms,
                chassis_tau=self.args.chassis_tau, lidar=lidar)
            with open(self.best_file, "w") as f:
                json.dump(self.best, f, ensure_ascii=False, indent=2)

    def run(self):
        scenes = self.args.scenes.split(",") if self.args.scenes else DEFAULT_SCENES
        delays = [int(x) for x in self.args.delays.split(",")] if self.args.delays else [0]
        t_start = time.time()
        print(f"[sweep_master] scenes={scenes} delays={delays} out={self.out_dir}")

        search_delay = delays[0]
        for scene in scenes:
            over, agg, score = self.coordinate_descent(
                scene, search_delay, self.args.perception_delay_ms, self.args.chassis_tau)
            self._update_best(scene, over, agg, score)
            print(f"[sweep] {scene} best: {over} succ={agg['success_rate']*100:.0f}%")

        if len(delays) > 1:
            print(f"\n[sweep] delay regression: {delays[1:]}")
            for scene in scenes:
                best_over = self.best.get(scene, {}).get("overrides", {})
                for d in delays[1:]:
                    agg = self.evaluate(
                        scene, best_over, d,
                        self.args.perception_delay_ms, self.args.chassis_tau)
                    sc = self.score(agg)
                    self._log(
                        scene, best_over, d,
                        self.args.perception_delay_ms, self.args.chassis_tau,
                        agg, sc, "delay_regress")
                    print(f"  {scene} delay={d}ms: succ={agg['success_rate']*100:.0f}% "
                          f"coll={agg['collision_rate']*100:.0f}%")

        self._write_report(scenes, delays, time.time() - t_start)

    def _write_report(self, scenes, delays, elapsed):
        lines = ["# SW01 NeuPAN Parameter Optimization Report\n",
                 f"- Elapsed time: {elapsed/60:.1f} min",
                 f"- Evaluations: {self.eval_count}",
                 f"- Scenes: {scenes}", f"- Action delays: {delays}",
                 f"- Perception delay: {self.args.perception_delay_ms} ms",
                 f"- Chassis tau: {self.args.chassis_tau} s",
                 f"- Real lidar: {self.args.real_lidar}\n",
                 "## Best Parameters Per Scene\n",
                 "| Scene | Best Overrides | Success Rate | Collision Rate | Sim Time (s) |",
                 "|---|---|---|---|---|"]
        for scene, rec in self.best.items():
            ov = ", ".join(f"{k.split('.')[-1]}={v}" for k, v in rec["overrides"].items()) or "baseline"
            lines.append(f"| {scene} | {ov} | {rec['success_rate']*100:.0f}% | "
                         f"{rec['collision_rate']*100:.0f}% | {rec['sim_time']} |")
        self.report_file.write_text("\n".join(lines), encoding="utf-8")
        print(f"\n[done] Report saved: {self.report_file}")
        print(f"[done] Best params: {self.best_file}")


def main():
    ap = argparse.ArgumentParser(description="SW01 coordinate-descent parameter sweep")
    ap.add_argument("--scenes", default="", help="Comma-separated scene subset (defaults to all 9 scenes)")
    ap.add_argument("--delays", default="0", help="Comma-separated action delay levels (ms)")
    ap.add_argument("--perception-delay-ms", type=int, default=0)
    ap.add_argument("--chassis-tau", type=float, default=0.0)
    ap.add_argument("--real-lidar", action="store_true")
    ap.add_argument("--max-steps", type=int, default=800)
    ap.add_argument("--fast", action="store_true", help="Fast mode (single run per dynamic scene)")
    args = ap.parse_args()
    SweepMaster(args).run()


if __name__ == "__main__":
    main()
