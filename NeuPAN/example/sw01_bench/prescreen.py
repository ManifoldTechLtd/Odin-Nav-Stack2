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

"""Prescreen matrix runner for Stage A (corner90 q_s x curvature_speed) and Stage B (corridor rotate_clearance_m)."""
import csv
import json
import re
import subprocess
import sys
import time
from pathlib import Path

BENCH = Path(__file__).resolve().parent
PY = sys.executable
COMMON = ["--global-path", "--real-defaults",
          "--delay-ms", "100", "--perception-delay-ms", "200", "--chassis-tau", "0.15",
          "--set", "ref_speed=0.6"]
OUT_RE = re.compile(r"\[done\] Results saved: (\S+)")

STAGE_A = []
for q_s in (1.0, 2.0, 2.5, 3.0):
    for curv in (False, True):
        STAGE_A.append(dict(
            scene="corner90", runs=3, max_steps=700,
            extra=["--set", f"adjust.q_s={q_s}"] + (["--curvature"] if curv else []),
            label=f"q_s={q_s} curv={'on' if curv else 'off'}"))

STAGE_B = []
for gate in (0.57, 0.75, 0.85):
    for scene in ("corridor110", "corridor130", "narrow09_entry", "deadend_T", "narrow_blocked"):
        STAGE_B.append(dict(
            scene=scene, runs=2, max_steps=1400,
            extra=["--set", "adjust.q_s=2.0", "--escape-set", f"rotate_clearance_m={gate}"],
            label=f"gate={gate}"))


def run_case(case, out_root):
    cmd = [PY, str(BENCH / "benchmark_sw01.py"),
           "--scene", case["scene"], "--runs", str(case["runs"]),
           "--max-steps", str(case["max_steps"]),
           "--tag", case["label"].replace(" ", "_").replace("=", ""),
           ] + COMMON + case["extra"]
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=BENCH)
    m = OUT_RE.search(proc.stdout)
    row = dict(stage=case["stage"], scene=case["scene"], label=case["label"],
               wall_s=round(time.time() - t0, 1))
    if not m:
        row["error"] = (proc.stdout + proc.stderr)[-400:]
        return row
    summ = json.loads((Path(m.group(1)) / "summary.json").read_text())
    agg = summ["aggregate"]
    runs = summ["runs"]
    row.update(
        success_rate=agg["success_rate"], collision_rate=agg["collision_rate"],
        sim_time_s=agg.get("sim_time_s_avg"),
        path_lat_mean=round(sum(r["path_lat_mean"] or 0 for r in runs) / len(runs), 3),
        path_lat_max=round(max((r["path_lat_max"] or 0) for r in runs), 3),
        corner_ov=round(max((r["corner_overshoot_max"] or 0) for r in runs), 3),
        min_dist=agg.get("min_dist_min"),
        escapes=round(sum(r["escape_count"] for r in runs) / len(runs), 2),
        esc_rot=sorted({round(a) for r in runs for a in (r.get("escape_rot_deg") or [])}),
        flips=agg.get("omega_flips_avg"),
        curv_vmin=min([r["curv_v_min"] for r in runs if r.get("curv_v_min")] or [None]),
        out_dir=str(Path(m.group(1)).name),
    )
    return row


def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else "A"
    out_root = BENCH / "results" / f"prescreen_{stage}_{time.strftime('%m%d_%H%M')}"
    out_root.mkdir(parents=True, exist_ok=True)
    cases = STAGE_A if stage == "A" else STAGE_B
    for c in cases:
        c["stage"] = stage
    rows = []
    for i, case in enumerate(cases):
        print(f"[{i+1}/{len(cases)}] {case['scene']} {case['label']} ...", flush=True)
        row = run_case(case, out_root)
        rows.append(row)
        print("   ", {k: row[k] for k in row if k not in ("error", "stage")}, flush=True)
        with open(out_root / "prescreen.csv", "w", newline="") as f:
            wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            wtr.writeheader()
            wtr.writerows(rows)
    lines = [f"# Prescreen Stage {stage} Results ({time.strftime('%F %H:%M')})", "",
             "| scene | label | success | collision | path_lat_mean | path_lat_max | corner_ov | min_dist | escapes | rot_deg | flips | sim_time |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if "error" in r:
            lines.append(f"| {r['scene']} | {r['label']} | ERROR | | | | | | | | | {r['error'][:120]} |")
        else:
            lines.append(f"| {r['scene']} | {r['label']} | {r['success_rate']:.0%} | {r['collision_rate']:.0%} "
                         f"| {r['path_lat_mean']} | {r['path_lat_max']} | {r['corner_ov']} | {r['min_dist']} "
                         f"| {r['escapes']} | {r['esc_rot']} | {r['flips']} | {r['sim_time_s']}s |")
    (out_root / "report.md").write_text("\n".join(lines))
    print(f"\n[done] {out_root}/prescreen.csv + report.md")


if __name__ == "__main__":
    main()
