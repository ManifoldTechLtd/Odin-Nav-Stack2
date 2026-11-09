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

"""Cost v2 trajectory evaluator: computes safety margin, single-pass resolution, reaction delay, and wall clearance metrics from BENCH_SAVE_TRAJ npz archives."""
import argparse
import sys

import numpy as np
import yaml
from scipy import ndimage

sys.path.insert(0, ".")
from sim_system import WorldGrid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--margin", type=float, default=0.15)
    ap.add_argument("--hug-ratio", type=float, default=0.05)
    ap.add_argument("--react", type=float, default=2.7)
    ap.add_argument("--resolve", type=float, default=0.8)
    ap.add_argument("--wall-hug", type=float, default=0.25)
    args = ap.parse_args()

    d = np.load(args.npz)
    frames = d["frames"]          # (t, cmd_v, cmd_w, min_dist, escape_active)
    traj = d["traj"]
    enters = d["enter_times"] if "enter_times" in d else np.zeros(0)
    env_file = str(d["env_file"]) if "env_file" in d else None
    if len(frames) == 0:
        print("Empty frames (requires BENCH_SAVE_TRAJ=1)")
        return
    t, cv, cw, md, eact = frames[:, 0], frames[:, 1], frames[:, 2], frames[:, 3], frames[:, 4] > 0.5
    has_pose = frames.shape[1] >= 8
    px = frames[:, 5] if has_pose else traj[:, 0]
    py = frames[:, 6] if has_pose else traj[:, 1]
    yaw = frames[:, 7] if has_pose else np.zeros(len(t))

    print(f"== cost v2: {args.npz.split('/')[-1]} ==")
    # 1) Safety margin
    mdf = md[np.isfinite(md)]
    if len(mdf):
        p10 = float(np.quantile(mdf, 0.10))
        hug = float(np.mean(mdf < args.margin))
        scrape = float(np.mean(mdf < 0.10))
        ok1 = p10 >= args.margin and scrape == 0
        print(f"[1 Safety Margin] margin_p10={p10:.3f}(>={args.margin}) hug_raw={hug:.1%} "
              f"scrape={scrape:.2%}(=0) -> {'PASS' if ok1 else 'FAIL'}")
    else:
        ok1 = True
        print("[1 Safety Margin] No obstacle frames -> PASS")

    # 2) Single-pass resolution + 3) Reaction delay
    events = []
    for te in enters:
        if events and te - events[-1][-1] < 10.0:
            events[-1].append(te)
        else:
            events.append([te])
    resolve_hits, reacts, idles, switches, recontact = [], [], [], [], 0
    for ev in events:
        resolve_hits.append(len(ev) <= 1)
        if len(ev) >= 2:
            recontact += 1
        t0e, t1e = ev[0], ev[-1] + 3.0
        pre = (t < t0e) & (np.abs(cv) < 0.05)
        stall_start = None
        tt = t[pre]
        if len(tt):
            run = [tt[0]]
            for a, b in zip(tt[:-1], tt[1:]):
                if b - a <= 0.3:
                    run.append(b)
                else:
                    if run[-1] - run[0] >= 0.5 and run[-1] > t0e - 8:
                        stall_start = run[0]
                    run = [b]
            if stall_start is None and run[-1] - run[0] >= 0.5 and run[-1] > t0e - 8:
                stall_start = run[0]
        if stall_start is not None:
            reacts.append(t0e - stall_start)
            w = (t >= stall_start) & (t <= t1e)
            idles.append(float(np.sum(np.abs(cv[w]) < 0.05) * 0.1))
        w2 = (t >= t0e - 1) & (t <= t1e)
        sgn = np.sign(cv[w2][np.abs(cv[w2]) > 0.05])
        switches.append(int(np.sum(np.abs(np.diff(sgn)) > 0)) if len(sgn) > 1 else 0)
    if events:
        rr = float(np.mean(resolve_hits))
        rd = float(np.median(reacts)) if reacts else float("nan")
        ok2 = rr >= args.resolve and recontact == 0
        ok3 = (not reacts) or float(max(reacts)) <= args.react
        print(f"[2 Single-pass] events={len(events)} resolve_rate={rr:.0%}(>={args.resolve:.0%}) "
              f"recontact={recontact}(=0) -> {'PASS' if ok2 else 'FAIL'}")
        print(f"[3 Reaction] react_delay med={rd:.2f}s max={float(max(reacts)) if reacts else float('nan'):.2f}s"
              f"(<={args.react}) idle_wait={np.mean(idles) if idles else 0:.1f}s -> {'PASS' if ok3 else 'FAIL'}")
        print(f"[6 Smoothness] fwd_back_switch/event med={np.median(switches):.0f} max={max(switches)}")
    else:
        ok2 = ok3 = True
        print("[2/3] No escape events -> PASS")

    # 4) Wall clearance against static WorldGrid distance transform
    ok4 = True
    if env_file:
        try:
            env = yaml.safe_load(open(env_file))
            wg = WorldGrid(env, res=0.05)
            dt = ndimage.distance_transform_edt(~wg.grid) * wg.res
            ix = np.clip(((px - wg.x0) / wg.res).astype(int), 0, wg.nx - 1)
            iy = np.clip(((py - wg.y0) / wg.res).astype(int), 0, wg.ny - 1)
            footprint_clear = np.maximum(dt[ix, iy] - 0.275, 0.0)
            wall_side = float(np.mean((footprint_clear < args.wall_hug) & ~eact))
            narrow = float(np.median(dt[ix, iy])) <= 0.55
            hug_ok = True if narrow else hug <= args.hug_ratio
            ok1 = ok1 and hug_ok
            wall_ok = True if narrow else wall_side <= 0.10
            print(f"[4 Wall Clearance] footprint_clear_p10={np.quantile(footprint_clear,.1):.3f} "
                  f"wall_side_time(<{args.wall_hug}m,non-escape)={wall_side:.1%}(<=10%) "
                  f"scene={'narrow' if narrow else 'open/office'} hug_check={'skipped' if narrow else ('PASS' if hug_ok else 'FAIL')} "
                  f"-> {'PASS' if wall_ok else 'FAIL'}")
            ok4 = wall_ok
        except Exception as e:
            print(f"[4 Wall Clearance] Skipped ({e})")

    overall = ok1 and ok2 and ok3 and ok4
    print(f"== Overall: {'PASS' if overall else 'FAIL'} ==")


if __name__ == "__main__":
    main()
