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

"""PGM occupancy map downsampler.

Reduces the resolution of a PGM + YAML map by an integer factor N (default 2: 0.05m -> 0.10m)
while preserving ROS map_server threshold semantics:
  - occ = (255 - pixel) / 255
  - occ > occupied_thresh (0.65) -> occupied (0)
  - occ < free_thresh (0.196)    -> free (254)
  - otherwise                    -> unknown (205)
Merges NxN blocks with priority occupied > free > unknown, keeping the map origin unchanged.

Usage:
    python3 pgm_downsample.py <map.pgm> [--factor 2] [--output prefix] [--inplace]
"""

import argparse
import os
import shutil
import sys

import numpy as np


def read_pgm(filepath):
    """Read binary PGM (P5) and return (grid, width, height)."""
    with open(filepath, 'rb') as f:
        while True:
            ln = f.readline().strip()
            if not ln:
                raise ValueError("Unexpected EOF in PGM header")
            if ln.startswith(b'#') or ln.startswith(b'P5'):
                continue
            if b' ' in ln and ln.split()[0].isdigit():
                w, h = map(int, ln.split())
                break
        f.readline()  # maxval
        grid = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return grid, w, h


def classify(grid, occupied_thresh=0.65, free_thresh=0.196):
    """Classify grayscale values into (is_occ, is_free, is_unknown) boolean masks using map_server thresholds."""
    occ_ratio = (255.0 - grid.astype(np.float32)) / 255.0
    is_occ = occ_ratio > occupied_thresh
    is_free = occ_ratio < free_thresh
    is_unknown = ~is_occ & ~is_free
    return is_occ, is_free, is_unknown


def downsample(grid, factor, min_occ_count=1, fill_unknown=False,
               occupied_thresh=0.65, free_thresh=0.196):
    """Downsample occupancy grid by merging NxN blocks (priority: occupied > free > unknown)."""
    is_occ, is_free, is_unknown = classify(grid, occupied_thresh, free_thresh)
    if fill_unknown:
        is_free = is_free | is_unknown
        is_unknown = np.zeros_like(is_unknown)

    h, w = grid.shape
    nh, nw = h // factor, w // factor
    occ_b = is_occ[:nh * factor, :nw * factor].reshape(nh, factor, nw, factor)
    free_b = is_free[:nh * factor, :nw * factor].reshape(nh, factor, nw, factor)

    occ_count = occ_b.sum(axis=(1, 3))
    free_count = free_b.sum(axis=(1, 3))

    out = np.full((nh, nw), 205, dtype=np.uint8)
    out[free_count >= 1] = 254
    out[occ_count >= min_occ_count] = 0
    return out


def write_pgm(filepath, grid):
    h, w = grid.shape
    with open(filepath, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())


def main():
    parser = argparse.ArgumentParser(description='PGM occupancy map downsampler')
    parser.add_argument('input', help='Input PGM file path (requires matching .yaml in same directory)')
    parser.add_argument('--factor', type=int, default=2,
                        help='Downsampling factor (default: 2)')
    parser.add_argument('--min-occ-count', type=int, default=1,
                        help='Minimum occupied cells in NxN block to mark output cell occupied (default: 1)')
    parser.add_argument('--fill-unknown', action='store_true',
                        help='Treat unknown cells as free')
    parser.add_argument('-o', '--output', default=None,
                        help='Output prefix (default: <input_stem>_ds)')
    parser.add_argument('--inplace', action='store_true',
                        help='Overwrite input files (backs up originals to .bak)')
    args = parser.parse_args()

    if args.factor < 2:
        print("Error: --factor must be >= 2")
        sys.exit(1)

    pgm_in = args.input
    yaml_in = os.path.splitext(pgm_in)[0] + '.yaml'
    if not os.path.isfile(pgm_in) or not os.path.isfile(yaml_in):
        print(f"Error: {pgm_in} or its companion {yaml_in} not found")
        sys.exit(1)

    orig_res = None
    orig_origin = None
    orig_thresh = []
    occ_th, free_th = 0.65, 0.196
    with open(yaml_in) as f:
        for ln in f:
            ln = ln.strip()
            if ln.startswith('resolution:'):
                orig_res = float(ln.split(':', 1)[1].strip())
            elif ln.startswith('origin:'):
                import re
                m = re.search(r'\[([^]]+)\]', ln)
                if m:
                    orig_origin = [float(v) for v in m.group(1).split(',')]
            elif ln.startswith(('occupied_thresh', 'free_thresh')):
                orig_thresh.append(ln)
                try:
                    val = float(ln.split(':', 1)[1].strip())
                    if ln.startswith('occupied_thresh'):
                        occ_th = val
                    else:
                        free_th = val
                except ValueError:
                    pass
    if orig_res is None or orig_origin is None:
        print("Error: YAML is missing resolution or origin")
        sys.exit(1)

    grid, w, h = read_pgm(pgm_in)
    print(f"Input map: {w}x{h} = {w*h:,} pixels @ {orig_res} m/cell")
    o0, f0, u0 = classify(grid, occ_th, free_th)
    print(f"  Input distribution: occ={o0.sum():,} ({o0.mean()*100:.2f}%), "
          f"free={f0.sum():,} ({f0.mean()*100:.1f}%), "
          f"unknown={u0.sum():,} ({u0.mean()*100:.1f}%)"
          f"  [thresholds occ>{occ_th} / free<{free_th}]")
    new_res = orig_res * args.factor
    print(f"Target: @ {new_res} m/cell (factor {args.factor}x)")

    out = downsample(grid, args.factor, args.min_occ_count, args.fill_unknown,
                     occ_th, free_th)
    nw, nh = out.shape[1], out.shape[0]
    print(f"Output map: {nw}x{nh} = {nw*nh:,} pixels "
          f"({w*h/(nw*nh):.0f}x smaller)")
    o1, f1, u1 = classify(out, occ_th, free_th)
    print(f"  Output distribution: occ={o1.sum():,} ({o1.mean()*100:.2f}%), "
          f"free={f1.sum():,} ({f1.mean()*100:.1f}%), "
          f"unknown={u1.sum():,} ({u1.mean()*100:.1f}%)")

    if args.inplace:
        shutil.copy2(pgm_in, pgm_in + '.bak')
        shutil.copy2(yaml_in, yaml_in + '.bak')
        pgm_out, yaml_out = pgm_in, yaml_in
    else:
        prefix = args.output or os.path.splitext(pgm_in)[0] + '_ds'
        pgm_out, yaml_out = prefix + '.pgm', prefix + '.yaml'

    write_pgm(pgm_out, out)
    with open(yaml_out, 'w') as f:
        f.write(f'image: {os.path.basename(pgm_out)}\n')
        f.write(f'resolution: {new_res}\n')
        f.write(f'origin: [{orig_origin[0]:.6f}, {orig_origin[1]:.6f}, {orig_origin[2]}]\n')
        f.write('negate: 0\n')
        for ln in orig_thresh:
            f.write(ln + '\n')

    size_mb = os.path.getsize(pgm_out) / 1024 / 1024
    print(f"Output: {pgm_out} ({size_mb:.1f} MB) + {yaml_out}")
    print(f"Origin: {orig_origin}")
    if args.inplace:
        print("Backup: originals saved as .bak")


if __name__ == '__main__':
    main()
