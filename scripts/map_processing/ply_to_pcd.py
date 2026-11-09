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

"""Convert PLY point clouds to binary PCD format (with optional 3D Gaussian Splatting filtering).

Supports:
  - Standard PLY files (x, y, z)
  - 3DGS PLY exports (with opacity/scale/rot attributes), filtering out low-opacity
    floaters and oversized Gaussians before writing binary XYZ PCD.

Usage:
    python ply_to_pcd.py input.ply
    python ply_to_pcd.py input.ply --min-opacity 0.1 --max-scale 0.2
    python ply_to_pcd.py input.ply --no-filter
"""

import argparse
import os
import sys
import time

import numpy as np


def read_ply_header(filepath):
    """Parse PLY header and return (fields, field_bytes, num_points, data_offset)."""
    with open(filepath, 'rb') as f:
        header_lines = []
        offset = 0
        while True:
            line = f.readline()
            offset += len(line)
            if not line:
                raise ValueError("Unexpected EOF in PLY header before end_header")
            header_lines.append(line.decode('ascii', errors='ignore').strip())
            if header_lines[-1].startswith('end_header'):
                break

    fields = []
    num_points = 0
    for ln in header_lines:
        parts = ln.split()
        if not parts:
            continue
        if parts[0] == 'element' and parts[1] == 'vertex':
            num_points = int(parts[2])
        elif parts[0] == 'property':
            if parts[1] == 'list':
                continue
            fields.append((parts[2], parts[1]))

    if num_points == 0:
        raise ValueError("PLY header does not declare vertex count")

    type_size = {
        'char': 1, 'int8': 1,
        'uchar': 1, 'uint8': 1,
        'short': 2, 'int16': 2,
        'ushort': 2, 'uint16': 2,
        'int': 4, 'int32': 4,
        'uint': 4, 'uint32': 4,
        'float': 4, 'float32': 4,
        'double': 8, 'float64': 8,
    }
    field_bytes = []
    for name, typ in fields:
        if typ not in type_size:
            raise ValueError(f"Unsupported property type: {typ}")
        field_bytes.append(type_size[typ])

    return fields, field_bytes, num_points, offset


def read_ply_points(filepath):
    """Read PLY vertex records and return (field_names, N x K float64 array)."""
    fields, field_bytes, num_points, data_offset = read_ply_header(filepath)
    point_size = sum(field_bytes)
    total_bytes = point_size * num_points

    with open(filepath, 'rb') as f:
        f.seek(data_offset)
        raw = f.read(total_bytes)

    actual = len(raw) // point_size
    if actual < num_points:
        print(f"  Warning: incomplete data, expected {num_points} points, got {actual} points")
        num_points = actual

    arr = np.frombuffer(raw[:num_points * point_size], dtype=np.uint8)
    arr = arr.reshape(num_points, point_size)

    names = [n for n, _ in fields]
    dtype_map = {4: np.float32, 8: np.float64}
    out = np.zeros((num_points, len(fields)), dtype=np.float64)
    col = 0
    for i, (name, typ) in enumerate(fields):
        sz = field_bytes[i]
        dt = dtype_map.get(sz, np.float32)
        col_bytes = arr[:, col:col + sz].tobytes()
        out[:, i] = np.frombuffer(col_bytes, dtype=dt).astype(np.float64)
        col += sz
    return names, out


def filter_gaussians(names, data, min_opacity, max_scale, z_range, x_range, y_range):
    """Filter 3DGS points by opacity, maximum scale, and X/Y/Z bounds."""
    keep = np.ones(len(data), dtype=bool)

    if 'opacity' in names:
        idx = names.index('opacity')
        if min_opacity is not None:
            n_before = int(keep.sum())
            op_p = 1.0 / (1.0 + np.exp(-data[:, idx]))
            keep &= op_p >= min_opacity
            print(f"  sigmoid(opacity) >= {min_opacity}: "
                  f"{n_before - int(keep.sum()):,} points filtered")
    elif min_opacity is not None:
        print("  Warning: PLY has no opacity field, skipping opacity filter")

    if max_scale is not None and 'scale_0' in names:
        idx0 = names.index('scale_0')
        idx1 = names.index('scale_1')
        idx2 = names.index('scale_2')
        n_before = int(keep.sum())
        scale_max = np.exp(np.maximum(np.maximum(data[:, idx0], data[:, idx1]), data[:, idx2]))
        keep &= scale_max <= max_scale
        print(f"  exp(scale_max) <= {max_scale} m: {n_before - int(keep.sum()):,} points filtered")
    elif max_scale is not None:
        print("  Warning: PLY has no scale_* fields, skipping scale filter")

    if z_range is not None:
        z_min, z_max = z_range
        n_before = int(keep.sum())
        z = data[:, names.index('z')]
        keep &= (z >= z_min) & (z <= z_max)
        print(f"  Z in [{z_min}, {z_max}] m: {n_before - int(keep.sum()):,} points filtered")

    if x_range is not None:
        x_min, x_max = x_range
        n_before = int(keep.sum())
        x = data[:, names.index('x')]
        keep &= (x >= x_min) & (x <= x_max)
        print(f"  X in [{x_min}, {x_max}] m: {n_before - int(keep.sum()):,} points filtered")

    if y_range is not None:
        y_min, y_max = y_range
        n_before = int(keep.sum())
        y = data[:, names.index('y')]
        keep &= (y >= y_min) & (y <= y_max)
        print(f"  Y in [{y_min}, {y_max}] m: {n_before - int(keep.sum()):,} points filtered")

    return data[keep]


def write_pcd_binary(filepath, xyz):
    """Write binary PCD file (FIELDS x y z)."""
    n = len(xyz)
    with open(filepath, 'wb') as f:
        header = (f"# .PCD v0.7 - Point Cloud Data file format\n"
                  f"VERSION 0.7\n"
                  f"FIELDS x y z\n"
                  f"SIZE 4 4 4\n"
                  f"TYPE F F F\n"
                  f"COUNT 1 1 1\n"
                  f"WIDTH {n}\n"
                  f"HEIGHT 1\n"
                  f"VIEWPOINT 0 0 0 1 0 0 0\n"
                  f"POINTS {n}\n"
                  f"DATA binary\n").encode('ascii')
        f.write(header)
        f.write(xyz.astype(np.float32).tobytes())


def main():
    parser = argparse.ArgumentParser(
        description='PLY to PCD converter (with 3DGS Gaussian filtering)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input', help='Input PLY file path')
    parser.add_argument('-o', '--output', default=None,
                        help='Output PCD file path (default: <input_stem>.pcd)')
    parser.add_argument('--min-opacity', type=float, default=0.1,
                        help='Minimum sigmoid(opacity) threshold in [0, 1] (default: 0.1)')
    parser.add_argument('--max-scale', type=float, default=1.0,
                        help='Maximum exp(scale) threshold in meters (default: 1.0)')
    parser.add_argument('--z-range', type=float, nargs=2, metavar=('Z_MIN', 'Z_MAX'),
                        default=None,
                        help='Z range crop, e.g. --z-range -5 20')
    parser.add_argument('--x-range', type=float, nargs=2, metavar=('X_MIN', 'X_MAX'),
                        default=None,
                        help='X range crop, e.g. --x-range -100 200')
    parser.add_argument('--y-range', type=float, nargs=2, metavar=('Y_MIN', 'Y_MAX'),
                        default=None,
                        help='Y range crop, e.g. --y-range -100 300')
    parser.add_argument('--no-filter', action='store_true',
                        help='Disable all point filtering')
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: input file does not exist: {args.input}")
        sys.exit(1)

    if args.no_filter:
        args.min_opacity = None
        args.max_scale = None
        args.z_range = None
        args.x_range = None
        args.y_range = None

    if args.output is None:
        args.output = os.path.splitext(args.input)[0] + '.pcd'

    t0 = time.time()
    print(f"Reading PLY: {args.input}")
    names, data = read_ply_points(args.input)
    total = len(data)
    print(f"  Total points: {total:,}, fields: {names}")

    if 'x' not in names or 'y' not in names or 'z' not in names:
        print("Error: PLY is missing x/y/z fields")
        sys.exit(1)

    if (args.min_opacity is not None or args.max_scale is not None
            or args.z_range is not None or args.x_range is not None
            or args.y_range is not None):
        data = filter_gaussians(names, data, args.min_opacity, args.max_scale,
                                args.z_range, args.x_range, args.y_range)
        print(f"  Points after filtering: {len(data):,} ({len(data) / total * 100:.1f}%)")

    if len(data) == 0:
        print("Error: no points remaining after filtering; lower thresholds or use --no-filter")
        sys.exit(1)

    xyz = data[:, [names.index('x'), names.index('y'), names.index('z')]]
    print(f"  Z range: [{xyz[:, 2].min():.2f}, {xyz[:, 2].max():.2f}] m")
    print(f"  X range: [{xyz[:, 0].min():.2f}, {xyz[:, 0].max():.2f}] m")
    print(f"  Y range: [{xyz[:, 1].min():.2f}, {xyz[:, 1].max():.2f}] m")

    print(f"Writing PCD: {args.output}")
    write_pcd_binary(args.output, xyz)
    size_mb = os.path.getsize(args.output) / 1024 / 1024
    print(f"  Output size: {size_mb:.1f} MB")
    print(f"  Elapsed:     {time.time() - t0:.1f} s")


if __name__ == '__main__':
    main()
