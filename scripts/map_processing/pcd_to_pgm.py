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

"""Standalone PCD to PGM + YAML occupancy grid converter (no ROS dependency).

Slices a 3D PCD point cloud along the Z axis based on sensor mounting height
and projects it onto the XY plane to generate a 2D occupancy grid compatible
with ROS map_server:
  1. Read PCD point cloud (supports ascii, binary, and binary_compressed).
  2. Slice Z axis within [lidar_height - slice_below, lidar_height + slice_above].
  3. Mark all cells containing any point as free (254).
  4. Mark cells containing points within the Z slice as occupied (0).
  5. Leave unobserved cells as unknown (205).
  6. Write binary PGM (P5) and YAML configuration files.

Usage:
    python pcd_to_pgm.py <input.pcd> [options]
"""

import argparse
import os
import sys
import time

import numpy as np


def lzf_decompress_pure(compressed, uncompressed_size):
    """Pure-Python LZF decompressor (matching liblzf lzf_d.c)."""
    out = bytearray()
    ip = 0
    n = len(compressed)
    while ip < n and len(out) < uncompressed_size:
        ctrl = compressed[ip]
        ip += 1
        if ctrl < 32:
            out.extend(compressed[ip:ip + ctrl + 1])
            ip += ctrl + 1
        else:
            length = (ctrl >> 5) + 2
            if length == 9:
                length += compressed[ip]
                ip += 1
            offset = ((ctrl & 0x1f) << 8) + compressed[ip]
            ip += 1
            ref = len(out) - offset - 1
            if ref < 0:
                raise ValueError("Corrupted LZF data")
            for _ in range(length):
                out.append(out[ref])
                ref += 1
    if len(out) != uncompressed_size:
        raise ValueError("LZF decompressed size mismatch")
    return bytes(out)


def read_pcd(filepath):
    """Read a PCD file (ascii, binary, or binary_compressed) and return an Nx3 numpy array of XYZ coordinates."""
    with open(filepath, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Unexpected EOF in PCD header before DATA line")
            line_str = line.decode('ascii', errors='ignore').strip()
            if line_str.startswith('#') or line_str == '':
                continue
            if line_str.startswith('DATA'):
                parts = line_str.split()
                header['DATA'] = parts[1] if len(parts) > 1 else 'ascii'
                break
            parts = line_str.split(None, 1)
            if len(parts) == 2:
                header[parts[0]] = parts[1]

        num_points = int(header.get('POINTS', header.get('WIDTH', '0')))
        data_format = header.get('DATA', 'ascii').lower()

        fields = header.get('FIELDS', 'x y z').split()
        sizes = [int(s) for s in header.get('SIZE', '4 4 4').split()]
        types = header.get('TYPE', 'F F F').split()
        counts = [int(c) for c in header.get('COUNT', ' '.join(['1'] * len(fields))).split()]

        field_bytes = [s * c for s, c in zip(sizes, counts)]
        point_size = sum(field_bytes)

        cur = 0
        field_map = {}
        for i, fld in enumerate(fields):
            field_map[fld] = (cur, sizes[i], types[i])
            cur += field_bytes[i]

        xyz_info = []
        for ax in ('x', 'y', 'z'):
            if ax in field_map:
                xyz_info.append(field_map[ax])
            else:
                raise ValueError(f"PCD file is missing '{ax}' field")

        if data_format == 'ascii':
            lines = []
            for i in range(num_points):
                raw = f.readline()
                if not raw:
                    break
                lines.append(raw.decode('ascii', errors='ignore').strip())
            all_values = []
            for ln in lines:
                all_values.append([float(v) for v in ln.split()])
            if not all_values:
                return np.zeros((0, 3), dtype=np.float64)
            arr = np.array(all_values, dtype=np.float64)
            x_idx = fields.index('x')
            y_idx = fields.index('y')
            z_idx = fields.index('z')
            return arr[:, [x_idx, y_idx, z_idx]]

        elif data_format in ('binary', 'binary_compressed'):
            if data_format == 'binary_compressed':
                compressed_size_bytes = f.read(4)
                uncompressed_size_bytes = f.read(4)
                if len(compressed_size_bytes) < 4 or len(uncompressed_size_bytes) < 4:
                    raise ValueError("Incomplete binary_compressed PCD header")
                compressed_size = int.from_bytes(compressed_size_bytes, 'little')
                uncompressed_size = int.from_bytes(uncompressed_size_bytes, 'little')
                raw_data = f.read(compressed_size)

                try:
                    import lzf
                    raw_data = lzf.decompress(raw_data, uncompressed_size)
                except ImportError:
                    try:
                        raw_data = lzf_decompress_pure(raw_data, uncompressed_size)
                    except Exception as e:
                        print(f"Warning: binary_compressed decompression failed: {e}")
                        sys.exit(1)
                except Exception as e:
                    try:
                        raw_data = lzf_decompress_pure(raw_data, uncompressed_size)
                    except Exception as e2:
                        print(f"Warning: lzf decompression failed: {e}; fallback failed: {e2}")
                        sys.exit(1)

                dtype_map = {
                    (4, 'F'): np.float32, (8, 'F'): np.float64,
                    (4, 'U'): np.uint32,  (4, 'I'): np.int32,
                    (2, 'U'): np.uint16,  (2, 'I'): np.int16,
                    (1, 'U'): np.uint8,   (1, 'I'): np.int8,
                }
                points = np.zeros((num_points, 3), dtype=np.float64)
                col_offset = 0
                for i, fld in enumerate(fields):
                    sz = sizes[i]
                    dt = dtype_map.get((sz, types[i].upper()), np.float32)
                    col_bytes = sz * counts[i] * num_points
                    if fld in ('x', 'y', 'z'):
                        j = ('x', 'y', 'z').index(fld)
                        chunk = raw_data[col_offset:col_offset + col_bytes]
                        points[:, j] = np.frombuffer(chunk, dtype=dt).astype(np.float64)
                    col_offset += col_bytes
                return points

            else:
                raw_data = f.read(num_points * point_size)

            if len(raw_data) < num_points * point_size:
                print(f"  Warning: incomplete data, expected {num_points * point_size} bytes, "
                      f"got {len(raw_data)} bytes")
                num_points = len(raw_data) // point_size

            dtype_map = {
                (4, 'F'): np.float32, (8, 'F'): np.float64,
                (4, 'U'): np.uint32,  (4, 'I'): np.int32,
                (2, 'U'): np.uint16,  (2, 'I'): np.int16,
                (1, 'U'): np.uint8,   (1, 'I'): np.int8,
            }

            xyz_dtypes = []
            for off, sz, typ in xyz_info:
                dt = dtype_map.get((sz, typ.upper()))
                if dt is None:
                    raise ValueError(f"Unsupported field type: size={sz}, type={typ}")
                xyz_dtypes.append(dt)

            raw_arr = np.frombuffer(raw_data, dtype=np.uint8)
            actual_points = len(raw_arr) // point_size
            if actual_points < num_points:
                print(f"  Warning: expected {num_points} points, got {actual_points} points")
                num_points = actual_points
            raw_arr = raw_arr[:num_points * point_size].reshape(num_points, point_size)

            points = np.zeros((num_points, 3), dtype=np.float64)
            for j, (off, sz, typ) in enumerate(xyz_info):
                col_bytes = raw_arr[:, off:off + sz].tobytes()
                points[:, j] = np.frombuffer(col_bytes, dtype=xyz_dtypes[j]).astype(np.float64)
            return points
        else:
            raise ValueError(f"Unsupported PCD data format: {data_format}")


def convert_pcd_to_pgm(args):
    """Main conversion pipeline: PCD -> PGM + YAML."""
    t_start = time.time()

    # 1. Read PCD point cloud
    print(f"Reading PCD file: {args.input}")
    points = read_pcd(args.input)
    total_points = len(points)

    if total_points == 0:
        print("Error: point cloud is empty.")
        sys.exit(1)

    valid_coords = np.isfinite(points).all(axis=1)
    if not valid_coords.all():
        nan_count = np.sum(~valid_coords)
        print(f"  Filtered {nan_count} invalid (NaN/Inf) points")
        points = points[valid_coords]
        total_points = len(points)

    print(f"  Total points: {total_points:,}")

    # 2. Z-axis slice
    z_min = args.lidar_height - args.slice_below
    z_max = args.lidar_height + args.slice_above
    print(f"  Z-axis slice range: [{z_min:.3f}, {z_max:.3f}] m")

    z_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    sliced_points = points[z_mask]
    slice_point_count = len(sliced_points)
    print(f"  Points in slice: {slice_point_count:,} ({slice_point_count / total_points * 100:.1f}%)")

    if slice_point_count == 0:
        print("Error: no points within Z slice range. Check --lidar-height / --slice-above / --slice-below.")
        sys.exit(1)

    # 3. Compute map bounds
    x_min_all = float(points[:, 0].min())
    x_max_all = float(points[:, 0].max())
    y_min_all = float(points[:, 1].min())
    y_max_all = float(points[:, 1].max())

    padding = 1.0

    if args.use_origin_center:
        x_min = min(x_min_all, 0.0) - padding
        x_max = max(x_max_all, 0.0) + padding
        y_min = min(y_min_all, 0.0) - padding
        y_max = max(y_max_all, 0.0) + padding
        print(f"  use_origin_center=true: grid includes origin (0,0)")
        print(f"  Point cloud bbox: [{x_min_all:.2f}, {x_max_all:.2f}] x "
              f"[{y_min_all:.2f}, {y_max_all:.2f}]")
        print(f"  Grid bounds:      [{x_min:.2f}, {x_max:.2f}] x "
              f"[{y_min:.2f}, {y_max:.2f}]")
    else:
        x_min = x_min_all - padding
        x_max = x_max_all + padding
        y_min = y_min_all - padding
        y_max = y_max_all + padding
        print(f"  use_origin_center=false: using bbox min corner")

    # 4. Compute grid dimensions
    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))

    print(f"  Map dimensions: {width} x {height} (cells)")
    print(f"  Physical size:  {width * args.resolution:.2f} x {height * args.resolution:.2f} (m)")

    # 5. Populate occupancy grid (205=unknown, 254=free, 0=occupied)
    grid = np.full((height, width), 205, dtype=np.uint8)

    gx_all = ((points[:, 0] - x_min) / args.resolution).astype(np.int32)
    gy_all = ((points[:, 1] - y_min) / args.resolution).astype(np.int32)
    valid_all = (gx_all >= 0) & (gx_all < width) & (gy_all >= 0) & (gy_all < height)
    grid[gy_all[valid_all], gx_all[valid_all]] = 254

    gx_slice = ((sliced_points[:, 0] - x_min) / args.resolution).astype(np.int32)
    gy_slice = ((sliced_points[:, 1] - y_min) / args.resolution).astype(np.int32)
    valid_slice = (gx_slice >= 0) & (gx_slice < width) & (gy_slice >= 0) & (gy_slice < height)
    grid[gy_slice[valid_slice], gx_slice[valid_slice]] = 0

    occupied_count = int(np.sum(grid == 0))
    free_count = int(np.sum(grid == 254))
    unknown_count = int(np.sum(grid == 205))
    total_cells = width * height

    # 6. Write binary PGM (P5)
    pgm_path = args.output + '.pgm'
    print(f"\nWriting PGM: {pgm_path}")

    with open(pgm_path, 'wb') as f:
        header = f'P5\n{width} {height}\n255\n'.encode('ascii')
        f.write(header)
        f.write(np.flipud(grid).tobytes())

    pgm_size_kb = os.path.getsize(pgm_path) / 1024.0

    # 7. Write YAML configuration
    yaml_path = args.output + '.yaml'
    pgm_filename = os.path.basename(pgm_path)
    print(f"Writing YAML: {yaml_path}")

    with open(yaml_path, 'w') as f:
        f.write(f'image: {pgm_filename}\n')
        f.write(f'resolution: {args.resolution}\n')
        f.write(f'origin: [{x_min:.6f}, {y_min:.6f}, 0.0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: {args.occupied_thresh}\n')
        f.write(f'free_thresh: {args.free_thresh}\n')

    # 8. Print summary statistics
    t_elapsed = time.time() - t_start
    print()
    print("=" * 60)
    print("  Conversion Complete - Map Statistics")
    print("=" * 60)
    print(f"  Input file:      {args.input}")
    print(f"  Output PGM:      {pgm_path} ({pgm_size_kb:.1f} KB)")
    print(f"  Output YAML:     {yaml_path}")
    print(f"  {'-' * 56}")
    print(f"  Total points:    {total_points:>14,}")
    print(f"  Sliced points:   {slice_point_count:>14,}"
          f"  ({slice_point_count / total_points * 100:.1f}%)")
    print(f"  {'-' * 56}")
    print(f"  Resolution:      {args.resolution:.4f} m/cell")
    print(f"  Dimensions:      {width} x {height} cells")
    print(f"  Physical range:  {width * args.resolution:.2f} x"
          f" {height * args.resolution:.2f} m")
    print(f"  Area:            {width * height * args.resolution ** 2:.2f} m^2")
    print(f"  {'-' * 56}")
    print(f"  Occupied (0):    {occupied_count:>14,}"
          f"  ({occupied_count / total_cells * 100:.1f}%)")
    print(f"  Free (254):      {free_count:>14,}"
          f"  ({free_count / total_cells * 100:.1f}%)")
    print(f"  Unknown (205):   {unknown_count:>14,}"
          f"  ({unknown_count / total_cells * 100:.1f}%)")
    print(f"  {'-' * 56}")
    print(f"  Z slice range:   [{z_min:.3f}, {z_max:.3f}] m")
    print(f"  Map origin:      ({x_min:.4f}, {y_min:.4f})")
    if args.use_origin_center:
        origin_gx = int((0.0 - x_min) / args.resolution)
        origin_gy = int((0.0 - y_min) / args.resolution)
        print(f"  Origin (0,0) cell: ({origin_gx}, {origin_gy})")
    print(f"  {'-' * 56}")
    print(f"  Elapsed:         {t_elapsed:.2f} s")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Standalone PCD to PGM + YAML occupancy grid converter (compatible with map_server)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('input',
                        help='Input PCD file path')
    parser.add_argument('-o', '--output', default=None,
                        help='Output file prefix (default: input stem)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05,
                        help='Grid resolution in meters (default: 0.05)')
    parser.add_argument('--lidar-height', type=float, default=0.545,
                        help='Sensor mounting height in meters (default: 0.545)')
    parser.add_argument('--slice-above', type=float, default=1.5,
                        help='Slice height above sensor in meters (default: 1.5)')
    parser.add_argument('--slice-below', type=float, default=0.2,
                        help='Slice depth below sensor in meters (default: 0.2)')
    parser.add_argument('--occupied-thresh', type=float, default=0.65,
                        help='Occupancy threshold (default: 0.65)')
    parser.add_argument('--free-thresh', type=float, default=0.196,
                        help='Free threshold (default: 0.196)')
    parser.add_argument('--use-origin-center', dest='use_origin_center',
                        action='store_true', default=True,
                        help='Ensure grid bounds include coordinate origin (0,0) (default: enabled)')
    parser.add_argument('--no-origin-center', dest='use_origin_center',
                        action='store_false',
                        help='Use bounding box min corner as origin')

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: input file does not exist: {args.input}")
        sys.exit(1)

    if args.output is None:
        args.output = os.path.splitext(args.input)[0]

    if args.resolution <= 0:
        print("Error: --resolution must be > 0")
        sys.exit(1)
    if args.lidar_height < 0:
        print("Error: --lidar-height must be >= 0")
        sys.exit(1)
    if args.slice_above < 0 or args.slice_below < 0:
        print("Error: --slice-above / --slice-below must be >= 0")
        sys.exit(1)
    if args.occupied_thresh < 0 or args.occupied_thresh > 1:
        print("Error: --occupied-thresh must be in [0, 1]")
        sys.exit(1)
    if args.free_thresh < 0 or args.free_thresh > 1:
        print("Error: --free-thresh must be in [0, 1]")
        sys.exit(1)

    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    convert_pcd_to_pgm(args)


if __name__ == '__main__':
    main()
