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

"""Voxel-persistence and near-field blind-spot point cloud filter core (pure NumPy + 16-byte PointCloud2 packing).

- Voxel key: q = floor(xyz * (1.0 / res)), key = (q.x + 2^20) << 42 | (q.y + 2^20) << 21 | (q.z + 2^20)
- Frame set and sliding window count: tracks unique voxels for (active & ~near) points across a FIFO window;
  points with count(v) >= persist are classified as persistent.
- Outputs:
  * cloud_filtered: persistent & ~near (preserves original rgb)
  * cloud_single_shot: ~persistent & ~near (red 0x00FF0000)
  * cloud_near: near (yellow 0x00FFFF00)
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

FLOAT32 = PointField.FLOAT32

COLOR_NEAR_UINT32 = np.uint32(0x00FFFF00)
COLOR_SINGLE_SHOT_UINT32 = np.uint32(0x00FF0000)
COLOR_DEFAULT_UINT32 = np.uint32(0x00FFFFFF)

COLOR_NEAR_RGB_F32 = COLOR_NEAR_UINT32.view(np.float32)
COLOR_SINGLE_SHOT_RGB_F32 = COLOR_SINGLE_SHOT_UINT32.view(np.float32)
COLOR_DEFAULT_RGB_F32 = COLOR_DEFAULT_UINT32.view(np.float32)

OUTPUT_FIELDS = [
    PointField(name="x", offset=0, datatype=FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=FLOAT32, count=1),
]


def unpack_cloud(msg):
    """Read x/y/z and optional rgb (as uint32 bits) from a PointCloud2 message."""
    width = int(msg.width)
    height = int(msg.height)
    n = width * height
    step = int(msg.point_step)
    row_step = int(msg.row_step) if int(msg.row_step) > 0 else width * step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if step < 12 or row_step < width * step or raw.size < height * row_step:
        return None
    off = {f.name: int(f.offset) for f in msg.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return None
    if n == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint32)
    endian = ">" if msg.is_bigendian else "<"
    dt = np.dtype({
        "names": ["x", "y", "z", "rgb"],
        "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4", f"{endian}u4"],
        "offsets": [off["x"], off["y"], off["z"], off.get("rgb", off["x"])],
        "itemsize": step,
    })
    if raw.size == n * step:
        arr = raw.view(dt)
    else:
        rows = raw[:height * row_step].reshape(height, row_step)
        payload = np.ascontiguousarray(rows[:, :width * step]).reshape(-1)
        arr = payload.view(dt)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    rgb = arr["rgb"].astype(np.uint32, copy=True) if "rgb" in off else np.full(n, 0xFFFFFF, dtype=np.uint32)
    return xyz, rgb


def parse_pointcloud_xyz_rgb(msg):
    """Read (xyz, rgb_f32) from PointCloud2 or raise ValueError."""
    data = unpack_cloud(msg)
    if data is None:
        raise ValueError("PointCloud2 missing valid x/y/z fields or invalid byte length")
    xyz, rgb_u32 = data
    return xyz, rgb_u32.view(np.float32)


def pack_cloud(header, xyz, rgb):
    """Pack xyz (N, 3) and rgb (uint32 or float32) into a 16-byte/point PointCloud2 message."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    n = int(xyz.shape[0])
    data = np.empty((n, 4), dtype=np.float32)
    if n > 0:
        data[:, :3] = xyz
        rgb_arr = np.asarray(rgb)
        if np.issubdtype(rgb_arr.dtype, np.floating):
            data[:, 3] = rgb_arr.astype(np.float32)
        else:
            data[:, 3] = rgb_arr.astype(np.uint32).view(np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = list(OUTPUT_FIELDS)
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


pack_xyz_rgb_cloud = pack_cloud


def voxel_keys(xyz, res):
    q = np.floor(np.asarray(xyz) * (1.0 / res)).astype(np.int64)
    q += 1 << 20
    np.clip(q, 0, (1 << 21) - 1, out=q)
    return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]


class PersistVoxelFilter:
    """Voxel-persistence and near-field blind-spot point cloud filter core."""

    def __init__(self, res=0.02, window=5, persist=2,
                 near_radius=0.5, active_radius=2.0):
        if not np.isfinite([res, near_radius, active_radius]).all():
            raise ValueError("res, near_radius, active_radius must be finite")
        if res <= 0.0:
            raise ValueError("res must be > 0")
        if (isinstance(window, bool) or isinstance(persist, bool)
                or int(window) != window or int(persist) != persist):
            raise ValueError("window and persist must be integers")
        window = int(window)
        persist = int(persist)
        if window < 2:
            raise ValueError("window must be >= 2")
        if persist < 2 or persist > window:
            raise ValueError("persist must be >= 2 and <= window")
        if near_radius > 0.0 and active_radius > 0.0 and active_radius < near_radius:
            raise ValueError("active_radius must be >= near_radius when both are > 0")

        self.res = float(res)
        self.window = window
        self.persist = persist
        self.near_radius = float(near_radius) if near_radius > 0.0 else 0.0
        self.active_radius = float(active_radius) if active_radius > 0.0 else 0.0
        self.history = []

    def reset(self):
        self.history.clear()

    @staticmethod
    def validate_pose(matrix):
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("sensor_to_fixed must be a finite 4x4 rigid transform")
        rotation = matrix[:3, :3]
        if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7, rtol=0)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
            raise ValueError("sensor_to_fixed must contain a proper rotation")
        return matrix

    voxel_keys = staticmethod(voxel_keys)

    def classify(self, xyz, odom_xy=None, enabled=True):
        """Return (filtered_mask, single_shot_mask, near_mask, stats) matching the length of xyz."""
        xyz = np.asarray(xyz)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("xyz must have shape (N, 3)")
        n_total = int(xyz.shape[0])
        with np.errstate(invalid="ignore"):
            finite = np.isfinite(xyz).all(axis=1)
        finite_idx = np.flatnonzero(finite)
        n = int(finite_idx.size)

        filtered_mask = np.zeros(n_total, dtype=bool)
        single_shot_mask = np.zeros(n_total, dtype=bool)
        near_mask = np.zeros(n_total, dtype=bool)

        if not enabled:
            self.reset()
            filtered_mask[finite_idx] = True
            stats = dict(
                input_points=n_total,
                valid_points=n,
                near_points=0,
                tracked_points=0,
                far_points=n,
                persistent_points=0,
                single_shot_points=0,
                filtered_points=n,
                removed_points=0,
                roi_points=0,
                kept_roi_points=0,
                window_frames=0,
                unique_window_voxels=0,
                has_odom=odom_xy is not None,
                warmup=True,
                reset_reason="",
            )
            return filtered_mask, single_shot_mask, near_mask, stats

        has_odom = odom_xy is not None
        pts = xyz[finite_idx]
        if has_odom:
            ox, oy = float(odom_xy[0]), float(odom_xy[1])
            if not (np.isfinite(ox) and np.isfinite(oy)):
                raise ValueError("odom_xy must be finite")
            if n > 0:
                d = pts[:, :2].astype(np.float64) - np.array([ox, oy], dtype=np.float64)
                d2 = np.einsum("ij,ij->i", d, d)
                near = d2 < (self.near_radius ** 2) if self.near_radius > 0.0 else np.zeros(n, dtype=bool)
                active = d2 <= (self.active_radius ** 2) if self.active_radius > 0.0 else np.ones(n, dtype=bool)
            else:
                near = np.zeros(0, dtype=bool)
                active = np.ones(0, dtype=bool)
        else:
            near = np.zeros(n, dtype=bool)
            active = np.ones(n, dtype=bool)

        tracked = active & ~near
        idx = np.flatnonzero(tracked)
        persistent = np.ones(n, dtype=bool)
        if idx.size:
            keys = voxel_keys(pts[idx], self.res)
            self.history.append(np.unique(keys))
            while len(self.history) > self.window:
                self.history.pop(0)
            uniq, hits = np.unique(np.concatenate(self.history), return_counts=True)
            pos = np.minimum(np.searchsorted(uniq, keys), uniq.size - 1)
            count = np.where(uniq[pos] == keys, hits[pos], 0)
            persistent[idx] = count >= self.persist
            unique_window_voxels = int(uniq.size)
        else:
            self.history.append(np.empty(0, dtype=np.int64))
            while len(self.history) > self.window:
                self.history.pop(0)
            non_empty = [s for s in self.history if s.size > 0]
            unique_window_voxels = int(np.unique(np.concatenate(non_empty)).size) if non_empty else 0

        keep = persistent & ~near
        single = ~persistent & ~near

        filtered_mask[finite_idx[keep]] = True
        single_shot_mask[finite_idx[single]] = True
        near_mask[finite_idx[near]] = True

        near_count = int(near.sum())
        tracked_count = int(idx.size)
        far_count = int((~active & ~near).sum())
        single_count = int(single.sum())
        persistent_count = tracked_count - single_count
        kept_count = int(keep.sum())

        stats = dict(
            input_points=n_total,
            valid_points=n,
            near_points=near_count,
            tracked_points=tracked_count,
            far_points=far_count,
            persistent_points=persistent_count,
            single_shot_points=single_count,
            filtered_points=kept_count,
            removed_points=near_count + single_count,
            roi_points=near_count + tracked_count,
            kept_roi_points=persistent_count,
            window_frames=len(self.history),
            unique_window_voxels=unique_window_voxels,
            has_odom=has_odom,
            warmup=len(self.history) < self.persist,
            reset_reason="",
        )
        return filtered_mask, single_shot_mask, near_mask, stats

    def process(self, xyz, odom_xy=None, enabled=True):
        filtered_mask, _, _, stats = self.classify(xyz, odom_xy=odom_xy, enabled=enabled)
        return filtered_mask, stats


SlamVoxelFilter = PersistVoxelFilter
