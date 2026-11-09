#!/usr/bin/env python3
"""SW01 cloud_slam geometric temporal filter core (ROS-independent, NumPy only)."""
import numpy as np


def _voxel_keys(xyz, size):
    q = np.floor(xyz / size).astype(np.int64) + (1 << 20)
    return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]


def _bin_keys(points, bin_rad):
    azimuth = np.arctan2(points[:, 1], points[:, 0])
    elevation = np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1]))
    ia = np.floor((azimuth + np.pi) / bin_rad).astype(np.int64)
    ie = np.floor((elevation + np.pi / 2.0) / bin_rad).astype(np.int64)
    return (ia << 20) | ie


def _rotation_deg(transform):
    trace = transform[0, 0] + transform[1, 1] + transform[2, 2]
    return np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))


def _norm_sq(points):
    return np.einsum("ij,ij->i", points, points)


def _transform(points, rotation, translation):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return np.column_stack((
        x * rotation[0, 0] + y * rotation[1, 0] + z * rotation[2, 0],
        x * rotation[0, 1] + y * rotation[1, 1] + z * rotation[2, 1],
        x * rotation[0, 2] + y * rotation[1, 2] + z * rotation[2, 2],
    )) + translation


class SlamVoxelFilter:
    def __init__(self, roi_radius_m=0.8, voxel_size_m=0.03,
                 min_range_drop_m=0.10, angular_bin_deg=0.5,
                 max_frame_gap_sec=0.30, pose_jump_m=0.5, pose_jump_deg=30.0):
        values = {
            "roi_radius_m": roi_radius_m,
            "voxel_size_m": voxel_size_m,
            "min_range_drop_m": min_range_drop_m,
            "angular_bin_deg": angular_bin_deg,
            "max_frame_gap_sec": max_frame_gap_sec,
            "pose_jump_m": pose_jump_m,
            "pose_jump_deg": pose_jump_deg,
        }
        for name, value in values.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be finite and > 0" % name)
        self.roi_radius_m = float(roi_radius_m)
        self.voxel_size_m = float(voxel_size_m)
        self.min_range_drop_m = float(min_range_drop_m)
        self.bin_rad = np.deg2rad(float(angular_bin_deg))
        self.max_frame_gap_sec = float(max_frame_gap_sec)
        self.pose_jump_m = float(pose_jump_m)
        self.pose_jump_deg = float(pose_jump_deg)
        self._prev_xyz = None
        self._prev_stamp_ns = None
        self._prev_sensor_to_fixed = None
        self._total_removed = 0

    def reset(self):
        self._prev_xyz = None
        self._prev_stamp_ns = None
        self._prev_sensor_to_fixed = None

    def process(self, xyz, sensor_to_fixed, stamp_ns=None, enabled=True):
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("xyz must have shape (N, 3)")
        count = len(xyz)
        stats = {
            "input_points": count,
            "valid_points": 0,
            "roi_points": 0,
            "kept_roi_points": 0,
            "sudden_near_points": 0,
            "removed_points": 0,
            "total_removed_points": self._total_removed,
            "warmup": False,
            "reset_reason": "",
        }
        if not enabled:
            self.reset()
            stats["warmup"] = True
            stats["reset_reason"] = "disabled"
            return np.ones(count, dtype=bool), stats

        transform = np.asarray(sensor_to_fixed, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            self.reset()
            stats["warmup"] = True
            stats["reset_reason"] = "invalid_transform"
            return np.ones(count, dtype=bool), stats

        valid = np.isfinite(xyz).all(axis=1) & (_norm_sq(xyz) > 0.0025)
        stats["valid_points"] = int(valid.sum())

        if self._prev_xyz is None:
            self._store(xyz, transform, stamp_ns)
            stats["warmup"] = True
            stats["reset_reason"] = "first_frame"
            return np.ones(count, dtype=bool), stats

        if stamp_ns is not None and self._prev_stamp_ns is not None:
            dt = (stamp_ns - self._prev_stamp_ns) / 1e9
            if dt <= 0.0:
                return self._restart(xyz, transform, stamp_ns, stats, "non_monotonic_stamp")
            if dt > self.max_frame_gap_sec:
                return self._restart(xyz, transform, stamp_ns, stats, "frame_gap")

        delta = np.linalg.inv(self._prev_sensor_to_fixed) @ transform
        if (np.linalg.norm(delta[:3, 3]) > self.pose_jump_m
                or _rotation_deg(delta) > self.pose_jump_deg):
            return self._restart(xyz, transform, stamp_ns, stats, "pose_jump")

        to_sensor = np.linalg.inv(transform)
        with np.errstate(invalid="ignore"):
            current = _transform(xyz, to_sensor[:3, :3], to_sensor[:3, 3])
            previous = _transform(self._prev_xyz, to_sensor[:3, :3], to_sensor[:3, 3])

        horizontal = np.hypot(current[:, 0], current[:, 1])
        roi = valid & (current[:, 0] > 0.0) & (horizontal <= self.roi_radius_m)
        stats["roi_points"] = int(roi.sum())
        if not roi.any():
            self._store(xyz, transform, stamp_ns)
            return np.ones(count, dtype=bool), stats

        previous_valid = (np.isfinite(self._prev_xyz).all(axis=1)
                          & (_norm_sq(self._prev_xyz) > 0.0025))
        previous_keys = np.sort(_voxel_keys(self._prev_xyz[previous_valid], self.voxel_size_m))
        current_keys = _voxel_keys(xyz[roi], self.voxel_size_m)
        if len(previous_keys):
            position = np.searchsorted(previous_keys, current_keys)
            clipped = np.clip(position, 0, len(previous_keys) - 1)
            voxel_hit = (position < len(previous_keys)) & (previous_keys[clipped] == current_keys)
        else:
            voxel_hit = np.zeros(len(current_keys), dtype=bool)

        previous_ranges = np.sqrt(_norm_sq(previous))
        direction_valid = previous_valid & np.isfinite(previous).all(axis=1)
        previous_points = previous[direction_valid]
        previous_ranges = previous_ranges[direction_valid]
        previous_bins = _bin_keys(previous_points, self.bin_rad)
        if len(previous_bins):
            order = np.argsort(previous_bins)
            sorted_bins = previous_bins[order]
            sorted_ranges = previous_ranges[order]
            starts = np.flatnonzero(np.r_[True, sorted_bins[1:] != sorted_bins[:-1]])
            unique_bins = sorted_bins[starts]
            nearest = np.minimum.reduceat(sorted_ranges, starts)
        else:
            unique_bins = np.empty(0, dtype=np.int64)
            nearest = np.empty(0, dtype=np.float64)

        current_roi = current[roi]
        current_ranges = np.sqrt(_norm_sq(current_roi))
        current_bins = _bin_keys(current_roi, self.bin_rad)
        if len(unique_bins):
            position = np.searchsorted(unique_bins, current_bins)
            clipped = np.clip(position, 0, len(unique_bins) - 1)
            supported = (position < len(unique_bins)) & (unique_bins[clipped] == current_bins)
            sudden = supported & ((nearest[clipped] - current_ranges) >= self.min_range_drop_m)
        else:
            sudden = np.zeros(len(current_bins), dtype=bool)

        delete = ~voxel_hit & sudden
        stats["sudden_near_points"] = int(sudden.sum())
        stats["removed_points"] = int(delete.sum())
        stats["kept_roi_points"] = stats["roi_points"] - stats["removed_points"]
        self._total_removed += int(delete.sum())
        stats["total_removed_points"] = self._total_removed

        keep = np.ones(count, dtype=bool)
        keep[np.flatnonzero(roi)[delete]] = False
        self._store(xyz, transform, stamp_ns)
        return keep, stats

    def _restart(self, xyz, transform, stamp_ns, stats, reason):
        self._store(xyz, transform, stamp_ns)
        stats["warmup"] = True
        stats["reset_reason"] = reason
        return np.ones(len(xyz), dtype=bool), stats

    def _store(self, xyz, transform, stamp_ns):
        self._prev_xyz = xyz.copy()
        self._prev_sensor_to_fixed = transform.copy()
        self._prev_stamp_ns = stamp_ns
