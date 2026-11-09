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

"""Near-field transient noise filter for cloud_slam (SlamTransientFilter).

Enhancements over two-frame odom voxel matching:
1. Directional range baseline uses all valid points from the previous frame with a 3x3
   neighborhood minimum to fill sparse sampling gaps, treating empty bins inside the
   previous sensor FOV as +inf.
2. Restricts candidate points to the scan height band z in [z_min, z_max] in the sensor frame.
3. Protects large connected object clusters (>= max_cluster_voxels in a 5cm 26-connected grid).
"""

import numpy as np
from scipy.ndimage import binary_dilation, label as nd_label
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from slam_cloud_filter import SlamVoxelFilter

_STRUCT26 = np.ones((3, 3, 3), dtype=bool)


class SlamTransientFilter:
    validate_pose = staticmethod(SlamVoxelFilter.validate_pose)

    def __init__(self, roi_radius_m=1.0, voxel_size_m=0.03, min_range_drop_m=0.10,
                 angular_resolution_deg=0.5, z_min=-0.35, z_max=0.35,
                 cluster_voxel_m=0.05, max_cluster_voxels=40,
                 cluster_z_min=-0.40, cluster_z_max=1.5, cluster_margin_m=0.5,
                 max_frame_gap_sec=0.30, max_translation_jump_m=0.5,
                 max_rotation_jump_deg=30.0, full_baseline=True, no_return_as_far=True,
                 fov_az_deg=(-52.0, 54.0), fov_el_deg=(-39.0, 41.0)):
        fov_az_deg = tuple(fov_az_deg)
        fov_el_deg = tuple(fov_el_deg)
        if len(fov_az_deg) != 2 or len(fov_el_deg) != 2:
            raise ValueError('FOV bounds must be 2-tuples')
        values = (roi_radius_m, voxel_size_m, min_range_drop_m,
                  angular_resolution_deg, z_min, z_max, cluster_voxel_m,
                  max_cluster_voxels, cluster_z_min, cluster_z_max,
                  cluster_margin_m, max_frame_gap_sec, max_translation_jump_m,
                  max_rotation_jump_deg, *fov_az_deg, *fov_el_deg)
        if not np.isfinite(values).all():
            raise ValueError('SLAM transient filter thresholds must be finite')
        if not (roi_radius_m > 0 and voxel_size_m > 0 and min_range_drop_m > 0
                and 0.25 <= angular_resolution_deg <= 5.0 and z_min < z_max
                and cluster_voxel_m > 0 and max_cluster_voxels >= 1
                and cluster_z_min < cluster_z_max and cluster_margin_m >= 0
                and max_frame_gap_sec >= 0 and max_translation_jump_m > 0
                and 0 < max_rotation_jump_deg <= 180
                and fov_az_deg[0] < fov_az_deg[1]
                and fov_el_deg[0] < fov_el_deg[1]):
            raise ValueError('invalid SlamTransientFilter parameters')
        self.roi_radius_m = float(roi_radius_m)
        self.voxel_size_m = float(voxel_size_m)
        self.min_range_drop_m = float(min_range_drop_m)
        self.angular_resolution = np.deg2rad(angular_resolution_deg)
        self.azimuth_bins = int(np.ceil(360.0 / angular_resolution_deg))
        self.elevation_bins = int(np.ceil(180.0 / angular_resolution_deg))
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.cluster_voxel_m = float(cluster_voxel_m)
        self.max_cluster_voxels = int(max_cluster_voxels)
        self.cluster_z = (float(cluster_z_min), float(cluster_z_max))
        self.cluster_margin_m = float(cluster_margin_m)
        self.max_gap_ns = int(max_frame_gap_sec * 1e9)
        self.max_translation_jump = float(max_translation_jump_m)
        self.max_rotation_jump = np.deg2rad(max_rotation_jump_deg)
        self.full_baseline = bool(full_baseline)
        self.no_return_as_far = bool(no_return_as_far)
        self.fov_az = np.deg2rad(fov_az_deg)
        self.fov_el = np.deg2rad(fov_el_deg)
        self.reset()

    def reset(self):
        self._previous_xyz = None
        self._previous_pose = None
        self._previous_stamp_ns = None

    def _reset_reason(self, matrix, stamp_ns):
        if stamp_ns is not None and self._previous_stamp_ns is not None:
            delta = int(stamp_ns) - self._previous_stamp_ns
            if delta <= 0:
                return 'non_monotonic_stamp'
            if self.max_gap_ns and delta > self.max_gap_ns:
                return 'frame_gap'
        if self._previous_pose is not None:
            translation = np.linalg.norm(matrix[:3, 3] - self._previous_pose[:3, 3])
            relative = matrix[:3, :3].T @ self._previous_pose[:3, :3]
            angle = np.arccos(np.clip((np.trace(relative) - 1) * 0.5, -1, 1))
            if translation > self.max_translation_jump or angle > self.max_rotation_jump + 1e-9:
                return 'pose_jump'
        return ''

    def _voxel_records(self, xyz):
        scaled = np.floor(xyz / self.voxel_size_m)
        if np.any(np.abs(scaled) >= float(np.iinfo(np.int64).max)):
            raise ValueError('fixed-frame voxel coordinates exceed int64')
        if np.all(np.abs(scaled) < (1 << 20)):
            k = scaled.astype(np.int64) + (1 << 20)
            return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
        keys = np.ascontiguousarray(scaled, dtype=np.int64)
        return keys.view(np.dtype('V24')).reshape(-1)

    def _direction_keys(self, relative):
        horizontal = np.hypot(relative[:, 0], relative[:, 1])
        a = np.floor((np.arctan2(relative[:, 1], relative[:, 0]) + np.pi)
                     / self.angular_resolution).astype(np.int64)
        e = np.floor((np.arctan2(relative[:, 2], horizontal) + np.pi * 0.5)
                     / self.angular_resolution).astype(np.int64)
        return (np.clip(a, 0, self.azimuth_bins - 1) * self.elevation_bins
                + np.clip(e, 0, self.elevation_bins - 1))

    def _cluster_voxels(self, relative, valid, query):
        rmax = self.roi_radius_m + self.cluster_margin_m
        ctx = (valid & (relative[:, 0] >= -0.1)
               & (relative[:, 0] * relative[:, 0] + relative[:, 1] * relative[:, 1] <= rmax * rmax)
               & (relative[:, 2] >= self.cluster_z[0]) & (relative[:, 2] <= self.cluster_z[1]))
        ctx[query] = True
        idx = np.flatnonzero(ctx)
        vk = np.floor(relative[idx] / self.cluster_voxel_m).astype(np.int64)
        vmin = vk.min(axis=0)
        shifted = vk - vmin
        shape = shifted.max(axis=0) + 1
        if int(shape[0]) * int(shape[1]) * int(shape[2]) <= 500_000:
            grid = np.zeros(shape, dtype=bool)
            grid[shifted[:, 0], shifted[:, 1], shifted[:, 2]] = True
            labels, _ = nd_label(grid, structure=_STRUCT26)
            sizes = np.bincount(labels.ravel())
            qk = np.floor(relative[query] / self.cluster_voxel_m).astype(np.int64) - vmin
            return sizes[labels[qk[:, 0], qk[:, 1], qk[:, 2]]]
        uniq, inv = np.unique(vk, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        n = len(uniq)
        pairs = cKDTree(uniq.astype(np.float64)).query_pairs(1.75, output_type='ndarray')
        graph = coo_matrix((np.ones(len(pairs), dtype=np.int8), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
        _, comp = connected_components(graph, directed=False)
        size = np.bincount(comp)
        pos = np.full(len(relative), -1, dtype=np.int64)
        pos[idx] = np.arange(len(idx))
        return size[comp[inv[pos[query]]]]

    def process(self, xyz, sensor_to_fixed, stamp_ns=None, enabled=True):
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('xyz must have shape (N, 3)')
        keep = np.ones(len(xyz), dtype=bool)
        stats = dict(input_points=len(xyz), valid_points=0, roi_points=0,
                     kept_roi_points=0, sudden_near_points=0, no_return_points=0,
                     matched_voxel_points=0, unsupported_points=0,
                     small_cluster_points=0, removed_points=0, history_points=0,
                     warmup=True, reset_reason='')
        if not enabled:
            self.reset()
            return keep, stats
        matrix = self.validate_pose(sensor_to_fixed)
        reason = self._reset_reason(matrix, stamp_ns)
        if reason:
            self.reset()
        warmup = self._previous_xyz is None or len(self._previous_xyz) == 0
        with np.errstate(invalid='ignore', over='ignore'):
            relative = (xyz - matrix[:3, 3]) @ matrix[:3, :3]
            ranges = np.linalg.norm(relative, axis=1)
            valid = (np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
                     & np.isfinite(ranges) & (ranges > 0.05))
            roi = (valid & (relative[:, 0] >= 0)
                   & (relative[:, 0] * relative[:, 0] + relative[:, 1] * relative[:, 1] <= self.roi_radius_m ** 2)
                   & (relative[:, 2] >= self.z_min) & (relative[:, 2] <= self.z_max))
        history_points = 0 if warmup else len(self._previous_xyz)
        if not warmup and roi.any():
            previous = self._previous_xyz
            pv = (previous - matrix[:3, 3]) @ matrix[:3, :3]
            pr = np.linalg.norm(pv, axis=1)
            pad = np.sqrt(3.0) * self.voxel_size_m
            exp = self.roi_radius_m + pad
            prev_near = ((pv[:, 0] >= -pad)
                         & (pv[:, 0] * pv[:, 0] + pv[:, 1] * pv[:, 1] <= exp * exp)
                         & (pr > 0.05))
            base_sel = (pr > 0.05) if self.full_baseline else prev_near
            tab = np.full((self.azimuth_bins, self.elevation_bins), np.inf)
            flat = tab.reshape(-1)
            np.minimum.at(flat, self._direction_keys(pv[base_sel]), pr[base_sel])
            ri = np.flatnonzero(roi)
            keys = self._direction_keys(relative[ri])
            baseline = flat[keys]
            if self.no_return_as_far:
                miss = np.flatnonzero(~np.isfinite(baseline))
                if len(miss):
                    a0 = keys[miss] // self.elevation_bins
                    e0 = keys[miss] % self.elevation_bins
                    for da in (-1, 0, 1):
                        for de in (-1, 0, 1):
                            if da or de:
                                a = np.clip(a0 + da, 0, self.azimuth_bins - 1)
                                e = np.clip(e0 + de, 0, self.elevation_bins - 1)
                                baseline[miss] = np.minimum(baseline[miss], tab[a, e])
                still = np.flatnonzero(~np.isfinite(baseline))
                if len(still):
                    prev_pose = self._previous_pose
                    q = (xyz[ri[still]] - prev_pose[:3, 3]) @ prev_pose[:3, :3]
                    az = np.arctan2(q[:, 1], q[:, 0])
                    el = np.arctan2(q[:, 2], np.hypot(q[:, 0], q[:, 1]))
                    in_fov = ((az >= self.fov_az[0]) & (az <= self.fov_az[1])
                              & (el >= self.fov_el[0]) & (el <= self.fov_el[1]))
                    baseline[still[in_fov]] = np.inf
                    no_return = np.zeros(len(ri), dtype=bool)
                    no_return[still[in_fov]] = True
                else:
                    no_return = np.zeros(len(ri), dtype=bool)
            else:
                no_return = np.zeros(len(ri), dtype=bool)
            sudden = no_return | (np.isfinite(baseline) & (baseline - ranges[ri] >= self.min_range_drop_m))
            matched = np.isin(self._voxel_records(xyz[ri]),
                              np.unique(self._voxel_records(previous[prev_near])))
            cand = sudden & ~matched
            small = np.zeros(len(ri), dtype=bool)
            if cand.any():
                ci = ri[cand]
                small[cand] = self._cluster_voxels(relative, valid, ci) < self.max_cluster_voxels
            drop = cand & small
            keep[ri[drop]] = False
            stats.update(sudden_near_points=int(sudden.sum()),
                         no_return_points=int(no_return.sum()),
                         matched_voxel_points=int(matched.sum()),
                         unsupported_points=int(cand.sum()),
                         small_cluster_points=int((cand & small).sum()))
        self._previous_xyz = xyz[valid].copy()
        self._previous_pose = matrix.copy()
        self._previous_stamp_ns = None if stamp_ns is None else int(stamp_ns)
        removed = int((~keep).sum())
        roi_count = int(roi.sum())
        stats.update(valid_points=int(valid.sum()),
                     roi_points=roi_count,
                     kept_roi_points=roi_count - removed,
                     removed_points=removed,
                     history_points=history_points,
                     warmup=warmup, reset_reason=reason)
        return keep, stats


def compute_dynamic_scan(xyz_odom, prev_xyz_odom, base_to_odom,
                         unsup_radius_m=0.06, dyn_frac_thresh=0.10,
                         zmin=-0.3, zmax=0.3, amin=-0.7853, amax=0.7853,
                         ainc=0.017, rmin=0.2, rmax=5.0):
    """Compute per-beam dynamic flags aligned with pc_to_scan using a 3D voxel grid."""
    nbeam = int(np.ceil((amax - amin) / ainc))
    ranges = np.full(nbeam, np.inf, dtype=np.float32)
    intensities = np.zeros(nbeam, dtype=np.float32)
    xyz = np.asarray(xyz_odom, dtype=np.float64)
    if len(xyz) == 0:
        return ranges, intensities
    matrix = SlamVoxelFilter.validate_pose(base_to_odom)
    pb = (xyz - matrix[:3, 3]) @ matrix[:3, :3]
    rh = np.hypot(pb[:, 0], pb[:, 1])
    ang = np.arctan2(pb[:, 1], pb[:, 0])
    ok = ((pb[:, 2] >= zmin) & (pb[:, 2] <= zmax)
          & (rh >= rmin) & (rh <= rmax)
          & (ang >= amin) & (ang <= amax) & np.isfinite(rh))
    idx = np.flatnonzero(ok)
    if len(idx) == 0:
        return ranges, intensities
    beam = np.clip(((ang[idx] - amin) / ainc).astype(np.int64), 0, nbeam - 1)
    order = np.lexsort((rh[idx], beam))
    b_sorted = beam[order]
    first = np.r_[True, b_sorted[1:] != b_sorted[:-1]]
    sel_beams = b_sorted[first]
    owners = idx[order][first]
    ranges[sel_beams] = rh[owners].astype(np.float32)

    if prev_xyz_odom is None or len(prev_xyz_odom) == 0:
        return ranges, intensities
    ctx = ((pb[:, 0] > -0.2) & (rh < 3.0)
           & (pb[:, 2] > -0.40) & (pb[:, 2] < 1.5) & np.isfinite(rh))
    ctx_idx = np.flatnonzero(ctx)
    if len(ctx_idx) == 0:
        return ranges, intensities

    prev_pb = (np.asarray(prev_xyz_odom, dtype=np.float64) - matrix[:3, 3]) @ matrix[:3, :3]
    prev_ctx = ((prev_pb[:, 0] > -0.3)
                & (prev_pb[:, 0] * prev_pb[:, 0] + prev_pb[:, 1] * prev_pb[:, 1] < 3.2 * 3.2)
                & (prev_pb[:, 2] > -0.50) & (prev_pb[:, 2] < 1.6))
    if not prev_ctx.any():
        return ranges, intensities

    step_u = max(0.04, float(unsup_radius_m))
    origin_u = np.array([-0.32, -3.12, -0.46])
    shape_u = np.ceil((np.array([3.12, 3.12, 1.56]) - origin_u) / step_u).astype(np.int64) + 3
    p_vk = np.clip(np.floor((prev_pb[prev_ctx] - origin_u) / step_u).astype(np.int64) + 1,
                   1, shape_u - 2)
    occ_u = np.zeros(shape_u, dtype=bool)
    occ_u[p_vk[:, 0], p_vk[:, 1], p_vk[:, 2]] = True
    occ_u = occ_u[:-2] | occ_u[1:-1] | occ_u[2:]
    occ_u = occ_u[:, :-2] | occ_u[:, 1:-1] | occ_u[:, 2:]
    occ_u = occ_u[:, :, :-2] | occ_u[:, :, 1:-1] | occ_u[:, :, 2:]
    c_vk = np.clip(np.floor((pb[ctx_idx] - origin_u) / step_u).astype(np.int64),
                   0, shape_u - 3)
    uns = ~occ_u[c_vk[:, 0], c_vk[:, 1], c_vk[:, 2]]
    if not uns.any():
        return ranges, intensities

    vk = np.floor(pb[ctx_idx] / 0.05).astype(np.int64)
    vmin = vk.min(axis=0)
    shifted = vk - vmin
    shape_c = shifted.max(axis=0) + 1
    grid = np.zeros(shape_c, dtype=bool)
    grid[shifted[:, 0], shifted[:, 1], shifted[:, 2]] = True
    labels, _ = nd_label(grid, structure=_STRUCT26)
    k = labels[shifted[:, 0], shifted[:, 1], shifted[:, 2]]
    cu = np.bincount(k, weights=uns.astype(np.float64), minlength=int(k.max()) + 1)
    cn = np.bincount(k, minlength=int(k.max()) + 1)
    frac = np.zeros(len(xyz), dtype=np.float64)
    frac[ctx_idx] = (cu / np.maximum(cn, 1))[k]
    intensities[sel_beams] = (frac[owners] >= dyn_frac_thresh).astype(np.float32)
    return ranges, intensities
