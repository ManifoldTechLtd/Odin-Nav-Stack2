#!/usr/bin/python3
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

"""Near-field two-frame voxel filter and optional ground-band confirmation for ordered raw point clouds.

Filters points inside the forward semicircular ROI in the sensor frame when a
point does not match any voxel from the previous frame and the same scan ray
exhibits a sudden range drop. Preserves all original PointCloud2 fields.
"""
from __future__ import annotations

import copy
import json
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
import yaml
from low_flicker_filter import LowFlickerFilter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String


FILTER_DEFAULTS = {
    'enable': True,
    'standard_enable': True,
    'input_topic': '/manifold/ODIN2/device0/cloud/raw',
    'output_topic': '/manifold/ODIN2/device0/cloud/filtered',
    'trace_topic': '/odin_cloud_temporal_filter/trace',
    'expected_input_frame': '',
    'roi_radius_m': 0.8,
    'voxel_size_m': 0.03,
    'min_range_drop_m': 0.10,
    'warmup_passthrough': True,
    'max_frame_gap_sec': 0.30,
    'log_period_sec': 2.0,
    'low_flicker_enable': False,
    'low_flicker_radius_m': 1.2,
    'low_flicker_height_min_m': 0.05,
    'low_flicker_height_max_m': 0.25,
    'low_flicker_min_hits': 2,
    'low_flicker_ground_tolerance_m': 0.03,
    'low_flicker_plane_normal': [0.0, 0.0, 0.0],
    'low_flicker_plane_offset_m': float('nan'),
    'low_flicker_plane_frame': '',
}


@dataclass
class FilterStats:
    input_points: int
    valid_points: int
    roi_points: int
    kept_roi_points: int
    removed_points: int
    sudden_near_points: int
    warmup: bool
    reset_reason: str


class TwoFrameVoxelFilter:
    """Pure numpy two-frame voxel filter operating in the sensor frame."""

    def __init__(self, roi_radius_m=0.8, voxel_size_m=0.03,
                 min_range_drop_m=0.10, warmup_passthrough=True,
                 max_frame_gap_sec=0.30):
        if not np.isfinite(roi_radius_m) or roi_radius_m <= 0.0:
            raise ValueError('roi_radius_m must be > 0')
        if not np.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
            raise ValueError('voxel_size_m must be > 0')
        if not np.isfinite(min_range_drop_m) or min_range_drop_m <= 0.0:
            raise ValueError('min_range_drop_m must be > 0')
        if not np.isfinite(max_frame_gap_sec) or max_frame_gap_sec < 0:
            raise ValueError('max_frame_gap_sec must be finite and >= 0')
        self.roi_radius_m = float(roi_radius_m)
        self.voxel_size_m = float(voxel_size_m)
        self.min_range_drop_m = float(min_range_drop_m)
        self.warmup_passthrough = bool(warmup_passthrough)
        self.max_frame_gap_ns = int(float(max_frame_gap_sec) * 1e9)
        self._previous_voxels = None
        self._previous_ranges = None
        self._previous_stamp_ns = None

    def reset(self):
        self._previous_voxels = None
        self._previous_ranges = None
        self._previous_stamp_ns = None

    @staticmethod
    def _pack_voxels(keys):
        keys = np.ascontiguousarray(keys, dtype=np.int32)
        if not len(keys):
            return np.empty(0, dtype=np.dtype((np.void, 12)))
        return keys.view(np.dtype((np.void, 12))).reshape(-1)

    def _current_reset_reason(self, stamp_ns):
        if stamp_ns is None or self._previous_stamp_ns is None:
            return ''
        delta = int(stamp_ns) - int(self._previous_stamp_ns)
        if delta <= 0:
            return 'non_monotonic_stamp'
        if self.max_frame_gap_ns > 0 and delta > self.max_frame_gap_ns:
            return 'frame_gap'
        return ''

    def process(self, xyz, stamp_ns=None, enabled=True):
        """Return per-point keep mask and FilterStats."""
        xyz = np.asarray(xyz)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('xyz must be an Nx3 array')

        point_count = len(xyz)
        finite = np.isfinite(xyz).all(axis=1)
        ranges = np.linalg.norm(xyz, axis=1)
        valid = finite & (ranges > 0.05)
        horizontal_sq = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
        roi = valid & (xyz[:, 0] >= 0.0) & (
            horizontal_sq <= self.roi_radius_m * self.roi_radius_m)

        reason = self._current_reset_reason(stamp_ns)
        if (self._previous_ranges is not None
                and len(self._previous_ranges) != point_count):
            reason = 'point_count_change'
        if reason:
            self.reset()

        keep = np.ones(point_count, dtype=bool)
        warmup = self._previous_voxels is None
        sudden_near = np.zeros(point_count, dtype=bool)
        if enabled and not warmup and np.any(roi):
            keys = np.floor(xyz[roi] / self.voxel_size_m).astype(np.int32)
            packed = self._pack_voxels(keys)
            matched = np.isin(packed, self._previous_voxels, assume_unique=False)
            previous_valid = np.isfinite(self._previous_ranges) & (
                self._previous_ranges > 0.05)
            sudden_near = roi & previous_valid & (
                self._previous_ranges - ranges >= self.min_range_drop_m)
            # Only remove points in new voxels where the same fixed scan ray suddenly shortened.
            remove_roi = sudden_near[roi] & ~matched
            keep[np.flatnonzero(roi)] = ~remove_roi
        elif enabled and warmup and not self.warmup_passthrough:
            keep[roi] = False

        expanded_radius = self.roi_radius_m + self.voxel_size_m
        previous_roi = valid & (xyz[:, 0] >= -self.voxel_size_m) & (
            horizontal_sq <= expanded_radius * expanded_radius)
        previous_keys = np.floor(
            xyz[previous_roi] / self.voxel_size_m).astype(np.int32)
        self._previous_voxels = np.unique(self._pack_voxels(previous_keys))
        self._previous_ranges = ranges.copy()
        self._previous_stamp_ns = stamp_ns

        if not enabled:
            keep[:] = True
            self.reset()

        kept_roi = int(np.count_nonzero(keep & roi))
        stats = FilterStats(
            input_points=point_count,
            valid_points=int(np.count_nonzero(valid)),
            roi_points=int(np.count_nonzero(roi)),
            kept_roi_points=kept_roi,
            removed_points=int(np.count_nonzero(roi)) - kept_roi,
            sudden_near_points=int(np.count_nonzero(sudden_near)),
            warmup=warmup,
            reset_reason=reason,
        )
        return keep, stats


def validate_filter_parameters(parameters):
    """Validate the same configuration for preflight and node construction."""
    unknown = set(parameters) - set(FILTER_DEFAULTS) - {'use_sim_time'}
    if unknown:
        raise ValueError(f'unknown filter parameters: {sorted(unknown)}')
    p = dict(FILTER_DEFAULTS, **parameters)
    for key, default in FILTER_DEFAULTS.items():
        if type(p[key]) is not type(default):
            raise ValueError(f'{key} must have ROS parameter type {type(default).__name__}')
        if isinstance(default, list) and (
                len(p[key]) != 3 or any(type(v) is not float for v in p[key])):
            raise ValueError(f'{key} must contain three doubles (write 0.0, not 0)')
    if 'use_sim_time' in p and not isinstance(p['use_sim_time'], bool):
        raise ValueError('use_sim_time must be a boolean')
    for key in ('input_topic', 'output_topic', 'trace_topic'):
        if not isinstance(p[key], str) or not p[key].startswith('/'):
            raise ValueError(f'{key} must be an absolute topic')
    if p['input_topic'].rstrip('/').endswith('/cloud_slam'):
        raise ValueError('cloud_slam is not a fixed-index sensor cloud; use cloud_raw')
    if len({p['input_topic'], p['output_topic'], p['trace_topic']}) != 3:
        raise ValueError('input/output/trace topics must differ')
    frame = p['expected_input_frame']
    if not isinstance(frame, str) or frame in ('map', 'odom', 'world'):
        raise ValueError('expected_input_frame must be a sensor frame')
    if p['low_flicker_enable'] and (
            not frame or p['low_flicker_plane_frame'] != frame):
        raise ValueError('low_flicker plane frame must match expected_input_frame')
    if not np.isfinite(p['log_period_sec']) or p['log_period_sec'] < 0:
        raise ValueError('log_period_sec must be finite and nonnegative')
    standard = TwoFrameVoxelFilter(**{
        key: p[key] for key in ('roi_radius_m', 'voxel_size_m',
                               'min_range_drop_m', 'warmup_passthrough',
                               'max_frame_gap_sec')})
    low = LowFlickerFilter(
        enabled=p['low_flicker_enable'],
        plane_normal=p['low_flicker_plane_normal'],
        plane_offset_m=p['low_flicker_plane_offset_m'],
        radius_m=p['low_flicker_radius_m'],
        height_min_m=p['low_flicker_height_min_m'],
        height_max_m=p['low_flicker_height_max_m'],
        min_hits=p['low_flicker_min_hits'],
        ground_tolerance_m=p['low_flicker_ground_tolerance_m'],
        max_frame_gap_sec=p['max_frame_gap_sec'])
    return standard, low


def check_stack_config(path):
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or set(document) != {'odin_cloud_temporal_filter'}:
        raise ValueError('config must contain only odin_cloud_temporal_filter')
    node_config = document['odin_cloud_temporal_filter']
    if not isinstance(node_config, dict) or set(node_config) != {'ros__parameters'}:
        raise ValueError('config must contain ros__parameters')
    params = node_config['ros__parameters']
    if not isinstance(params, dict):
        raise ValueError('ros__parameters must be a mapping')
    validate_filter_parameters(params)
    for key, expected in (
            ('input_topic', '/odin1/cloud_raw'),
            ('output_topic', '/odin1/cloud_filtered'),
            ('trace_topic', '/odin_cloud_temporal_filter/trace'),
            ('expected_input_frame', 'lidar')):
        if params.get(key) != expected:
            raise ValueError(f'SW01 stack requires {key}: {expected}')
    return params


def pointcloud_xyz(message):
    """Extract xyz columns from PointCloud2 by field name regardless of point_step."""
    field_names = {field.name for field in message.fields}
    if not {'x', 'y', 'z'}.issubset(field_names):
        raise ValueError('PointCloud2 missing x/y/z fields')
    width, height = int(message.width), int(message.height)
    if (message.row_step < width * message.point_step
            or len(message.data) < height * message.row_step):
        raise ValueError('invalid PointCloud2 row_step/data length')
    dtype = pc2.dtype_from_fields(message.fields, message.point_step).newbyteorder(
        '>' if message.is_bigendian else '<')
    points = np.ndarray(
        shape=(height, width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step))
    points = points.reshape(-1)
    return np.column_stack((points['x'], points['y'], points['z']))


def select_pointcloud_records(message, keep):
    """Compact kept PointCloud2 records while preserving all raw field bytes."""
    keep = np.asarray(keep, dtype=bool)
    expected_points = int(message.width) * int(message.height)
    if len(keep) != expected_points:
        raise ValueError(
            f'keep length {len(keep)} does not match point count {expected_points}')
    if np.all(keep):
        return copy.copy(message)

    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected_bytes = int(message.row_step) * int(message.height)
    if raw.size < expected_bytes:
        raise ValueError('PointCloud2 data shorter than height*row_step')
    rows = raw[:expected_bytes].reshape(int(message.height), int(message.row_step))
    payload_width = int(message.width) * int(message.point_step)
    payload = np.ascontiguousarray(rows[:, :payload_width])
    records = payload.reshape(expected_points, int(message.point_step))
    selected = np.ascontiguousarray(records[keep])

    output = copy.copy(message)
    output.height = 1
    output.width = int(np.count_nonzero(keep))
    output.row_step = output.width * int(message.point_step)
    output.data = selected.reshape(-1).tobytes()
    return output


class OdinCloudTemporalFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('odin_cloud_temporal_filter', **kwargs)
        for name, default in FILTER_DEFAULTS.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        p = {name: self.get_parameter(name).value for name in FILTER_DEFAULTS}
        self.core, self.low_core = validate_filter_parameters(p)
        self.enabled = p['enable']
        self.standard_enabled = p['standard_enable']
        self.input_topic = p['input_topic']
        self.output_topic = p['output_topic']
        self.expected_frame = p['expected_input_frame']
        self.log_period_sec = p['log_period_sec']
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.trace_publisher = self.create_publisher(
            String, p['trace_topic'], 20)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._callback,
            qos_profile_sensor_data)
        self._last_log_time = 0.0
        self._frames = 0
        self._total_removed = 0
        self._layout = None

        self.get_logger().info(
            '[odin_cloud_temporal_filter] enable=%s standard=%s low_flicker=%s '
            'input=%s output=%s expected_frame=%s '
            'ROI=前方半圆 R=%.2fm voxel=%.3fm range_drop>=%.2fm'
            % (self.enabled, self.standard_enabled, self.low_core.enabled,
               self.input_topic, self.output_topic, self.expected_frame,
               self.core.roi_radius_m, self.core.voxel_size_m,
               self.core.min_range_drop_m))

    def _reset(self):
        self.core.reset()
        self.low_core.reset()
        self._layout = None

    def _callback(self, message):
        started = time.perf_counter()
        low_stats = dict(candidate_points=0, removed_points=0, confirmed_points=0)
        removed = 0
        reason = ''
        try:
            layout = (message.header.frame_id, message.width, message.height,
                      message.point_step, message.row_step, message.is_bigendian,
                      tuple((f.name, f.offset, f.datatype, f.count) for f in message.fields))
            if self._layout is not None and layout != self._layout:
                reason = ('frame_id_change' if layout[0] != self._layout[0]
                          else 'cloud_layout_change')
                self._reset()
            if (message.header.frame_id in ('map', 'odom', 'world')
                    or (self.expected_frame and message.header.frame_id != self.expected_frame)):
                reason = 'unexpected_frame'
                raise ValueError(f'unexpected sensor frame: {message.header.frame_id}')
            xyz = pointcloud_xyz(message)
            stamp = message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            keep, stats = self.core.process(
                xyz, stamp_ns=stamp_ns,
                enabled=self.enabled and self.standard_enabled)
            if self.enabled:
                low_keep, low_stats = self.low_core.process(xyz, stamp_ns=stamp_ns)
                keep &= low_keep
            else:
                self.low_core.reset()
            removed = int(np.count_nonzero(~keep))
            if removed:
                removed_xyz = xyz[~keep]
                removed_in_roi = ((removed_xyz[:, 0] >= 0)
                                  & (removed_xyz[:, 0] ** 2 + removed_xyz[:, 1] ** 2
                                     <= self.core.roi_radius_m ** 2))
                stats.kept_roi_points = stats.roi_points - int(np.count_nonzero(removed_in_roi))
            output = select_pointcloud_records(message, keep)
            self._layout = layout
            if reason:
                stats.reset_reason = reason
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_log_time >= self.log_period_sec:
                self.get_logger().error(f'点云滤波失败，本帧原样转发: {exc}')
                self._last_log_time = now
            self._reset()
            output = message
            removed = 0
            low_stats = dict(candidate_points=0, removed_points=0, confirmed_points=0)
            stats = FilterStats(
                input_points=int(message.width) * int(message.height),
                valid_points=0, roi_points=0, kept_roi_points=0,
                removed_points=0, sudden_near_points=0,
                warmup=True, reset_reason=(
                    reason if reason == 'unexpected_frame' else 'exception'))

        self.publisher.publish(output)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._frames += 1
        self._total_removed += removed
        trace = {
            'stamp': float(message.header.stamp.sec)
                     + float(message.header.stamp.nanosec) * 1e-9,
            'frame': self._frames,
            'input_points': stats.input_points,
            'valid_points': stats.valid_points,
            'roi_points': stats.roi_points,
            'kept_roi_points': stats.kept_roi_points,
            'removed_points': removed,
            'standard_removed_points': stats.removed_points,
            'low_flicker_candidate_points': low_stats['candidate_points'],
            'low_flicker_removed_points': low_stats['removed_points'],
            'low_flicker_confirmed_points': low_stats['confirmed_points'],
            'enable': self.enabled,
            'standard_enable': self.standard_enabled,
            'low_flicker_enable': self.low_core.enabled,
            'sudden_near_points': stats.sudden_near_points,
            'total_removed_points': self._total_removed,
            'warmup': (stats.warmup if self.standard_enabled
                       else low_stats.get('warmup', True)),
            'reset_reason': stats.reset_reason or low_stats.get('reset_reason', ''),
            'elapsed_ms': round(elapsed_ms, 3),
        }
        self.trace_publisher.publish(String(
            data=json.dumps(trace, ensure_ascii=False, separators=(',', ':'))))

        now = time.monotonic()
        if now - self._last_log_time >= self.log_period_sec:
            self._last_log_time = now
            self.get_logger().info(
                '[odin_cloud_temporal_filter] frame=%d roi=%d removed=%d '
                'output=%d elapsed=%.2fms'
                % (self._frames, stats.roi_points, removed,
                   int(output.width) * int(output.height), elapsed_ms))


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if argv and argv[0] == '--check-config':
        if len(argv) != 2:
            raise SystemExit('usage: odin_cloud_temporal_filter.py --check-config CONFIG.yaml')
        try:
            params = check_stack_config(argv[1])
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            raise SystemExit(f'invalid cloud filter configuration: {exc}') from exc
        print('cloud filter configuration OK: standard=%s low_flicker=%s'
              % (params.get('standard_enable', True), params.get('low_flicker_enable', False)))
        return
    rclpy.init(args=args)
    node = OdinCloudTemporalFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
