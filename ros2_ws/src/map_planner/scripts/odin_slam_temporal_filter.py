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

"""Voxel-persistence and near-field blind-spot point cloud filter node (ROS 2).

- Subscribes to `input` (/odin1/cloud_slam) and `odom` (/odin1/odometry) using SensorDataQoS.
- Publishes:
  * `output_topic` (/odin1/cloud_filtered) and `~/cloud_filtered`: persistent + far-field points
  * `single_shot_topic` (~/cloud_single_shot): single-frame transient points inside the active radius
  * `near_topic` (~/cloud_near): near-field blind-spot points (d^2 < near_radius^2)
  * `trace_topic` (/odin_cloud_temporal_filter/trace): per-frame JSON statistics
"""

import json
import os
import sys
import time

for _env_key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_env_key, '1')

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from slam_cloud_filter import (
    COLOR_NEAR_RGB_F32,
    COLOR_SINGLE_SHOT_RGB_F32,
    PersistVoxelFilter,
    pack_cloud,
    parse_pointcloud_xyz_rgb,
    unpack_cloud,
    voxel_keys,
)
from std_msgs.msg import String


FILTER_DEFAULTS = {
    'enable': True,
    'input': '/odin1/cloud_slam',
    'odom': '/odin1/odometry',
    'input_topic': '',
    'odom_topic': '',
    'output_topic': '/odin1/cloud_filtered',
    'single_shot_topic': '~/cloud_single_shot',
    'near_topic': '~/cloud_near',
    'trace_topic': '/odin_cloud_temporal_filter/trace',
    'expected_input_frame': 'odom',
    'res': 0.02,
    'window': 5,
    'persist': 2,
    'near_radius': 0.5,
    'active_radius': 2.0,
    'log_period_sec': 2.0,
}

CORE_KEYS = ('res', 'window', 'persist', 'near_radius', 'active_radius')


def _resolve_topics(p):
    input_topic = p['input_topic'] if p.get('input_topic') else p['input']
    odom_topic = p['odom_topic'] if p.get('odom_topic') else p['odom']
    return input_topic, odom_topic


def validate_filter_parameters(parameters):
    unknown = set(parameters) - set(FILTER_DEFAULTS) - {'use_sim_time'}
    if unknown:
        raise ValueError(f'unknown SLAM filter parameters: {sorted(unknown)}')
    p = dict(FILTER_DEFAULTS, **parameters)
    for key, default in FILTER_DEFAULTS.items():
        if isinstance(default, float) and isinstance(p[key], int) and not isinstance(p[key], bool):
            p[key] = float(p[key])
        if type(p[key]) is not type(default):
            raise ValueError(f'{key} requires {type(default).__name__}')
    if 'use_sim_time' in p and not isinstance(p['use_sim_time'], bool):
        raise ValueError('use_sim_time requires bool')

    input_topic, odom_topic = _resolve_topics(p)
    for name, topic in (
        ('input', input_topic),
        ('odom', odom_topic),
        ('output_topic', p['output_topic']),
        ('single_shot_topic', p['single_shot_topic']),
        ('near_topic', p['near_topic']),
        ('trace_topic', p['trace_topic']),
    ):
        if not topic or not (topic.startswith('/') or topic.startswith('~/')) or any(c.isspace() for c in topic):
            raise ValueError(f'{name} requires a valid ROS topic (/... or ~/...)')
    if input_topic.rstrip('/').endswith('/cloud_raw') or input_topic.rstrip('/').endswith('/raw'):
        raise ValueError('SLAM persistence filter requires cloud_slam, not ordered raw input')
    if len({input_topic, p['output_topic'], p['single_shot_topic'], p['near_topic'], p['trace_topic']}) != 5:
        raise ValueError('input, output, single_shot, near and trace topics must differ')
    if not np.isfinite(p['log_period_sec']) or p['log_period_sec'] < 0.0:
        raise ValueError('log_period_sec must be finite and nonnegative')
    return PersistVoxelFilter(**{key: p[key] for key in CORE_KEYS})


def check_stack_config(path):
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not document:
        raise ValueError('config must be a non-empty mapping')
    allowed_top = {'odin_cloud_temporal_filter', 'persist_filter', '/**'}
    if not set(document).issubset(allowed_top):
        raise ValueError(f'config top-level keys must be in {sorted(allowed_top)}')
    first_key = next(iter(document))
    settings = document[first_key]
    if not isinstance(settings, dict) or set(settings) != {'ros__parameters'}:
        raise ValueError('config must contain ros__parameters')
    params = settings['ros__parameters']
    if not isinstance(params, dict):
        raise ValueError('ros__parameters must be a mapping')
    validate_filter_parameters(params)
    merged = dict(FILTER_DEFAULTS, **params)
    input_topic, odom_topic = _resolve_topics(merged)
    if input_topic != '/odin1/cloud_slam':
        raise ValueError('SW01 SLAM stack requires input: /odin1/cloud_slam')
    if odom_topic != '/odin1/odometry':
        raise ValueError('SW01 SLAM stack requires odom: /odin1/odometry')
    if merged['output_topic'] != '/odin1/cloud_filtered':
        raise ValueError('SW01 SLAM stack requires output_topic: /odin1/cloud_filtered')
    if merged['trace_topic'] != '/odin_cloud_temporal_filter/trace':
        raise ValueError('SW01 SLAM stack requires trace_topic: /odin_cloud_temporal_filter/trace')
    return merged


def passthrough_stats(count, has_odom=False, reason=''):
    return dict(
        input_points=count, valid_points=0, near_points=0, tracked_points=0,
        far_points=0, persistent_points=0, single_shot_points=0,
        filtered_points=0, removed_points=0, roi_points=0, kept_roi_points=0,
        window_frames=0, unique_window_voxels=0, has_odom=has_odom,
        warmup=True, reset_reason=reason,
    )


class OdinSlamTemporalFilter(Node):
    def __init__(self, node_name='odin_cloud_temporal_filter', **kwargs):
        super().__init__(node_name, **kwargs)
        for name, default in FILTER_DEFAULTS.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        p = {name: self.get_parameter(name).value for name in FILTER_DEFAULTS}
        self.core = validate_filter_parameters(p)
        self.enabled = p['enable']
        self.input_topic, self.odom_topic = _resolve_topics(p)
        self.input = self.input_topic
        self.res = self.core.res
        self.window = self.core.window
        self.persist = self.core.persist
        self.near_radius = self.core.near_radius
        self.active_radius = self.core.active_radius
        self.output_topic = p['output_topic']
        self.single_shot_topic = p['single_shot_topic']
        self.near_topic = p['near_topic']
        self.trace_topic = p['trace_topic']
        self.expected_frame = p['expected_input_frame']
        self.log_period = p['log_period_sec']

        self._odom_xy = None
        self.pose_xy = None
        self._warned_no_odom = False
        self._warned_frame = False
        self._warned_odom_frame = False
        self._frames = 0
        self._total_removed = 0
        self._last_warning = 0.0
        self.dt_sum = 0.0
        self.dt_max = 0.0

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        private_filtered = '~/cloud_filtered'
        if self.resolve_topic_name(private_filtered) != self.resolve_topic_name(self.output_topic):
            self.private_filtered_publisher = self.create_publisher(
                PointCloud2, private_filtered, qos_profile_sensor_data)
        else:
            self.private_filtered_publisher = None
        self.pub_filtered = self.private_filtered_publisher or self.publisher
        self.single_shot_publisher = self.create_publisher(
            PointCloud2, self.single_shot_topic, qos_profile_sensor_data)
        self.pub_single = self.single_shot_publisher
        self.near_publisher = self.create_publisher(
            PointCloud2, self.near_topic, qos_profile_sensor_data)
        self.pub_near = self.near_publisher
        self.trace_publisher = self.create_publisher(String, self.trace_topic, 20)

        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic} & ~/cloud_filtered '
            f'(res {self.res:.3f} m, window {self.window}, persist {self.persist}, '
            f'near {self.near_radius:.2f} m, active {self.active_radius:.2f} m)')

    def on_odom(self, message):
        pos = message.pose.pose.position
        ox, oy = float(pos.x), float(pos.y)
        if not (np.isfinite(ox) and np.isfinite(oy)):
            return
        self._odom_xy = (ox, oy)
        self.pose_xy = np.array([ox, oy], dtype=np.float64)
        if (not self._warned_odom_frame and message.header.frame_id
                and self.expected_frame and message.header.frame_id != self.expected_frame):
            self._warned_odom_frame = True
            self.get_logger().warning(f"odom frame is '{message.header.frame_id}'")

    _odom_callback = on_odom

    def on_cloud(self, message):
        t0 = time.perf_counter()
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
        if (self.expected_frame and message.header.frame_id != self.expected_frame
                and not self._warned_frame):
            self._warned_frame = True
            self.get_logger().warning(
                f"cloud frame is '{message.header.frame_id}', expected '{self.expected_frame}'")
        if self._odom_xy is None and not self._warned_no_odom:
            self._warned_no_odom = True
            self.get_logger().warning('no odometry yet, near/active radius disabled')

        try:
            xyz, rgb = parse_pointcloud_xyz_rgb(message)
            filtered_mask, single_shot_mask, near_mask, stats = self.core.classify(
                xyz, odom_xy=self._odom_xy, enabled=self.enabled)
            filtered_msg = pack_cloud(
                message.header, xyz[filtered_mask], rgb[filtered_mask])
            single_shot_msg = pack_cloud(
                message.header, xyz[single_shot_mask],
                np.full(int(single_shot_mask.sum()), 0x00FF0000, dtype=np.uint32))
            near_msg = pack_cloud(
                message.header, xyz[near_mask],
                np.full(int(near_mask.sum()), 0x00FFFF00, dtype=np.uint32))
        except Exception as exc:
            self.core.reset()
            stats = passthrough_stats(
                int(message.width) * int(message.height),
                has_odom=self._odom_xy is not None,
                reason='exception')
            filtered_msg = message
            empty_xyz = np.empty((0, 3), dtype=np.float32)
            empty_u32 = np.empty(0, dtype=np.uint32)
            single_shot_msg = pack_cloud(message.header, empty_xyz, empty_u32)
            near_msg = pack_cloud(message.header, empty_xyz, empty_u32)
            now = time.monotonic()
            if now - self._last_warning >= self.log_period:
                self.get_logger().warning(f'Persist filter exception, passthrough: {exc}')
                self._last_warning = now

        self.publisher.publish(filtered_msg)
        if self.private_filtered_publisher is not None:
            self.private_filtered_publisher.publish(filtered_msg)
        self.single_shot_publisher.publish(single_shot_msg)
        self.near_publisher.publish(near_msg)

        self._frames += 1
        self._total_removed += stats['removed_points']
        dt = (time.perf_counter() - t0) * 1e3
        self.dt_sum += dt
        self.dt_max = max(self.dt_max, dt)
        if self._frames % 20 == 0:
            self.get_logger().info(
                f"frames={self._frames} pts={stats['valid_points']} "
                f"kept={stats['filtered_points']} single={stats['single_shot_points']} "
                f"near={stats['near_points']} ms avg={self.dt_sum / 20.0:.1f} max={self.dt_max:.1f}")
            self.dt_sum = 0.0
            self.dt_max = 0.0

        trace = dict(
            stats,
            stamp=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            stamp_ns=stamp_ns,
            frame=self._frames,
            algorithm='voxel_persistence_near_blindzone',
            fixed_frame=message.header.frame_id,
            enable=self.enabled,
            res=self.core.res,
            window=self.core.window,
            persist=self.core.persist,
            near_radius=self.core.near_radius,
            active_radius=self.core.active_radius,
            total_removed_points=self._total_removed,
            elapsed_ms=round(dt, 3),
        )
        self.trace_publisher.publish(String(data=json.dumps(trace, separators=(',', ':'))))

    _callback = on_cloud


PersistFilter = OdinSlamTemporalFilter
PersistFilterNode = OdinSlamTemporalFilter


def main(args=None, default_node_name='odin_cloud_temporal_filter'):
    argv = sys.argv[1:] if args is None else args
    if argv and argv[0] == '--check-config':
        if len(argv) != 2:
            raise SystemExit('usage: odin_slam_temporal_filter.py --check-config CONFIG.yaml')
        try:
            params = check_stack_config(argv[1])
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            raise SystemExit(f'invalid SLAM filter configuration: {exc}') from exc
        print('SLAM filter configuration OK: algorithm=voxel_persistence_near_blindzone '
              f'res={params["res"]} window={params["window"]} persist={params["persist"]} '
              f'near={params["near_radius"]} active={params["active_radius"]}')
        return
    if hasattr(os, 'sched_getaffinity') and hasattr(os, 'sched_setaffinity'):
        try:
            cpus = os.sched_getaffinity(0)
            non_neupan = {c for c in cpus if c >= 2}
            if non_neupan and len(cpus) > 2:
                os.sched_setaffinity(0, non_neupan)
        except OSError:
            pass
    rclpy.init(args=args)
    node = OdinSlamTemporalFilter(node_name=default_node_name)
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
