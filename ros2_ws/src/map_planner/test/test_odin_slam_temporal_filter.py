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

import importlib.util
import json
import os
from pathlib import Path
import struct
import sys
import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String


SCRIPT = Path(__file__).parents[1] / 'scripts/odin_slam_temporal_filter.py'
sys.path.insert(0, str(SCRIPT.parent))


def implementation():
    assert SCRIPT.exists(), 'SLAM ROS wrapper is not implemented'
    spec = importlib.util.spec_from_file_location('slam_filter_wrapper_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('changes', [
    {'input': '/odin1/cloud_raw'},
    {'input_topic': '/manifold/cloud/raw'},
    {'res': 0.0},
    {'res': -0.02},
    {'res': float('nan')},
    {'window': 1},
    {'persist': 1},
    {'persist': 6, 'window': 5},
    {'near_radius': 2.5, 'active_radius': 2.0},
    {'enable': 'true'},
    {'output_topic': '/odin1/cloud_slam'},
    {'unknown_parameter': 1},
])
def test_bad_navigation_configuration_rejected_before_ros_start(changes):
    with pytest.raises(ValueError):
        implementation().validate_filter_parameters(changes)


def test_navigation_yaml_uses_persist_filter_parameters():
    module = implementation()
    params = module.check_stack_config(SCRIPT.parent.parent / 'config/odin_cloud_filter.yaml')
    assert params.get('input', params.get('input_topic')) == '/odin1/cloud_slam'
    assert params.get('odom', params.get('odom_topic')) == '/odin1/odometry'
    assert params['output_topic'] == '/odin1/cloud_filtered'
    assert params['res'] == pytest.approx(0.02)
    assert params['window'] == 5
    assert params['persist'] == 2
    assert params['near_radius'] == pytest.approx(0.5)
    assert params['active_radius'] == pytest.approx(2.0)


@pytest.fixture
def context(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    value = Context()
    rclpy.init(context=value, domain_id=210 + os.getpid() % 10)
    yield value
    value.shutdown()


class Peer:
    def __init__(self, context, **options):
        self.module = implementation()
        self.filter = self.module.OdinSlamTemporalFilter(
            context=context, parameter_overrides=[
                Parameter(name, value=value) for name, value in options.items()])
        self.node = rclpy.create_node('slam_filter_test_peer', context=context)
        self.outputs, self.private_outputs = [], []
        self.single_shots, self.nears, self.traces = [], [], []
        self.node.create_subscription(
            PointCloud2, '/odin1/cloud_filtered',
            self.outputs.append, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, '/odin_cloud_temporal_filter/cloud_filtered',
            self.private_outputs.append, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, '/odin_cloud_temporal_filter/cloud_single_shot',
            self.single_shots.append, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, '/odin_cloud_temporal_filter/cloud_near',
            self.nears.append, qos_profile_sensor_data)
        self.node.create_subscription(
            String, '/odin_cloud_temporal_filter/trace',
            lambda message: self.traces.append(json.loads(message.data)), 20)
        self.pub_cloud = self.node.create_publisher(
            PointCloud2, '/odin1/cloud_slam', qos_profile_sensor_data)
        self.pub_odom = self.node.create_publisher(
            Odometry, '/odin1/odometry', qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.filter)
        self.wait(lambda: self.pub_cloud.get_subscription_count() > 0
                  and self.pub_odom.get_subscription_count() > 0
                  and self.filter.publisher.get_subscription_count() > 0
                  and self.filter.single_shot_publisher.get_subscription_count() > 0
                  and self.filter.near_publisher.get_subscription_count() > 0
                  and self.filter.trace_publisher.get_subscription_count() > 0)

    def wait(self, predicate):
        deadline = time.monotonic() + 4
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'isolated SLAM/odom delivery timed out'

    def send_odom(self, x=10.0, y=0.0, index=1):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'imu'
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(index * 10**8, 10**9)
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom)
        self.wait(lambda: self.filter._odom_xy == (float(x), float(y)))

    def send_cloud(self, points, index=1, frame='odom', custom_rgb=None):
        header = Header(frame_id=frame)
        header.stamp.sec, header.stamp.nanosec = divmod(index * 10**8, 10**9)
        if custom_rgb is not None:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='confidence', offset=16, datatype=PointField.UINT16, count=1),
            ]
            payload = b''.join(
                struct.pack('<fffIH', p[0], p[1], p[2], int(rgb_val), 65530)
                for p, rgb_val in zip(points, custom_rgb)
            )
            message = PointCloud2(
                header=header, height=1, width=len(points),
                point_step=18, row_step=len(payload), fields=fields, data=payload)
        else:
            message = pc2.create_cloud_xyz32(header, points)
        count = len(self.outputs)
        self.pub_cloud.publish(message)
        self.wait(lambda: len(self.outputs) > count
                  and len(self.single_shots) > count
                  and len(self.nears) > count
                  and len(self.traces) > count)
        assert self.outputs[-1].header == message.header
        assert self.single_shots[-1].header == message.header
        assert self.nears[-1].header == message.header
        return self.outputs[-1], self.single_shots[-1], self.nears[-1], self.traces[-1]

    def close(self):
        self.executor.shutdown()
        self.filter.destroy_node()
        self.node.destroy_node()


def unpack_records(cloud):
    assert cloud.point_step == 16
    assert [f.name for f in cloud.fields] == ['x', 'y', 'z', 'rgb']
    return np.frombuffer(
        cloud.data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])


def test_splits_near_single_shot_persistent_and_far_into_three_clouds(context):
    peer = Peer(context, res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    try:
        peer.send_odom(10.0, 0.0, index=1)
        # 3 points relative to odom=(10, 0):
        # - near: (10.2, 0, 0) -> d=0.2 < 0.5
        # - tracked: (11.0, 0, 0) -> 0.5 <= d=1.0 <= 2.0
        # - far: (12.5, 0, 0) -> d=2.5 > 2.0
        pts = [[10.2, 0.0, 0.0], [11.0, 0.0, 0.0], [12.5, 0.0, 0.0]]
        colors = [0x010203, 0x112233, 0x445566]

        # Frame 1: tracked point has count=1 < persist=2 -> goes to single_shot (red)
        # far point passes immediately to filtered; near point goes to near (yellow)
        filtered, single_shot, near, trace = peer.send_cloud(pts, index=1, custom_rgb=colors)
        rec_f = unpack_records(filtered)
        rec_s = unpack_records(single_shot)
        rec_n = unpack_records(near)

        assert len(rec_f) == 1 and pytest.approx(float(rec_f['x'][0])) == 12.5
        assert int(rec_f['rgb'][0]) == 0x445566
        assert len(rec_s) == 1 and pytest.approx(float(rec_s['x'][0])) == 11.0
        assert int(rec_s['rgb'][0]) == 0xFF0000
        assert len(rec_n) == 1 and pytest.approx(float(rec_n['x'][0])) == 10.2
        assert int(rec_n['rgb'][0]) == 0xFFFF00
        assert trace['algorithm'] == 'voxel_persistence_near_blindzone'
        assert trace['near_points'] == 1
        assert trace['single_shot_points'] == 1
        assert trace['persistent_points'] == 0
        assert trace['far_points'] == 1

        # Frame 2: tracked point at (11.0, 0, 0) seen a 2nd time -> count=2 >= persist=2 -> persistent!
        # Also add a new transient noise point at (11.5, 0.2, 0.0) -> single_shot
        pts2 = pts + [[11.5, 0.2, 0.0]]
        colors2 = colors + [0x778899]
        filtered, single_shot, near, trace = peer.send_cloud(pts2, index=2, custom_rgb=colors2)
        rec_f = unpack_records(filtered)
        rec_s = unpack_records(single_shot)
        rec_n = unpack_records(near)

        assert len(rec_f) == 2
        np.testing.assert_allclose(rec_f['x'], [11.0, 12.5])
        assert rec_f['rgb'].tolist() == [0x112233, 0x445566]
        assert len(rec_s) == 1 and pytest.approx(float(rec_s['x'][0])) == 11.5
        assert int(rec_s['rgb'][0]) == 0xFF0000
        assert len(rec_n) == 1 and pytest.approx(float(rec_n['x'][0])) == 10.2
        assert int(rec_n['rgb'][0]) == 0xFFFF00
        assert trace['persistent_points'] == 1
        assert trace['single_shot_points'] == 1
    finally:
        peer.close()


def test_missing_odom_tracks_all_points_and_warns_once(context):
    peer = Peer(context, res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    try:
        pts = [[0.1, 0.0, 0.0], [5.0, 0.0, 0.0]]
        filtered, single_shot, near, trace = peer.send_cloud(pts, index=1)
        assert not trace['has_odom']
        assert near.width == 0
        assert filtered.width == 0
        assert single_shot.width == 2
        assert peer.filter._warned_no_odom is True

        # Second frame without odom: both points reach count=2 -> filtered
        filtered, single_shot, near, trace = peer.send_cloud(pts, index=2)
        assert filtered.width == 2
        assert single_shot.width == 0
    finally:
        peer.close()


def test_non_odom_frame_warns_once_and_preserves_header(context):
    peer = Peer(context)
    try:
        peer.send_odom(0.0, 0.0, index=1)
        filtered, _, _, _ = peer.send_cloud([[3.0, 0.0, 0.0]], index=1, frame='lidar')
        assert filtered.header.frame_id == 'lidar'
        assert filtered.width == 1
        assert peer.filter._warned_frame is True
    finally:
        peer.close()


def test_disabled_filter_passes_finite_points_and_clears_history(context):
    peer = Peer(context, enable=False)
    try:
        filtered, single_shot, near, trace = peer.send_cloud([[0.2, 0.0, 0.0], [1.0, 0.0, 0.0]], index=1)
        assert filtered.width == 2
        assert single_shot.width == 0
        assert near.width == 0
        assert trace['removed_points'] == 0
    finally:
        peer.close()


def test_startup_parameters_are_read_only(context):
    peer = Peer(context)
    try:
        result = peer.filter.set_parameters([Parameter('persist', value=3)])
        assert not result[0].successful
    finally:
        peer.close()


def test_navigation_yaml_is_parsed_by_actual_ros_node(context):
    module = implementation()
    config = SCRIPT.parent.parent / 'config/odin_cloud_filter.yaml'
    node = module.OdinSlamTemporalFilter(
        context=context, cli_args=['--ros-args', '--params-file', str(config)])
    try:
        assert node.input_topic == '/odin1/cloud_slam'
        assert node.odom_topic == '/odin1/odometry'
        assert node.output_topic == '/odin1/cloud_filtered'
        assert node.core.res == pytest.approx(0.02)
        assert node.core.window == 5
        assert node.core.persist == 2
        assert node.core.near_radius == pytest.approx(0.5)
        assert node.core.active_radius == pytest.approx(2.0)
    finally:
        node.destroy_node()


def test_mock_slam_is_in_odom_coordinates_not_imu_coordinates(context):
    script = Path(__file__).resolve().parents[3] / 'scripts/mock_odin_driver.py'
    spec = importlib.util.spec_from_file_location('slam_mock_driver_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = module.MockOdinDriver(context=context, x0=10.0)
    node = rclpy.create_node('mock_slam_test_peer', context=context)
    outputs = []
    node.create_subscription(PointCloud2, '/odin1/cloud_slam',
                             outputs.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(driver)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 4
        while not outputs and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert outputs
        assert outputs[0].header.frame_id == 'odom'
        points = pc2.read_points(outputs[0], field_names=['x'], skip_nans=False)
        assert float(points['x'][0]) == pytest.approx(13.5)
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
