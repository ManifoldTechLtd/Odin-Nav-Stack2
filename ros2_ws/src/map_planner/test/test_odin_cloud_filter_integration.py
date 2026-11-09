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

import json
import importlib.util
import os
from pathlib import Path
import struct
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String

from test_odin_cloud_temporal_filter import MODULE


def configuration(**overrides):
    return dict(input_topic='/odin1/cloud_raw', output_topic='/odin1/cloud_filtered',
                expected_input_frame='lidar', low_flicker_enable=True,
                low_flicker_plane_normal=[0.0, 0.0, 1.0],
                low_flicker_plane_offset_m=0.6,
                low_flicker_plane_frame='lidar', **overrides)


@pytest.mark.parametrize('changes', [
    {'low_flicker_plane_normal': [0.0, 0.0, 0.0]},
    {'low_flicker_plane_offset_m': float('nan')},
    {'low_flicker_plane_frame': ''},
    {'low_flicker_plane_frame': 'other_lidar'},
    {'expected_input_frame': 'odom'},
    {'input_topic': '/odin1/cloud_slam'},
    {'output_topic': '/odin1/cloud_raw'},
    {'roi_radius_m': float('nan')},
    {'max_frame_gap_sec': -0.1},
    {'low_flicker_enable': 'false'},
    {'low_flicker_min_hits': 2.0},
    {'roi_radius_m': 1},
    {'low_flicker_plane_normal': [0, 0, 1]},
    {'low_flicker_plane_frame': None},
    {'unknown_typo': 2},
])
def test_bad_configuration_rejected_before_ros_start(changes):
    assert hasattr(MODULE, 'validate_filter_parameters'), 'configuration preflight missing'
    params = configuration()
    params.update(changes)
    with pytest.raises(ValueError):
        MODULE.validate_filter_parameters(params)


def test_standard_only_defaults_do_not_require_ground_calibration():
    assert hasattr(MODULE, 'validate_filter_parameters'), 'configuration preflight missing'
    standard, low = MODULE.validate_filter_parameters({})
    assert standard.roi_radius_m == 0.8
    assert not low.enabled


def test_xyz_reader_supports_row_padding_and_big_endian_without_mutating_input():
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z'))]
    payload = struct.pack('>fff', 0.5, 0.1, -0.6) + b'pad!'
    payload += struct.pack('>fff', 1.0, 0.2, -0.5) + b'end!'
    cloud = PointCloud2(height=2, width=1, fields=fields, point_step=12,
                        row_step=16, is_bigendian=True, data=payload)
    xyz = MODULE.pointcloud_xyz(cloud)
    np.testing.assert_allclose(xyz, [[0.5, 0.1, -0.6], [1, 0.2, -0.5]])
    assert bytes(cloud.data) == payload
    selected = MODULE.select_pointcloud_records(cloud, [False, True])
    assert bytes(selected.data) == payload[16:28]
    assert selected.is_bigendian


@pytest.fixture
def ros_context(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    context = Context()
    rclpy.init(context=context, domain_id=170 + os.getpid() % 20)
    yield context
    context.shutdown()


class FilterPeer:
    def __init__(self, context, **options):
        params = configuration()
        params.update(options)
        self.filter = MODULE.OdinCloudTemporalFilter(
            context=context, parameter_overrides=[
                Parameter(name, value=value) for name, value in params.items()])
        self.node = rclpy.create_node('cloud_filter_test_peer', context=context)
        self.outputs, self.traces = [], []
        self.node.create_subscription(PointCloud2, '/odin1/cloud_filtered',
                                      self.outputs.append, qos_profile_sensor_data)
        self.node.create_subscription(
            String, '/odin_cloud_temporal_filter/trace',
            lambda msg: self.traces.append(json.loads(msg.data)), 20)
        self.publisher = self.node.create_publisher(
            PointCloud2, '/odin1/cloud_raw', qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.filter)
        self.executor.add_node(self.node)
        self.wait_for(lambda: self.publisher.get_subscription_count() > 0
                      and self.filter.publisher.get_subscription_count() > 0
                      and self.filter.trace_publisher.get_subscription_count() > 0)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 4
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'local ROS cloud/trace delivery timed out'

    def send(self, points, index, frame='lidar'):
        header = Header(frame_id=frame)
        header.stamp.sec, header.stamp.nanosec = divmod(index * 100_000_000, 10**9)
        message = pc2.create_cloud_xyz32(header, points)
        count = len(self.outputs)
        self.publisher.publish(message)
        self.wait_for(lambda: len(self.outputs) > count and len(self.traces) > count)
        assert self.outputs[-1].header == header
        return self.outputs[-1], self.traces[-1]

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.filter.destroy_node()


def test_both_masks_use_original_indices_and_do_not_add_confirmation_delays(ros_context):
    peer = FilterPeer(ros_context, low_flicker_min_hits=3)
    try:
        ground = [[0.6, 0, -0.6], [0.4, 0, -0.6], [1.8, 0, -0.6]]
        low = [[0.4, 0, -0.5], [0.2, 0, -0.5], [1.8, 0, -0.6]]
        peer.send(ground, 1)
        output, trace = peer.send(low, 2)
        assert output.width == 1
        assert trace['standard_removed_points'] == 2
        assert trace['low_flicker_removed_points'] == 2
        assert trace['removed_points'] == 2
        assert peer.send(low, 3)[0].width == 1
        assert peer.send(low, 4)[0].width == 3
        assert peer.send(ground, 5)[0].width == 3
    finally:
        peer.close()


@pytest.mark.parametrize('options', [
    {'standard_enable': False}, {'enable': False},
])
def test_algorithm_switches_are_independent(ros_context, options):
    peer = FilterPeer(ros_context, **options)
    try:
        peer.send([[0.6, 0, -0.6]], 1)
        output, trace = peer.send([[0.4, 0, -0.5]], 2)
        assert trace['standard_removed_points'] == 0
        assert trace['kept_roi_points'] == output.width
        if options.get('enable') is False:
            assert output.width == 1
            assert trace['removed_points'] == 0
        else:
            assert output.width == 0
            assert trace['low_flicker_removed_points'] == 1
            assert not trace['warmup']
    finally:
        peer.close()


def test_wrong_frame_is_passthrough_and_clears_both_histories(ros_context):
    peer = FilterPeer(ros_context)
    try:
        peer.send([[0.6, 0, -0.6]], 1)
        output, trace = peer.send([[0.4, 0, -0.5]], 2, frame='odom')
        assert output.width == 1
        assert trace['reset_reason'] == 'unexpected_frame'
        assert peer.send([[0.2, 0, -0.5]], 3)[0].width == 1
    finally:
        peer.close()


def test_malformed_cloud_passthrough_and_layout_change_reset(ros_context):
    peer = FilterPeer(ros_context)
    try:
        peer.send([[0.6, 0, -0.6], [0.6, 0.1, -0.6]], 1)
        message = pc2.create_cloud_xyz32(
            Header(frame_id='lidar'), [[0.4, 0, -0.5], [0.4, 0.1, -0.5]])
        message.header.stamp.nanosec = 200_000_000
        message.height, message.width, message.row_step = 2, 1, 12
        peer.publisher.publish(message)
        peer.wait_for(lambda: len(peer.outputs) == 2 and len(peer.traces) == 2)
        assert peer.outputs[-1].width * peer.outputs[-1].height == 2
        assert peer.traces[-1]['reset_reason'] == 'cloud_layout_change'
        message.fields = []
        peer.publisher.publish(message)
        peer.wait_for(lambda: len(peer.outputs) == 3 and len(peer.traces) == 3)
        assert bytes(peer.outputs[-1].data) == bytes(message.data)
        assert peer.traces[-1]['reset_reason'] == 'exception'
    finally:
        peer.close()


def test_startup_parameters_cannot_silently_diverge_from_active_cores(ros_context):
    peer = FilterPeer(ros_context)
    try:
        result = peer.filter.set_parameters([Parameter('standard_enable', value=False)])
        assert not result[0].successful
    finally:
        peer.close()


def test_mock_driver_provides_stable_raw_sensor_cloud(ros_context):
    script = Path(__file__).resolve().parents[3] / 'scripts/mock_odin_driver.py'
    spec = importlib.util.spec_from_file_location('mock_odin_driver', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = module.MockOdinDriver(context=ros_context)
    peer = rclpy.create_node('mock_raw_test_peer', context=ros_context)
    outputs = []
    peer.create_subscription(PointCloud2, '/odin1/cloud_raw',
                             outputs.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(driver)
    executor.add_node(peer)
    try:
        deadline = time.monotonic() + 4
        while len(outputs) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(outputs) >= 2
        assert outputs[0].header.frame_id == 'lidar'
        assert bytes(outputs[0].data) == bytes(outputs[1].data)
        assert outputs[0].header.stamp != outputs[1].header.stamp
    finally:
        executor.shutdown()
        peer.destroy_node()
        driver.destroy_node()


def test_legacy_raw_validator_rejects_the_slam_navigation_yaml():
    config = Path(__file__).parents[1] / 'config/odin_cloud_filter.yaml'
    with pytest.raises(ValueError):
        MODULE.check_stack_config(config)
