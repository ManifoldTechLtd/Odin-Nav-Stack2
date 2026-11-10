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

"""Per-frame ray update contract of the compiled sliding_map_node (clearing, no-return, angles)."""
import math
import os
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

BASE_PARAMS = {
    'map_frame': 'odom',
    'base_frame': 'odin1_base_link',
    'footprint_frame': 'base_link',
    'input_scan_topic': '/scan',
    'fake_scan_topic': '/scan_360',
    'map_topic': '/sliding_map',
    'range_min': 0.05,
    'max_fake_range': 4.5,
    'output_points': 360,
    'cell_decay_sec': 15.0,
    'decay_check_period_sec': 0.05,
    'use_footprint_polygon': True,
    'trace_period_sec': 0.5,
    'dynamic_memory_enable': False,
}


def _param_text(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _stamp(ts):
    return rclpy.time.Time(seconds=int(ts), nanoseconds=int((ts % 1.0) * 1e9)).to_msg()


class SlidingMapRig:
    """Runs sliding_map_node and feeds it synthetic scans and odom->odin1_base_link TF."""

    def __init__(self, monkeypatch, **params):
        monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
        self.domain = str(210 + os.getpid() % 15)
        monkeypatch.setenv('ROS_DOMAIN_ID', self.domain)
        binary = os.path.join(get_package_prefix('sliding_map'), 'lib', 'sliding_map',
                              'sliding_map_node')
        assert os.path.isfile(binary), f'sliding_map_node binary not found: {binary}'

        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=int(self.domain))
        self.node = rclpy.create_node('sliding_map_contract_' + uuid.uuid4().hex[:6], context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.maps = []
        self.fake_scans = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/sliding_map', self.maps.append, latched)
        self.node.create_subscription(LaserScan, '/scan_360', self.fake_scans.append, 5)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 5)
        self.dyn_pub = self.node.create_publisher(LaserScan, '/scan_dynamic', 5)
        self.static_tf = StaticTransformBroadcaster(self.node)
        self.tf = TransformBroadcaster(self.node)
        self._static = TransformStamped()
        self._static.header.frame_id = 'odin1_base_link'
        self._static.child_frame_id = 'base_link'
        self._static.transform.translation.x = -0.42
        self._static.transform.translation.z = 0.12
        self._static.transform.rotation.w = 1.0
        self.static_tf.sendTransform(self._static)

        merged = dict(BASE_PARAMS, **params)
        args = [binary, '--ros-args']
        for key, value in merged.items():
            args += ['-p', f'{key}:={_param_text(value)}']
        self.proc = subprocess.Popen(
            args, env=dict(os.environ, ROS_DOMAIN_ID=self.domain,
                           ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        deadline = time.monotonic() + 5.0
        while (self.scan_pub.get_subscription_count() == 0
               or self.node.count_subscribers('/tf') == 0
               or self.node.count_subscribers('/tf_static') == 0) and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        assert self.scan_pub.get_subscription_count() > 0
        self.static_tf.sendTransform(self._static)
        time.sleep(0.1)
        self.base_sec = time.time()

    def _send_tf(self, ts):
        for dt in (-0.05, 0.0, 0.05):
            msg = TransformStamped()
            msg.header.stamp = _stamp(ts + dt)
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'odin1_base_link'
            msg.transform.rotation.w = 1.0
            self.tf.sendTransform(msg)

    def send_frame(self, sec_offset, ranges, angle_min=0.0, angle_increment=0.25, dynamic=None):
        """Publish one scan (repeated until processed) and return (map, fake_scan)."""
        ts = self.base_sec + sec_offset
        self._send_tf(ts)
        time.sleep(0.03)
        scan = LaserScan()
        scan.header.stamp = _stamp(ts)
        scan.header.frame_id = 'odin1_base_link'
        scan.angle_min = float(angle_min)
        scan.angle_increment = float(angle_increment)
        scan.angle_max = float(angle_min + (len(ranges) - 1) * angle_increment)
        scan.range_min = 0.2
        scan.range_max = 5.0
        scan.ranges = [float(r) for r in ranges]
        dyn = None
        if dynamic is not None:
            dyn = LaserScan()
            dyn.header = scan.header
            dyn.ranges = scan.ranges
            dyn.intensities = [float(v) for v in dynamic]
        map_count, scan_count = len(self.maps), len(self.fake_scans)
        deadline = time.monotonic() + 3.0
        while ((len(self.maps) <= map_count or len(self.fake_scans) <= scan_count)
               and time.monotonic() < deadline):
            self._send_tf(ts)
            if dyn is not None:
                self.dyn_pub.publish(dyn)
                time.sleep(0.02)
            self.scan_pub.publish(scan)
            for _ in range(3):
                self.executor.spin_once(timeout_sec=0.02)
        assert len(self.maps) > map_count and len(self.fake_scans) > scan_count
        return self.maps[-1], self.fake_scans[-1]

    def close(self):
        self.proc.terminate()
        try:
            self.proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.communicate()
        self.executor.shutdown()
        self.node.destroy_node()
        self.ctx.shutdown()


@pytest.fixture
def rig_factory(monkeypatch):
    rigs = []

    def make(**params):
        rig = SlidingMapRig(monkeypatch, **params)
        rigs.append(rig)
        return rig

    yield make
    for rig in rigs:
        rig.close()


def cell(msg, wx, wy):
    mx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
    my = int((wy - msg.info.origin.position.y) / msg.info.resolution)
    return msg.data[my * msg.info.width + mx]


def _wall_ranges(angles, point, heading):
    px, py = point
    dx, dy = math.cos(heading), math.sin(heading)
    ranges = []
    for a in angles:
        cx, cy = math.cos(a), math.sin(a)
        den = cx * dy - cy * dx
        ranges.append((px * dy - py * dx) / den)
    return ranges


def test_oblique_wall_hits_survive_same_frame(rig_factory):
    rig = rig_factory()
    inc = 0.017
    angles = [-0.5 + i * inc for i in range(int(1.0 / inc) + 1)]
    ranges = _wall_ranges(angles, (2.0, 0.0), math.radians(30.0))
    grid, _ = rig.send_frame(0.0, ranges, angle_min=angles[0], angle_increment=inc)
    # Returns beyond max_fake_range (4.5 m) are dropped by design; check the accepted ones.
    hits = [(r, a) for r, a in zip(ranges, angles) if 0.2 <= r < 4.5]
    assert len(hits) > 30
    lost = [(r, a) for r, a in hits if cell(grid, r * math.cos(a), r * math.sin(a)) != 100]
    assert not lost, f'{len(lost)}/{len(hits)} wall hits erased by later beams of the same frame'


def test_fake_scan_angle_contract(rig_factory):
    rig = rig_factory()
    _, fake = rig.send_frame(0.0, [2.0, 2.0])
    assert len(fake.ranges) == 360
    last = fake.angle_min + (len(fake.ranges) - 1) * fake.angle_increment
    assert fake.angle_max == pytest.approx(last, abs=1e-6)


def test_no_return_beam_keeps_near_memory(rig_factory):
    rig = rig_factory()
    rig.send_frame(0.0, [2.0, math.inf])
    grid, _ = rig.send_frame(0.2, [0.25, math.inf])
    assert cell(grid, 0.25, 0.0) == 100
    assert cell(grid, 2.0, 0.0) == 100
    grid, _ = rig.send_frame(0.4, [math.inf, math.inf])
    assert cell(grid, 0.25, 0.0) == 100, 'obstacle inside the near blind zone was erased'
    assert cell(grid, 2.0, 0.0) == 0, 'far cells along a no-return beam must still clear'


def test_negative_inf_beam_does_not_clear(rig_factory):
    rig = rig_factory()
    grid, _ = rig.send_frame(0.0, [2.0, math.inf])
    assert cell(grid, 2.0, 0.0) == 100
    grid, _ = rig.send_frame(0.2, [-math.inf, math.inf])
    assert cell(grid, 2.0, 0.0) == 100


def test_fallback_classification_keeps_static_obstacle(rig_factory):
    # Nobody publishes this topic, so the node uses its fallback dynamic classification.
    rig = rig_factory(dynamic_memory_enable=True, dynamic_input_topic='/scan_dynamic_unused',
                      dynamic_decay_sec=0.5, dynamic_min_hold_sec=0.5,
                      dynamic_require_vacated=True)
    # First sighting of a static obstacle at 1.04 m; beam 1 sees free space.
    rig.send_frame(0.0, [1.04, math.inf])
    # An occluder appears in front of it and something appears in previously free space nearby.
    rig.send_frame(0.2, [0.5, 1.04])
    grid, _ = rig.send_frame(1.0, [0.5, 1.04])
    assert cell(grid, 1.04, 0.0) == 100, 'static obstacle was classified dynamic and forgotten'


def test_vacated_hold_counts_from_vacate_time(rig_factory):
    rig = rig_factory(dynamic_memory_enable=True, dynamic_input_topic='/scan_dynamic',
                      dynamic_decay_sec=0.3, dynamic_min_hold_sec=1.0,
                      dynamic_require_vacated=True)
    rig.send_frame(0.0, [math.inf, math.inf], dynamic=[0.0, 0.0])
    rig.send_frame(0.2, [1.0, math.inf], dynamic=[1.0, 0.0])
    # Target moves to beam 1 and beam 0 is occluded: the old cell becomes vacated at t=0.9 s.
    rig.send_frame(0.9, [0.5, 1.0], dynamic=[0.0, 1.0])
    grid, _ = rig.send_frame(1.5, [0.5, 1.0], dynamic=[0.0, 1.0])
    assert cell(grid, 1.0, 0.0) == 100, 'vacated cell dropped before dynamic_min_hold_sec elapsed'
    grid, _ = rig.send_frame(2.0, [0.5, 1.0], dynamic=[0.0, 1.0])
    assert cell(grid, 1.0, 0.0) == -1
