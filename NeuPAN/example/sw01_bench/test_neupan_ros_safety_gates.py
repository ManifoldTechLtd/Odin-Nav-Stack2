#!/usr/bin/env python3
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

"""Safety gates of the NeuPAN ROS control cycle (ROS 2 shim + real SW01 planner).

Needs a sourced ROS 2 workspace with nav_interfaces, torch and cvxpy, e.g.:
  source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
  LD_PRELOAD=/opt/ros/jazzy/lib/libtf2.so:/opt/ros/jazzy/lib/libtf2_ros.so \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 <conda neupan python> -m pytest <this file>
"""
from math import cos, sin
from pathlib import Path
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT.parent / "robots" / "sw01" / "neupan" / "config.yaml"
sys.path.insert(0, str(ROOT))


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def core(ros_context, monkeypatch):
    from rclpy.parameter import Parameter
    from neupan.ros import ros2_shim
    from neupan.ros.neupan_ros import neupan_core

    monkeypatch.setenv("NEUPAN_ROS_CONFIG", str(CONFIG))
    node = rclpy.create_node(
        "neupan_gate_test_%d" % (time.monotonic_ns() % 1000000),
        parameter_overrides=[Parameter("navigation_mode", value="replan_periodic")])
    ros2_shim.bind_node(node)
    instance = neupan_core()
    published = []
    modes = []
    monkeypatch.setattr(instance.vel_pub, "publish", published.append)
    trace = instance._publish_control_trace
    monkeypatch.setattr(instance, "_publish_control_trace",
                        lambda mode, *a, **k: (modes.append(mode), trace(mode, *a, **k)))
    instance.test_published = published
    instance.test_modes = modes
    yield instance
    node.destroy_node()


def _set_tf(core, x=0.0, y=0.0, yaw=0.0, age=0.0):
    from geometry_msgs.msg import TransformStamped
    from neupan.ros import ros2_shim

    now = ros2_shim._ensure_node().get_clock().now().nanoseconds - int(age * 1e9)
    for child, offset in (("base_link", 0.0), ("odin1_base_link", 0.42)):
        tf_msg = TransformStamped()
        tf_msg.header.stamp.sec = now // 10**9
        tf_msg.header.stamp.nanosec = now % 10**9
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = x + offset * cos(yaw)
        tf_msg.transform.translation.y = y + offset * sin(yaw)
        tf_msg.transform.rotation.z = sin(yaw / 2.0)
        tf_msg.transform.rotation.w = cos(yaw / 2.0)
        core.listener._buffer.set_transform(tf_msg, "test")


def _set_path(core, x_end=5.0):
    core.neupan_planner.set_initial_path([
        np.array([[x], [0.0], [0.0], [1.0]]) for x in np.arange(0.0, x_end + 0.01, 0.1)])


def _feed_scan(core):
    from sensor_msgs.msg import LaserScan

    scan = LaserScan()
    scan.angle_min = -3.14159
    scan.angle_increment = 2 * 3.14159 / 360
    scan.angle_max = scan.angle_min + 359 * scan.angle_increment
    ranges = [float("inf")] * 360
    ranges[90] = 3.0  # one point well to the side of the path
    scan.ranges = ranges
    core.scan_callback(scan)


def _is_zero(msg):
    return msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0


def test_fresh_scan_drives_along_the_path(core):
    _set_tf(core)
    core._control_cycle()
    _set_path(core)
    _feed_scan(core)
    core._control_cycle()
    assert core.test_modes[-1] == "MPC"
    assert core.test_published and core.test_published[-1].linear.x > 0.0


def test_stale_scan_publishes_zero(core):
    _set_tf(core)
    core._control_cycle()
    _set_path(core)
    _feed_scan(core)
    time.sleep(0.6)
    _set_tf(core)
    core._control_cycle()
    assert core.test_modes[-1] == "WAIT_SCAN"
    assert core.test_published and _is_zero(core.test_published[-1])


def test_tf_failure_publishes_zero(core):
    _set_path(core)
    core._control_cycle()
    assert core.test_modes[-1] == "WAIT_TF"
    assert core.test_published and _is_zero(core.test_published[-1])


def test_arrival_publishes_one_zero_command(core):
    _set_tf(core, x=0.95)
    core._control_cycle()
    _set_path(core, x_end=1.0)
    _feed_scan(core)
    core._control_cycle()
    assert len(core.test_published) == 1 and _is_zero(core.test_published[-1])
    _feed_scan(core)
    core._control_cycle()
    assert len(core.test_published) == 1


def test_missing_config_fails_fast(ros_context, monkeypatch):
    from neupan.ros import ros2_shim
    from neupan.ros.neupan_ros import neupan_core

    monkeypatch.setenv("NEUPAN_ROS_CONFIG", str(CONFIG.parent / "does_not_exist.yaml"))
    node = rclpy.create_node("neupan_gate_missing_cfg")
    ros2_shim.bind_node(node)
    try:
        with pytest.raises(FileNotFoundError):
            neupan_core()
    finally:
        node.destroy_node()


def test_empty_obstacle_array_does_not_break_the_cycle(core):
    _set_tf(core)
    core._control_cycle()
    _set_path(core)
    _feed_scan(core)
    core._control_cycle()
    core.obstacle_points = np.zeros((2, 0))
    _set_tf(core)
    core._control_cycle()
    assert core.test_modes[-1] == "MPC"


def test_clear_navigation_stops_even_if_planner_reset_fails(ros_context, monkeypatch):
    import importlib.util
    from neupan.ros import ros2_shim

    monkeypatch.setenv("NEUPAN_ROS_CONFIG", str(CONFIG))
    script = ROOT.parent / "ros2_ws" / "src" / "neupan_ros" / "scripts" / "neupan_action_node.py"
    spec = importlib.util.spec_from_file_location("neupan_action_node_under_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.NeupanActionCompat()
    try:
        published = []
        monkeypatch.setattr(node.vel_pub, "publish", published.append)

        def failing_reset(*args, **kwargs):
            raise RuntimeError("reset failed")

        monkeypatch.setattr(node.neupan_planner, "reset", failing_reset)
        with pytest.raises(RuntimeError):
            node._clear_navigation("test")
        assert published and _is_zero(published[-1])
        assert node._control_cycle_token() is None
    finally:
        node.destroy_node()
        ros2_shim.bind_node(None)
