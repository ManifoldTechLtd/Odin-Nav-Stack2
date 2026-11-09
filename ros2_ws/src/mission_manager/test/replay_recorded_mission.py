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

"""Run real planner/mission binaries on frozen bag geometry with a fake action."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

import numpy as np
import rclpy
from nav_interfaces.action import NavPath
from nav_interfaces.msg import Waypoint
from nav_interfaces.srv import StartMission, ValidatePath
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped

from test_navigation_path_safety import NavigationRig, wait_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture", type=Path)
    parser.add_argument("mapper")
    parser.add_argument("mission")
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "184"
    assert os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"
    rclpy.init()
    rig = NavigationRig.__new__(NavigationRig)
    rig.node = rclpy.create_node("recorded_mission_" + uuid.uuid4().hex[:8])
    rig.executor = MultiThreadedExecutor(num_threads=4)
    rig.executor.add_node(rig.node)
    rig.spin_thread = threading.Thread(target=rig.executor.spin, daemon=True)
    rig.spin_thread.start()
    rig.running = True
    rig.paths, rig.stops, rig.events, rig.processes, rig.logs = [], [], [], [], []
    rig.latest_handle = None
    rig.reject_next, rig.rejected_goals = False, []
    rig.robot = [10.0789, 1.06287, 0.0]
    rig.action = ActionServer(
        rig.node, NavPath, "/neupan/nav_path", rig.execute_path,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup())
    static_tf = StaticTransformBroadcaster(rig.node)
    static_tf.sendTransform(deserialize_message(
        (args.fixture / "tf.cdr").read_bytes(), TFMessage).transforms)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    fake_pub = rig.node.create_publisher(OccupancyGrid, "/sliding_map", qos)
    scan_pub = rig.node.create_publisher(LaserScan, "/scan_360", 5)
    fake = deserialize_message((args.fixture / "fake.cdr").read_bytes(), OccupancyGrid)

    def pump():
        while rig.running:
            fake.header.stamp = rig.node.get_clock().now().to_msg()
            fake_pub.publish(fake)
            scan = LaserScan()
            scan.header.frame_id = "base_link"
            scan.header.stamp = fake.header.stamp
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180
            scan.range_min, scan.range_max = 0.05, 4.5
            scan.ranges = [math.inf] * 360
            scan_pub.publish(scan)
            time.sleep(0.08)
    rig.pump_thread = threading.Thread(target=pump, daemon=True)
    rig.pump_thread.start()
    try:
        for name, command in [
            ("map", [args.mapper, str(args.fixture), "--serve", "--ros-args",
                     "--params-file", str(args.fixture / "params.yaml")]),
            ("mission", [args.mission, "--ros-args",
                         "-p", "navigation_mode:=replan_periodic",
                         "-p", "enable_alignment:=false"]),
        ]:
            log = open("/tmp/" + rig.node.get_name() + "_" + name + ".log", "w+")
            rig.logs.append(log)
            rig.processes.append(subprocess.Popen(command, stdout=log, stderr=log))
        client = rig.node.create_client(StartMission, "/waypoint_manager/start_mission")
        assert client.wait_for_service(timeout_sec=10.0)
        goal = deserialize_message((args.fixture / "goal.cdr").read_bytes(), PoseStamped)
        waypoint = Waypoint()
        waypoint.x, waypoint.y = goal.pose.position.x, goal.pose.position.y
        request = StartMission.Request()
        request.waypoints = [waypoint]
        begin = time.monotonic()
        response = rig.call(client, request, timeout=2.0)
        accepted = time.monotonic() - begin
        assert response.success, response.message
        assert accepted < 0.6, "Full-map planning blocked mission acceptance"
        wait_until(lambda: rig.paths, timeout=20.0)
        installed = time.monotonic() - begin
        path = rig.paths[0]
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        assert 8.0 < length < 11.0, "Expected the first 10 m segment of the recorded far route"
        check = rig.node.create_client(ValidatePath, "/map_planner/validate_path")
        assert check.wait_for_service(timeout_sec=2.0)
        candidate = ValidatePath.Request()
        candidate.path, candidate.check_start_connection = path, True
        checked = rig.call(check, candidate)
        assert checked.success, checked.message
        print(json.dumps(dict(accepted_sec=accepted, installed_sec=installed,
                              requested_goal=[waypoint.x, waypoint.y],
                              reference_poses=len(path.poses), reference_length_m=length,
                              final_map_validation=checked.message), indent=2))
    finally:
        rig.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
