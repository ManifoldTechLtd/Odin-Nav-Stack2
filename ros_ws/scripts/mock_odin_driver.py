#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

"""Mock Odin ROS1 driver node for hardware-free navigation stack verification.

Publishes:
  - TF (~10Hz): odom->imu, map->odom
  - /odin1/odometry (nav_msgs/Odometry, frame=odom, child=imu)
  - /odin1/cloud_slam (sensor_msgs/PointCloud2, frame=odin1_base_link)

Usage:
  python3 mock_odin_driver.py [--hz 10] [--obstacle-x 2.5] [--no-obstacle]
"""
import argparse
import math
import threading
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MockOdinRos1:
    def __init__(self, hz=10.0, obstacle_x=2.5, with_obstacle=True):
        rospy.init_node("mock_odin_driver", anonymous=False)
        self.hz = hz
        self.obstacle_x = obstacle_x
        self.with_obstacle = with_obstacle

        self.br = tf2_ros.TransformBroadcaster()
        self.pub_odom = rospy.Publisher("/odin1/odometry", Odometry, queue_size=10)
        self.pub_cloud = rospy.Publisher("/odin1/cloud_slam", PointCloud2, queue_size=10)

        rospy.loginfo("[mock_odin] ready (hz=%.1f, obstacle_x=%.1f, obstacle=%s)",
                      hz, obstacle_x, with_obstacle)
        rospy.loginfo("Device ready and streams activated")

    def _tf(self, parent, child, stamp, yaw=0.0):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _cloud_msg(self, stamp):
        """Generate synthetic obstacle and wall point cloud within pc_to_scan height band."""
        pts = []
        if self.with_obstacle:
            for y in np.arange(-0.3, 0.31, 0.05):
                for z in np.arange(-0.35, 0.26, 0.1):
                    pts.append((float(self.obstacle_x), float(y), float(z)))
        for y in np.arange(-2.0, 2.01, 0.25):
            if abs(y) < 0.5:
                continue
            pts.append((3.0, float(y), -0.2))

        arr = np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), np.float32)
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id="odin1_base_link")
        msg.height = 1
        msg.width = arr.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = arr.tobytes()
        return msg

    def spin(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.br.sendTransform(self._tf("odom", "imu", stamp))
            self.br.sendTransform(self._tf("map", "odom", stamp))

            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = "odom"
            od.child_frame_id = "imu"
            od.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(od)

            self.pub_cloud.publish(self._cloud_msg(stamp))
            rate.sleep()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--obstacle-x", type=float, default=2.5)
    ap.add_argument("--no-obstacle", action="store_true")
    args, _ = ap.parse_known_args()

    node = MockOdinRos1(hz=args.hz, obstacle_x=args.obstacle_x,
                        with_obstacle=not args.no_obstacle)
    node.spin()


if __name__ == "__main__":
    main()
