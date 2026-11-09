#!/usr/bin/env python3
"""Simulated Odin driver node for hardware-free ROS 2 stack verification.

Publishes:
  - Dynamic TF (~10 Hz): `odom->imu`, `odom->map`, `imu->lidar`, `odom->camera_0`
  - `/odin1/odometry` (`frame_id=odom`, `child_frame_id=imu`)
  - `/odin1/cloud_slam` (`odom` frame) and `/odin1/cloud_raw` (`lidar` frame)
  - Readiness log lines matching Gate 1 ("Device ready and streams activated") and
    Gate 2 ("relocalization success!").
"""
import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import TransformBroadcaster


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MockOdinDriver(Node):
    def __init__(self, motion: str = "static", hz: float = 10.0,
                 x0: float = 0.0, y0: float = 0.0, yaw0: float = 0.0, **kwargs):
        super().__init__("mock_odin_driver", **kwargs)
        self.motion = motion
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.yaw0 = float(yaw0)
        self.t0 = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "/odin1/odometry", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/odin1/cloud_slam", 10)
        self.pub_raw = self.create_publisher(PointCloud2, "/odin1/cloud_raw", 10)
        self._raw_points = None
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self._reloc_at = 8.0
        self._reloc_done = False
        self.get_logger().info("Device ready and streams activated")

    def _pose(self, t: float):
        if self.motion == "circle":
            th = 2.0 * math.pi * t / 30.0
            return self.x0 + math.cos(th) - 1.0, self.y0 + math.sin(th), th
        return self.x0, self.y0, self.yaw0

    def _tf(self, stamp, parent, child, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        if not self._reloc_done and t >= self._reloc_at:
            self._reloc_done = True
            self.get_logger().info("relocalization success!")
        x, y, yaw = self._pose(t)
        stamp = now.to_msg()

        self.tf_broadcaster.sendTransform([
            self._tf(stamp, "odom", "imu", x, y, 0.0, yaw),
            self._tf(stamp, "odom", "map", 0.0, 0.0, 0.0, 0.0),
            self._tf(stamp, "imu", "lidar", 0.0, 0.0, 0.0, 0.0),
            self._tf(stamp, "odom", "camera_0", x, y, 0.0, yaw),
        ])

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "imu"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.pub_odom.publish(od)

        pts = [(3.5 + 0.05 * math.sin(i), -0.3 + 0.6 * i / 29.0, 0.0) for i in range(30)]
        rng = np.random.default_rng(int(t * 10) % 1000)
        for _ in range(40):
            for _try in range(20):
                bx, by = float(rng.uniform(-6, 6)), float(rng.uniform(-6, 6))
                if math.hypot(bx, by) > 3.0:
                    break
            pts.append((bx, by, float(rng.uniform(-0.2, 0.2))))
        header = od.header.__class__()
        header.stamp = stamp
        header.frame_id = "odom"
        c, s = math.cos(yaw), math.sin(yaw)
        fixed_points = [(x + c * px - s * py, y + s * px + c * py, pz) for px, py, pz in pts]
        self.pub_cloud.publish(pc2.create_cloud_xyz32(header, fixed_points))
        if self._raw_points is None:
            self._raw_points = pts
        header.frame_id = "lidar"
        self.pub_raw.publish(pc2.create_cloud_xyz32(header, self._raw_points))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motion", choices=["static", "circle"], default="static")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--x", type=float, default=0.0, help="Initial robot X in map frame")
    ap.add_argument("--y", type=float, default=0.0, help="Initial robot Y in map frame")
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="Initial robot yaw in degrees")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = MockOdinDriver(motion=args.motion, hz=args.hz, x0=args.x, y0=args.y,
                          yaw0=math.radians(args.yaw_deg))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
