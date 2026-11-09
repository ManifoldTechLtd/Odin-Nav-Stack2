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

"""Accumulate TF-transformed point clouds into a map and save as PCD (ROS 2)."""

import os
import struct
import threading
from datetime import datetime

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from pointcloud_saver.srv import SaveMap


class PointCloudSaver(Node):
    def __init__(self):
        super().__init__("pointcloud_saver_node")

        self.cloud_topic = self.declare_parameter("cloud_topic", "/odin1/cloud_slam").value
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.save_dir = self.declare_parameter("save_dir", os.path.expanduser("~/maps")).value
        self.max_points = self.declare_parameter("max_points", 10000000).value
        self.auto_publish = self.declare_parameter("auto_publish", True).value
        self.online_voxel_size = self.declare_parameter("online_voxel_size", 0.0).value
        self.save_rgb = self.declare_parameter("save_rgb", True).value
        self.max_stat_points = self.declare_parameter("max_stat_points", 200000).value
        self.pcd_binary = self.declare_parameter("pcd_binary", True).value

        self.is_recording = False
        # Storage: online voxel dict (key=(vx,vy,vz) -> [sum_x,sum_y,sum_z,(sum_r,sum_g,sum_b,)count])
        # or raw numpy arrays when online_voxel_size <= 0.
        self.voxel_map = (
            {} if self.online_voxel_size and self.online_voxel_size > 0 else None
        )
        self.map_lock = threading.Lock() if self.voxel_map is not None else None
        self.points_xyz = (
            np.empty((0, 3), dtype=np.float32) if not self.voxel_map else None
        )
        self.points_rgb = (
            np.empty((0, 3), dtype=np.uint8)
            if (not self.voxel_map and self.save_rgb)
            else None
        )
        self.frame_count = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Created save directory: {self.save_dir}")

        from rclpy.qos import qos_profile_sensor_data

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, qos_profile_sensor_data
        )

        if self.auto_publish:
            self.accumulated_pub = self.create_publisher(
                PointCloud2, "/accumulated_map", 1
            )
            self.pub_timer = self.create_timer(2.0, self.publish_accumulated_cloud)

        self.start_srv = self.create_service(
            Trigger, "~/start_recording", self.start_recording_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "~/stop_recording", self.stop_recording_callback
        )
        self.save_srv = self.create_service(SaveMap, "~/save_map", self.save_map_callback)
        self.clear_srv = self.create_service(
            Trigger, "~/clear_map", self.clear_map_callback
        )

        lg = self.get_logger()
        lg.info("=" * 60)
        lg.info("Point cloud saver node started")
        lg.info(f"Subscribed topic: {self.cloud_topic}")
        lg.info(f"Target frame: {self.target_frame}")
        lg.info(f"Save directory: {self.save_dir}")
        lg.info(
            f"Online voxel downsampling: {'enabled' if self.voxel_map is not None else 'disabled'} (voxel={self.online_voxel_size}m)"
        )
        lg.info(f"Save RGB: {'yes' if self.save_rgb else 'no'}")
        lg.info(f"Statistical filter point limit: {self.max_stat_points}")
        lg.info(f"PCD binary format: {'yes' if self.pcd_binary else 'no'}")
        lg.info("=" * 60)
        lg.info("Services:")
        lg.info("  - ~/start_recording : Start recording point cloud")
        lg.info("  - ~/stop_recording  : Stop recording point cloud")
        lg.info("  - ~/save_map        : Save accumulated map as PCD")
        lg.info("  - ~/clear_map       : Clear accumulated point cloud")
        lg.info("=" * 60)

    def _extract_rgb(self, rgb_value):
        """Unpack PointCloud2 rgb field (float32 or uint32) into (r, g, b)."""
        if isinstance(rgb_value, (float, np.floating)):
            rgb_int = struct.unpack("I", struct.pack("f", float(rgb_value)))[0]
        else:
            rgb_int = int(rgb_value)
        return (rgb_int >> 16) & 0xFF, (rgb_int >> 8) & 0xFF, rgb_int & 0xFF

    def cloud_callback(self, msg):
        if not self.is_recording:
            return

        try:
            if self.voxel_map is not None:
                if self.map_lock is not None:
                    with self.map_lock:
                        current_points = len(self.voxel_map)
                else:
                    current_points = len(self.voxel_map)
            else:
                current_points = int(self.points_xyz.shape[0])
            if current_points >= self.max_points:
                if self.frame_count % 100 == 0:
                    self.get_logger().warn(f"Reached max point limit {self.max_points}, stopping accumulation")
                return

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                if self.frame_count % 50 == 0:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                return

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

            points_np = pc2.read_points(msg, skip_nans=True)
            field_names = points_np.dtype.names if points_np.dtype.names else ()
            has_rgb_field = self.save_rgb and ("rgb" in field_names)
            new_xyz = []
            new_rgb = []
            for point in points_np:
                x, y, z = float(point["x"]), float(point["y"]), float(point["z"])

                point_local = np.array([x, y, z])
                point_map = R @ point_local + np.array([tx, ty, tz])

                if self.save_rgb:
                    if has_rgb_field:
                        r, g, b = self._extract_rgb(point["rgb"])
                    else:
                        r = g = b = 255

                if self.voxel_map is not None and self.online_voxel_size > 0:
                    vx = int(np.floor(point_map[0] / self.online_voxel_size))
                    vy = int(np.floor(point_map[1] / self.online_voxel_size))
                    vz = int(np.floor(point_map[2] / self.online_voxel_size))
                    key = (vx, vy, vz)
                    if self.map_lock is not None:
                        with self.map_lock:
                            if key not in self.voxel_map:
                                if self.save_rgb:
                                    self.voxel_map[key] = [
                                        float(point_map[0]),
                                        float(point_map[1]),
                                        float(point_map[2]),
                                        float(r),
                                        float(g),
                                        float(b),
                                        1.0,
                                    ]
                                else:
                                    self.voxel_map[key] = [
                                        float(point_map[0]),
                                        float(point_map[1]),
                                        float(point_map[2]),
                                        1.0,
                                    ]
                            else:
                                acc = self.voxel_map[key]
                                acc[0] += float(point_map[0])
                                acc[1] += float(point_map[1])
                                acc[2] += float(point_map[2])
                                if self.save_rgb:
                                    acc[3] += float(r)
                                    acc[4] += float(g)
                                    acc[5] += float(b)
                                    acc[6] += 1.0
                                else:
                                    acc[3] += 1.0
                    else:
                        if key not in self.voxel_map:
                            if self.save_rgb:
                                self.voxel_map[key] = [
                                    float(point_map[0]),
                                    float(point_map[1]),
                                    float(point_map[2]),
                                    float(r),
                                    float(g),
                                    float(b),
                                    1.0,
                                ]
                            else:
                                self.voxel_map[key] = [
                                    float(point_map[0]),
                                    float(point_map[1]),
                                    float(point_map[2]),
                                    1.0,
                                ]
                        else:
                            acc = self.voxel_map[key]
                            acc[0] += float(point_map[0])
                            acc[1] += float(point_map[1])
                            acc[2] += float(point_map[2])
                            if self.save_rgb:
                                acc[3] += float(r)
                                acc[4] += float(g)
                                acc[5] += float(b)
                                acc[6] += 1.0
                            else:
                                acc[3] += 1.0
                else:
                    new_xyz.append([point_map[0], point_map[1], point_map[2]])
                    if self.save_rgb:
                        new_rgb.append([r, g, b])

            if self.voxel_map is None:
                if new_xyz:
                    new_xyz_np = np.array(new_xyz, dtype=np.float32)
                    self.points_xyz = np.concatenate(
                        [self.points_xyz, new_xyz_np], axis=0
                    )
                    if self.save_rgb:
                        new_rgb_np = np.array(new_rgb, dtype=np.uint8)
                        self.points_rgb = np.concatenate(
                            [self.points_rgb, new_rgb_np], axis=0
                        )
            self.frame_count += 1

            if self.frame_count % 10 == 0:
                if self.voxel_map is not None:
                    self.get_logger().info(
                        f"Accumulated {len(self.voxel_map)} voxel centroids from {self.frame_count} frames"
                    )
                else:
                    self.get_logger().info(
                        f"Accumulated {self.points_xyz.shape[0]} points from {self.frame_count} frames"
                    )

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        R = np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ]
        )
        return R

    def publish_accumulated_cloud(self):
        if self.voxel_map is not None:
            if self.map_lock is not None:
                with self.map_lock:
                    values_snapshot = list(self.voxel_map.values())
            else:
                values_snapshot = list(self.voxel_map.values())
            if len(values_snapshot) == 0:
                return
            centroids = np.zeros((len(values_snapshot), 3), dtype=np.float32)
            idx = 0
            for v in values_snapshot:
                if self.save_rgb:
                    count = max(1.0, v[6])
                    centroids[idx, 0] = v[0] / count
                    centroids[idx, 1] = v[1] / count
                    centroids[idx, 2] = v[2] / count
                else:
                    count = max(1.0, v[3])
                    centroids[idx, 0] = v[0] / count
                    centroids[idx, 1] = v[1] / count
                    centroids[idx, 2] = v[2] / count
                idx += 1
            points_to_publish = centroids
        else:
            if self.points_xyz is None or self.points_xyz.shape[0] == 0:
                return
            points_to_publish = self.points_xyz

        try:
            if points_to_publish.shape[0] > 100000:
                step = max(1, points_to_publish.shape[0] // 100000)
                points_to_publish = points_to_publish[::step]

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame

            points_xyz = [
                (float(p[0]), float(p[1]), float(p[2])) for p in points_to_publish
            ]
            cloud_msg = pc2.create_cloud_xyz32(header, points_xyz)

            self.accumulated_pub.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing accumulated point cloud: {e}")

    def voxel_filter(self, points, voxel_size):
        self.get_logger().info(f"Applying voxel filter, voxel_size: {voxel_size}m")

        if len(points) == 0:
            return points

        voxel_dict = {}
        has_rgb = len(points[0]) >= 6

        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]

            vx = int(x / voxel_size)
            vy = int(y / voxel_size)
            vz = int(z / voxel_size)
            voxel_key = (vx, vy, vz)

            if voxel_key not in voxel_dict:
                if has_rgb:
                    voxel_dict[voxel_key] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]
                else:
                    voxel_dict[voxel_key] = [0.0, 0.0, 0.0, 0]
            acc = voxel_dict[voxel_key]
            acc[0] += float(x)
            acc[1] += float(y)
            acc[2] += float(z)
            if has_rgb:
                acc[3] += float(point[3])
                acc[4] += float(point[4])
                acc[5] += float(point[5])
                acc[6] += 1
            else:
                acc[3] += 1

        filtered_points = []
        if has_rgb:
            for acc in voxel_dict.values():
                count = max(1, acc[6])
                avg_x = acc[0] / count
                avg_y = acc[1] / count
                avg_z = acc[2] / count
                avg_r = int(acc[3] / count)
                avg_g = int(acc[4] / count)
                avg_b = int(acc[5] / count)
                filtered_points.append((avg_x, avg_y, avg_z, avg_r, avg_g, avg_b))
        else:
            for acc in voxel_dict.values():
                count = max(1, acc[3])
                avg_x = acc[0] / count
                avg_y = acc[1] / count
                avg_z = acc[2] / count
                filtered_points.append((avg_x, avg_y, avg_z))

        self.get_logger().info(f"Voxel filter: {len(points)} -> {len(filtered_points)} points")
        return filtered_points

    def voxel_filter_fast(self, points, voxel_size):
        """Vectorized numpy voxel filter supporting (N, 3) or (N, 6) point arrays."""
        if voxel_size <= 0:
            return np.asarray(
                points, dtype=np.float32 if len(points[0]) == 3 else np.float32
            )

        has_rgb = len(points[0]) >= 6
        if has_rgb:
            if isinstance(points, np.ndarray):
                xyz = points[:, :3].astype(np.float32, copy=False)
                rgb = points[:, 3:6].astype(np.float32, copy=False)
            else:
                xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
                rgb = np.array([(p[3], p[4], p[5]) for p in points], dtype=np.float32)
        else:
            if isinstance(points, np.ndarray):
                xyz = points[:, :3].astype(np.float32, copy=False)
            else:
                xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
            rgb = None

        if xyz.shape[0] == 0:
            return np.empty((0, 6 if has_rgb else 3), dtype=np.float32)

        vox = np.floor(xyz / float(voxel_size)).astype(np.int64)
        keys, inv, counts = np.unique(
            vox, axis=0, return_inverse=True, return_counts=True
        )

        cx = np.bincount(inv, weights=xyz[:, 0]) / counts
        cy = np.bincount(inv, weights=xyz[:, 1]) / counts
        cz = np.bincount(inv, weights=xyz[:, 2]) / counts
        centroids = np.stack([cx, cy, cz], axis=1).astype(np.float32)

        if has_rgb and rgb is not None:
            cr = np.bincount(inv, weights=rgb[:, 0]) / counts
            cg = np.bincount(inv, weights=rgb[:, 1]) / counts
            cb = np.bincount(inv, weights=rgb[:, 2]) / counts
            colors = np.stack([cr, cg, cb], axis=1).astype(np.float32)
            out = np.concatenate([centroids, colors], axis=1)
        else:
            out = centroids

        self.get_logger().info(f"Voxel filter (fast): {xyz.shape[0]} -> {out.shape[0]} points")
        return out

    def statistical_outlier_filter(self, points, k=50, std_ratio=1.0):
        self.get_logger().info(f"Applying statistical outlier filter, k={k}, std_ratio={std_ratio}")

        if len(points) < k:
            return points
        if len(points) > self.max_stat_points:
            self.get_logger().warn(
                f"Point count {len(points)} exceeds max_stat_points {self.max_stat_points}; skipping statistical filter"
            )
            return points

        points_xyz = np.array([(p[0], p[1], p[2]) for p in points])

        from scipy.spatial import cKDTree

        tree = cKDTree(points_xyz)

        mean_distances = []
        for i, point in enumerate(points_xyz):
            distances, _ = tree.query(point, k=k + 1)
            mean_dist = np.mean(distances[1:])
            mean_distances.append(mean_dist)

        mean_distances = np.array(mean_distances)

        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + std_ratio * global_std

        filtered_points = [
            points[i] for i in range(len(points)) if mean_distances[i] <= threshold
        ]

        self.get_logger().info(f"Statistical filter: {len(points)} -> {len(filtered_points)} points")
        return filtered_points

    def save_pcd(self, points, filename):
        """Write points [(x, y, z[, r, g, b]), ...] to a PCD v0.7 file."""
        self.get_logger().info(f"Saving point cloud to: {filename}")
        has_rgb = self.save_rgb and (len(points) > 0) and (len(points[0]) >= 6)

        if not self.pcd_binary:
            with open(filename, "w") as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                if has_rgb:
                    f.write("FIELDS x y z rgb\n")
                    f.write("SIZE 4 4 4 4\n")
                    f.write("TYPE F F F U\n")
                    f.write("COUNT 1 1 1 1\n")
                else:
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE F F F\n")
                    f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                if has_rgb:
                    for point in points:
                        x, y, z, r, g, b = point
                        rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                        f.write(
                            f"{float(x):.6f} {float(y):.6f} {float(z):.6f} {rgb_int}\n"
                        )
                else:
                    for point in points:
                        x, y, z = point
                        f.write(f"{float(x):.6f} {float(y):.6f} {float(z):.6f}\n")
            self.get_logger().info(f"Saved {len(points)} points (ASCII)")
            return

        N = len(points)
        with open(filename, "wb") as f:
            header = "# .PCD v0.7 - Point Cloud Data file format\n"
            header += "VERSION 0.7\n"
            if has_rgb:
                header += "FIELDS x y z rgb\n"
                header += "SIZE 4 4 4 4\n"
                header += "TYPE F F F U\n"
                header += "COUNT 1 1 1 1\n"
            else:
                header += "FIELDS x y z\n"
                header += "SIZE 4 4 4\n"
                header += "TYPE F F F\n"
                header += "COUNT 1 1 1\n"
            header += f"WIDTH {N}\n"
            header += "HEIGHT 1\n"
            header += "VIEWPOINT 0 0 0 1 0 0 0\n"
            header += f"POINTS {N}\n"
            header += "DATA binary\n"
            f.write(header.encode("ascii"))

            if has_rgb:
                pack_xyz = struct.Struct("<fff").pack
                pack_rgbu = struct.Struct("<I").pack
                for x, y, z, r, g, b in points:
                    f.write(pack_xyz(float(x), float(y), float(z)))
                    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                    f.write(pack_rgbu(rgb_int))
            else:
                pack_xyz = struct.Struct("<fff").pack
                for x, y, z in points:
                    f.write(pack_xyz(float(x), float(y), float(z)))
        self.get_logger().info(f"Saved {len(points)} points (binary)")

    def start_recording_callback(self, request, response):
        self.is_recording = True
        self.get_logger().info("Started recording point cloud")
        response.success = True
        response.message = "开始记录点云"
        return response

    def stop_recording_callback(self, request, response):
        self.is_recording = False
        if self.voxel_map is not None:
            if self.map_lock is not None:
                with self.map_lock:
                    count = len(self.voxel_map)
            else:
                count = len(self.voxel_map)
            self.get_logger().info(f"Stopped recording point cloud ({count} voxel centroids)")
            response.success = True
            response.message = f"停止记录，累积 {count} 个体素中心"
            return response
        else:
            count = int(self.points_xyz.shape[0]) if self.points_xyz is not None else 0
            self.get_logger().info(f"Stopped recording point cloud ({count} points)")
            response.success = True
            response.message = f"停止记录，累积 {count} 个点"
            return response

    def clear_map_callback(self, request, response):
        if self.voxel_map is not None:
            if self.map_lock is not None:
                with self.map_lock:
                    old_count = len(self.voxel_map)
                    self.voxel_map.clear()
            else:
                old_count = len(self.voxel_map)
                self.voxel_map.clear()
            self.frame_count = 0
            self.get_logger().info(f"Cleared accumulated point cloud ({old_count} voxel centroids)")
            response.success = True
            response.message = f"已清空 {old_count} 个体素中心"
            return response
        else:
            old_count = (
                int(self.points_xyz.shape[0]) if self.points_xyz is not None else 0
            )
            self.points_xyz = np.empty((0, 3), dtype=np.float32)
            if self.save_rgb:
                self.points_rgb = np.empty((0, 3), dtype=np.uint8)
            self.frame_count = 0
            self.get_logger().info(f"Cleared accumulated point cloud ({old_count} points)")
            response.success = True
            response.message = f"已清空 {old_count} 个点"
            return response

    def save_map_callback(self, request, response):
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info("Saving map...")

            raw_points = []
            if self.voxel_map is not None:
                if self.map_lock is not None:
                    with self.map_lock:
                        values_snapshot = list(self.voxel_map.values())
                else:
                    values_snapshot = list(self.voxel_map.values())
                if len(values_snapshot) == 0:
                    response.success = False
                    response.message = "没有累积的点云数据"
                    response.saved_path = ""
                    response.total_points = 0
                    response.filtered_points = 0
                    return response
                for v in values_snapshot:
                    if self.save_rgb:
                        count = max(1.0, v[6])
                        raw_points.append(
                            (
                                v[0] / count,
                                v[1] / count,
                                v[2] / count,
                                int(v[3] / count),
                                int(v[4] / count),
                                int(v[5] / count),
                            )
                        )
                    else:
                        count = max(1.0, v[3])
                        raw_points.append((v[0] / count, v[1] / count, v[2] / count))
            else:
                if self.points_xyz is None or self.points_xyz.shape[0] == 0:
                    response.success = False
                    response.message = "没有累积的点云数据"
                    response.saved_path = ""
                    response.total_points = 0
                    response.filtered_points = 0
                    return response
                if (
                    self.save_rgb
                    and self.points_rgb is not None
                    and self.points_rgb.shape[0] == self.points_xyz.shape[0]
                ):
                    for i in range(self.points_xyz.shape[0]):
                        x, y, z = self.points_xyz[i]
                        r, g, b = self.points_rgb[i]
                        raw_points.append(
                            (float(x), float(y), float(z), int(r), int(g), int(b))
                        )
                else:
                    for i in range(self.points_xyz.shape[0]):
                        x, y, z = self.points_xyz[i]
                        raw_points.append((float(x), float(y), float(z)))

            if len(raw_points) == 0:
                response.success = False
                response.message = "没有累积的点云数据"
                response.saved_path = ""
                response.total_points = 0
                response.filtered_points = 0
                return response

            total_points = len(raw_points)
            self.get_logger().info(f"Raw point cloud: {total_points} points")

            filtered_points = list(raw_points)

            voxel_size = request.voxel_size if request.voxel_size > 0 else 0.05
            filtered_np = self.voxel_filter_fast(filtered_points, voxel_size)
            if self.save_rgb and filtered_np.shape[1] >= 6:
                filtered_points = [
                    (
                        float(p[0]),
                        float(p[1]),
                        float(p[2]),
                        int(p[3]),
                        int(p[4]),
                        int(p[5]),
                    )
                    for p in filtered_np
                ]
            else:
                filtered_points = [
                    (float(p[0]), float(p[1]), float(p[2])) for p in filtered_np
                ]

            if request.apply_statistical_filter and len(filtered_points) > 100:
                try:
                    filtered_points = self.statistical_outlier_filter(filtered_points)
                except Exception as e:
                    self.get_logger().warn(f"Statistical filter failed: {e}, skipping")

            if request.filename:
                filename = request.filename
                if not filename.endswith(".pcd"):
                    filename += ".pcd"
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"map_{timestamp}.pcd"

            filepath = os.path.join(self.save_dir, filename)
            self.save_pcd(filtered_points, filepath)

            self.get_logger().info("=" * 60)
            self.get_logger().info("Map saved successfully!")
            self.get_logger().info(f"File path: {filepath}")
            self.get_logger().info(f"Raw points: {total_points}")
            self.get_logger().info(f"Filtered points: {len(filtered_points)}")
            self.get_logger().info(f"Compression ratio: {len(filtered_points)/total_points*100:.1f}%")
            self.get_logger().info("=" * 60)

            response.success = True
            response.message = "地图保存成功"
            response.saved_path = filepath
            response.total_points = total_points
            response.filtered_points = len(filtered_points)
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
            import traceback

            traceback.print_exc()
            response.success = False
            response.message = f"保存失败: {str(e)}"
            response.saved_path = ""
            response.total_points = 0
            response.filtered_points = 0
            return response


def main():
    rclpy.init()
    try:
        saver = PointCloudSaver()
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
