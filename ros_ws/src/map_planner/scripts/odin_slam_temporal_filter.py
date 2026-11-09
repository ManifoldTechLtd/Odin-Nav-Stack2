#!/usr/bin/env python3
"""ROS1 wrapper for the SW01 cloud_slam geometric temporal filter."""
import json
import os
import sys
import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slam_cloud_filter import SlamVoxelFilter


def _matrix_from_transform(transform):
    rotation = transform.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]
    translation = transform.transform.translation
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class OdinSlamTemporalFilter:
    def __init__(self):
        rospy.init_node("odin_cloud_temporal_filter")
        self.enable = rospy.get_param("~enable", True)
        self.input_topic = rospy.get_param("~input_topic", "/odin1/cloud_slam")
        self.output_topic = rospy.get_param("~output_topic", "/odin1/cloud_filtered")
        self.trace_topic = rospy.get_param("~trace_topic", "/odin_cloud_temporal_filter/trace")
        self.fixed_frame = rospy.get_param("~fixed_frame", "odom")
        self.reference_frame = rospy.get_param("~reference_frame", "lidar")
        self.expected_input_frame = rospy.get_param("~expected_input_frame", "odom")
        self.tf_timeout = rospy.Duration(rospy.get_param("~tf_timeout_sec", 0.02))
        self.core = SlamVoxelFilter(
            roi_radius_m=rospy.get_param("~roi_radius_m", 0.8),
            voxel_size_m=rospy.get_param("~voxel_size_m", 0.03),
            min_range_drop_m=rospy.get_param("~min_range_drop_m", 0.10),
            angular_bin_deg=rospy.get_param("~angular_bin_deg", 0.5),
            max_frame_gap_sec=rospy.get_param("~max_frame_gap_sec", 0.30),
            pose_jump_m=rospy.get_param("~pose_jump_m", 0.5),
            pose_jump_deg=rospy.get_param("~pose_jump_deg", 30.0),
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cloud_pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.trace_pub = rospy.Publisher(self.trace_topic, String, queue_size=20)
        self.frame_count = 0
        self.total_removed = 0
        rospy.Subscriber(self.input_topic, PointCloud2, self._callback,
                         queue_size=1, tcp_nodelay=True)
        rospy.loginfo(
            "[odin_cloud_temporal_filter] input=%s output=%s fixed=%s reference=%s "
            "roi=%.2f voxel=%.3f drop=%.2f bin=%.2fdeg gap=%.2f tf_timeout=%.3fs",
            self.input_topic, self.output_topic, self.fixed_frame, self.reference_frame,
            self.core.roi_radius_m, self.core.voxel_size_m, self.core.min_range_drop_m,
            np.rad2deg(self.core.bin_rad), self.core.max_frame_gap_sec,
            self.tf_timeout.to_sec())

    def _callback(self, msg):
        started = time.monotonic()
        self.frame_count += 1
        stats = {
            "stamp": msg.header.stamp.to_sec(),
            "frame": self.frame_count,
            "input_points": msg.width * msg.height,
            "valid_points": 0,
            "roi_points": 0,
            "kept_roi_points": 0,
            "sudden_near_points": 0,
            "removed_points": 0,
            "total_removed_points": self.total_removed,
            "enable": self.enable,
            "warmup": False,
            "reset_reason": "",
            "tf_status": "unavailable",
            "algorithm": "slam_voxel_raycast",
        }
        try:
            if not self.enable:
                self._fail(msg, stats, "disabled")
                return
            if msg.header.stamp.is_zero():
                self._fail(msg, stats, "invalid_stamp")
                return
            if self.expected_input_frame and msg.header.frame_id != self.expected_input_frame:
                self._fail(msg, stats, "unexpected_frame")
                return
            if not self._valid_layout(msg):
                self._fail(msg, stats, "schema")
                return
            transform = self._lookup_transform(msg.header.stamp)
            if transform is None:
                self._fail(msg, stats, "tf_unavailable")
                return
            stats["tf_status"] = "ready"
            xyz = self._extract_xyz(msg)
            keep, core_stats = self.core.process(
                xyz, transform, stamp_ns=msg.header.stamp.to_nsec())
            stats.update(core_stats)
            self.total_removed = core_stats["total_removed_points"]
            self._publish(msg, keep)
        except Exception as exc:
            stats["reset_reason"] = "exception"
            stats["error"] = str(exc)
            self.core.reset()
            self._publish(msg, None)
            rospy.logerr_throttle(2.0, "[odin_cloud_temporal_filter] %s", exc)
        finally:
            stats["total_removed_points"] = self.total_removed
            stats["elapsed_ms"] = (time.monotonic() - started) * 1000.0
            self.trace_pub.publish(String(data=json.dumps(stats, separators=(",", ":"))))
            rospy.loginfo_throttle(
                2.0, "[odin_cloud_temporal_filter] in=%d roi=%d removed=%d tf=%s %.1fms",
                stats["input_points"], stats["roi_points"], stats["removed_points"],
                stats["tf_status"], stats["elapsed_ms"])

    def _lookup_transform(self, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.reference_frame, stamp, self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return _matrix_from_transform(transform)

    @staticmethod
    def _valid_layout(msg):
        if msg.is_bigendian or msg.height != 1 or msg.row_step != msg.width * msg.point_step:
            return False
        fields = {field.name: field for field in msg.fields}
        for name in ("x", "y", "z"):
            field = fields.get(name)
            if field is None or field.datatype != PointField.FLOAT32:
                return False
        return True

    @staticmethod
    def _extract_xyz(msg):
        count = msg.width * msg.height
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * step).reshape(count, step)
        fields = {field.name: field for field in msg.fields}
        xyz = np.empty((count, 3), dtype=np.float32)
        for index, name in enumerate(("x", "y", "z")):
            offset = fields[name].offset
            xyz[:, index] = raw[:, offset:offset + 4].copy().view(np.float32).reshape(count)
        return xyz

    def _publish(self, msg, keep):
        if keep is None or bool(keep.all()):
            self.cloud_pub.publish(msg)
            return
        count = msg.width * msg.height
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * step).reshape(count, step)
        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = int(keep.sum())
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = step
        output.row_step = output.width * step
        output.data = raw[keep].tobytes()
        output.is_dense = msg.is_dense
        self.cloud_pub.publish(output)

    def _fail(self, msg, stats, reason):
        stats["reset_reason"] = reason
        stats["warmup"] = True
        self.core.reset()
        self._publish(msg, None)


if __name__ == "__main__":
    OdinSlamTemporalFilter()
    rospy.spin()
