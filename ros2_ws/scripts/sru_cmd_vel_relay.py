#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gated relay for the SRU velocity output (ROS2).

    /sru/cmd_vel -> [this node, gated by /sru/cmd_vel_enable] -> /cmd_vel

The gate (transient_local Bool, published by mission_manager) defaults to
closed: while closed nothing is published, leaving /cmd_vel to atomic_move.
If the relay dies, the chassis bridge sees no publisher and its 0.5 s
timeout commands zero velocity. ROS1 twin: ros_ws/scripts/sru_cmd_vel_relay.py.

Usage: python3 sru_cmd_vel_relay.py
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool

# Contract topics (must match mission_manager; not parameterized by design)
SRU_CMD_VEL_TOPIC = "/sru/cmd_vel"
SRU_ENABLE_TOPIC = "/sru/cmd_vel_enable"
OUTPUT_CMD_VEL_TOPIC = "/cmd_vel"


class SruCmdVelRelay(Node):
    def __init__(self):
        super().__init__("sru_cmd_vel_relay")
        self.enabled = False
        self.forwarded = 0
        self.suppressed = 0
        self._last_stats_ns = self.get_clock().now().nanoseconds

        self.pub = self.create_publisher(Twist, OUTPUT_CMD_VEL_TOPIC, 1)
        self.create_subscription(Twist, SRU_CMD_VEL_TOPIC, self._cmd_vel_callback, 1)
        # Gate QoS must match the publisher (TRANSIENT_LOCAL + RELIABLE),
        # otherwise the enable state is silently missed.
        gate_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, SRU_ENABLE_TOPIC, self._enable_callback, gate_qos)
        self.get_logger().info(
            "ready: %s -> %s (gate=%s, default closed)"
            % (SRU_CMD_VEL_TOPIC, OUTPUT_CMD_VEL_TOPIC, SRU_ENABLE_TOPIC))

    def _enable_callback(self, msg):
        enable = bool(msg.data)
        if enable != self.enabled:
            self.get_logger().info(
                "gate %s" % (
                    "OPEN (forwarding SRU cmd_vel)" if enable
                    else "CLOSED (hand over to atomic_move)"))
        self.enabled = enable

    def _cmd_vel_callback(self, msg):
        if not self.enabled:
            self.suppressed += 1
            return
        self.forwarded += 1
        self.pub.publish(msg)
        # ~30 s stats line (ROS1 twin uses loginfo_throttle)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_stats_ns > 30_000_000_000:
            self._last_stats_ns = now_ns
            self.get_logger().info(
                "forwarded=%d suppressed=%d enabled=%s"
                % (self.forwarded, self.suppressed, self.enabled))


def main():
    rclpy.init()
    node = SruCmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
