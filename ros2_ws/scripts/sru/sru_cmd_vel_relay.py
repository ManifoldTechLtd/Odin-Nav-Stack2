#!/usr/bin/env python3
"""Gated cmd_vel relay for the SRU local planner backend (ROS 2 Jazzy).

Forwards `/sru/cmd_vel` to `/cmd_vel` only when `/sru/cmd_vel_enable` (transient_local Bool)
is True, yielding `/cmd_vel` ownership to `atomic_move` during final heading alignment.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

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
        gate_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, SRU_ENABLE_TOPIC, self._enable_callback, gate_qos)
        self.get_logger().info(
            f"ready: {SRU_CMD_VEL_TOPIC} -> {OUTPUT_CMD_VEL_TOPIC} (gate={SRU_ENABLE_TOPIC}, default closed)"
        )

    def _enable_callback(self, msg):
        enable = bool(msg.data)
        if enable != self.enabled:
            state = "OPEN (forwarding SRU cmd_vel)" if enable else "CLOSED (hand over to atomic_move)"
            self.get_logger().info(f"gate {state}")
        self.enabled = enable

    def _cmd_vel_callback(self, msg):
        if not self.enabled:
            self.suppressed += 1
            return
        self.forwarded += 1
        self.pub.publish(msg)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_stats_ns > 30_000_000_000:
            self._last_stats_ns = now_ns
            self.get_logger().info(
                f"forwarded={self.forwarded} suppressed={self.suppressed} enabled={self.enabled}"
            )


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
