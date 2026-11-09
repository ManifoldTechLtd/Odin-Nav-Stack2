#!/usr/bin/env python3
"""ROS 2 Jazzy waypoint queue bridge: buffers /waypoint/goal and dispatches sequentially to /move_base_simple/goal."""
import copy
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_interfaces.msg import MissionStatus, Waypoint
from std_msgs.msg import String
from std_srvs.srv import Trigger


WAYPOINT_GOAL_TOPIC = "/waypoint/goal"
SINGLE_GOAL_TOPIC = "/move_base_simple/goal"
STATUS_TOPIC = "/waypoint_manager/status"
START_CAPTURE_SERVICE = "/waypoint/goal/start"
COMMIT_SERVICE = "/waypoint/goal/commit"
RESET_SERVICE = "/waypoint/goal/reset"
BRIDGE_STATUS_TOPIC = "/waypoint/goal/status"
ACTIVE_STATES = {"NAVIGATING", "ARRIVED", "ALIGNING", "DWELLING", "PAUSED", "RETURNING_HOME"}
TERMINAL_STATES = {"IDLE", "COMPLETED", "CANCELLED", "FAILED"}


class WaypointGoalBridge(Node):
    def __init__(self):
        super().__init__("waypoint_goal_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.lock = threading.RLock()
        self.queue = []
        self.capture_active = False
        self.executing = False
        self.advancing = False
        self.current_index = -1
        self.active_mission_id = ""
        self.latest_status = None

        self.goal_pub = self.create_publisher(PoseStamped, SINGLE_GOAL_TOPIC, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.bridge_status_pub = self.create_publisher(String, BRIDGE_STATUS_TOPIC, latched)
        self.create_subscription(Waypoint, WAYPOINT_GOAL_TOPIC, self._waypoint_callback, 20)
        self.create_subscription(MissionStatus, STATUS_TOPIC, self._status_callback, 10)
        self.create_service(Trigger, START_CAPTURE_SERVICE, self._start_capture)
        self.create_service(Trigger, COMMIT_SERVICE, self._commit)
        self.create_service(Trigger, RESET_SERVICE, self._reset)
        self.get_logger().info(
            "[waypoint_goal_bridge] ready: input=%s output=%s"
            % (WAYPOINT_GOAL_TOPIC, SINGLE_GOAL_TOPIC)
        )

    def _start_capture(self, _request, response):
        with self.lock:
            if self.executing:
                response.success = False
                response.message = "Cannot start capture while executing"
                return response
            self.queue = []
            self.capture_active = True
            self.current_index = -1
            self.active_mission_id = ""
        self._publish_bridge_status("CAPTURING")
        self.get_logger().info("[waypoint_goal_bridge] capture started")
        response.success = True
        response.message = "Waypoint capture started"
        return response

    def _waypoint_callback(self, message):
        with self.lock:
            if not self.capture_active or self.executing:
                self.get_logger().warn(
                    "[waypoint_goal_bridge] ignoring waypoint outside capture phase",
                    throttle_duration_sec=2.0,
                )
                return
            waypoint = copy.deepcopy(message)
            if waypoint.name == "":
                waypoint.name = "waypoint_{}".format(len(self.queue))
            if waypoint.dwell_sec < 0.0:
                self.get_logger().warn("[waypoint_goal_bridge] ignoring negative dwell waypoint")
                return
            self.queue.append(waypoint)
            count = len(self.queue)
        self.get_logger().info(
            "[waypoint_goal_bridge] captured #%d x=%.3f y=%.3f yaw=%.3f dwell=%.3f"
            % (count - 1, waypoint.x, waypoint.y, waypoint.yaw, waypoint.dwell_sec)
        )

    def _commit(self, _request, response):
        with self.lock:
            if self.executing:
                response.success = False
                response.message = "Waypoint queue is already executing"
                return response
            if not self.queue:
                response.success = False
                response.message = "Waypoint queue is empty"
                return response
            if self.latest_status and self.latest_status.state in ACTIVE_STATES:
                response.success = False
                response.message = "Mission manager is active in {}".format(
                    self.latest_status.state)
                return response
            self.capture_active = False
            self.executing = True
            self.advancing = False
            self.current_index = 0
            self.active_mission_id = ""
        self._publish_bridge_status("EXECUTING")
        self._publish_current_goal()
        response.success = True
        response.message = "Waypoint queue execution started"
        return response

    def _reset(self, _request, response):
        with self.lock:
            self.queue = []
            self.capture_active = False
            self.executing = False
            self.advancing = False
            self.current_index = -1
            self.active_mission_id = ""
        self._publish_bridge_status("IDLE")
        self.get_logger().info("[waypoint_goal_bridge] queue reset")
        response.success = True
        response.message = "Waypoint queue reset"
        return response

    def _status_callback(self, message):
        with self.lock:
            self.latest_status = message
            if not self.executing:
                return
            if not self.active_mission_id:
                if message.state == "NAVIGATING":
                    self.active_mission_id = message.mission_id
                    self.get_logger().info(
                        "[waypoint_goal_bridge] waypoint #%d mission started: %s"
                        % (self.current_index, self.active_mission_id)
                    )
                return
            if message.mission_id != self.active_mission_id:
                return
            if message.state == "FAILED":
                self.executing = False
                self._publish_bridge_status("FAILED")
                self.get_logger().error(
                    "[waypoint_goal_bridge] waypoint #%d failed: %s"
                    % (self.current_index, message.mission_id)
                )
                return
            if message.state == "CANCELLED":
                self.executing = False
                self._publish_bridge_status("CANCELLED")
                self.get_logger().warn(
                    "[waypoint_goal_bridge] waypoint #%d cancelled" % self.current_index
                )
                return
            if message.state == "COMPLETED" and not self.advancing:
                self.advancing = True
                index = self.current_index
                dwell = self.queue[index].dwell_sec
                mission_id = self.active_mission_id
                threading.Thread(
                    target=self._advance_after_dwell,
                    args=(index, dwell, mission_id),
                    daemon=True,
                ).start()

    def _advance_after_dwell(self, index, dwell, mission_id):
        if dwell > 0.0:
            self.get_logger().info(
                "[waypoint_goal_bridge] waypoint #%d completed; dwelling %.3f s"
                % (index, dwell)
            )
            time.sleep(dwell)
        with self.lock:
            if not self.executing or self.current_index != index:
                return
            if self.active_mission_id != mission_id:
                return
            if index + 1 >= len(self.queue):
                self.executing = False
                self.advancing = False
                self.active_mission_id = ""
                self._publish_bridge_status("COMPLETED")
                self.get_logger().info("[waypoint_goal_bridge] waypoint queue completed")
                return
            self.current_index += 1
            self.active_mission_id = ""
            self.advancing = False
        self._publish_current_goal()

    def _publish_bridge_status(self, state):
        self.bridge_status_pub.publish(String(data=state))

    def _publish_current_goal(self):
        with self.lock:
            if not self.executing or self.current_index < 0:
                return
            waypoint = copy.deepcopy(self.queue[self.current_index])
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        message.pose.position.z = 0.0
        half = waypoint.yaw / 2.0
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = math.sin(half)
        message.pose.orientation.w = math.cos(half)
        self.goal_pub.publish(message)
        self.get_logger().info(
            "[waypoint_goal_bridge] published waypoint #%d to %s x=%.3f y=%.3f yaw=%.3f"
            % (self.current_index, SINGLE_GOAL_TOPIC, waypoint.x, waypoint.y, waypoint.yaw)
        )


def main():
    rclpy.init()
    node = WaypointGoalBridge()
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
