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

"""Interactive multi-waypoint and RTK navigation test utility for ROS2."""

import argparse
import glob
import json
import math
import os
import shlex
import sys
import threading
import time
from collections import deque


def _bootstrap_ros_env():
    """Re-exec with ROS2 Jazzy and workspace setup sourced if missing."""
    if os.environ.get("NAV_TEST_ENV_BOOTSTRAPPED"):
        return
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_install = os.path.realpath(os.path.join(script_dir, "..", "install"))
    ws_setup = os.path.join(ws_install, "setup.bash")
    ros_setup = "/opt/ros/jazzy/setup.bash"
    if not (os.path.isfile(ws_setup) and os.path.isfile(ros_setup)):
        return
    prefixes = [os.path.realpath(p) for p in
                os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    if ws_install in prefixes and os.environ.get("ROS_DOMAIN_ID"):
        return
    os.environ["NAV_TEST_ENV_BOOTSTRAPPED"] = "1"
    cmd = ("source {} && source {} && export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-98}} && "
           "exec python3 {} {}").format(
        shlex.quote(ros_setup), shlex.quote(ws_setup),
        shlex.quote(os.path.abspath(__file__)),
        " ".join(shlex.quote(a) for a in sys.argv[1:]))
    os.execvp("bash", ["bash", "-c", cmd])


if __name__ == "__main__":
    _bootstrap_ros_env()

import rclpy
import tf2_py as tf2
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_interfaces.msg import MissionEvent, MissionStatus, OperatorCommand, Waypoint
from nav_interfaces.srv import StartMission
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rtk_nav_control import RtkNavControl


STATUS_TOPIC = "/waypoint_manager/status"
MODE_TOPIC = "/navigation_test/mode"
EVENT_TOPIC = "/waypoint_manager/mission_feedback"
CLICKED_POINT_TOPIC = "/clicked_point"
# Dedicated PoseStamped topic for picking waypoints with orientation in Foxglove/RViz
# without immediately triggering single-goal navigation on /move_base_simple/goal.
WAYPOINT_PICK_TOPIC = "/waypoint_pick"
WAYPOINT_GOAL_TOPIC = "/waypoint/goal"
START_SERVICE = "/waypoint_manager/start_mission"
PAUSE_SERVICE = "/waypoint_manager/pause"
RESUME_SERVICE = "/waypoint_manager/resume"
CANCEL_SERVICE = "/waypoint_manager/cancel"
RETURN_HOME_SERVICE = "/waypoint_manager/return_home"
MISSION_RESET_SERVICE = "/waypoint_manager/reset"
BRIDGE_START_SERVICE = "/waypoint/goal/start"
BRIDGE_COMMIT_SERVICE = "/waypoint/goal/commit"
BRIDGE_RESET_SERVICE = "/waypoint/goal/reset"
BRIDGE_STATUS_TOPIC = "/waypoint/goal/status"
COMMAND_TOPIC = "/waypoint_manager/operator_command"
TERMINAL_STATES = {"COMPLETED", "FAILED", "CANCELLED"}


class NavControlTest:
    def __init__(self, node, map_frame, arrival_threshold, allow_motion, queue_mode):
        self.node = node
        self.map_frame = map_frame
        self.arrival_threshold = arrival_threshold
        self.allow_motion = allow_motion
        self.queue_mode = queue_mode
        self.condition = threading.Condition()
        self.latest_status = None
        self.bridge_state = None
        self.previous_status = None
        self.status_history = []
        self.event_history = []
        self.collection_active = False
        self.waiting_dwell = False
        self.pending_point = None
        # Yaw (radians) received from /waypoint_pick; None when point was received via /clicked_point.
        self.pending_yaw = None
        self.collected_waypoints = []
        self.collection_aborted = False
        self.single_point_mode = False
        self.direct_collection = False
        self.expected_dwells = {}
        self.arrivals = {}
        self.dwell_started = {}
        self.dwell_completed = {}
        self.current_waypoints = []
        self.current_loop = False
        self.execution_thread = None
        self.execution_active = False
        self._rtk_execution_id = None
        self.last_start_response = None
        self.last_start_attempted = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self._clients = {
            START_SERVICE: node.create_client(StartMission, START_SERVICE),
            PAUSE_SERVICE: node.create_client(Trigger, PAUSE_SERVICE),
            RESUME_SERVICE: node.create_client(Trigger, RESUME_SERVICE),
            CANCEL_SERVICE: node.create_client(Trigger, CANCEL_SERVICE),
            RETURN_HOME_SERVICE: node.create_client(Trigger, RETURN_HOME_SERVICE),
            MISSION_RESET_SERVICE: node.create_client(Trigger, MISSION_RESET_SERVICE),
            BRIDGE_START_SERVICE: node.create_client(Trigger, BRIDGE_START_SERVICE),
            BRIDGE_COMMIT_SERVICE: node.create_client(Trigger, BRIDGE_COMMIT_SERVICE),
            BRIDGE_RESET_SERVICE: node.create_client(Trigger, BRIDGE_RESET_SERVICE),
        }
        self._map_params_client = node.create_client(GetParameters, "/map_server/get_parameters")

        self.waypoint_goal_publisher = node.create_publisher(Waypoint, WAYPOINT_GOAL_TOPIC, 20)
        latched_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.mode_publisher = node.create_publisher(String, MODE_TOPIC, latched_qos)
        self.command_publisher = node.create_publisher(OperatorCommand, COMMAND_TOPIC, 50)
        self.rtk = RtkNavControl(node, self.tf_buffer, self.map_frame)
        node.create_subscription(MissionStatus, STATUS_TOPIC, self._status_callback, latched_qos)
        node.create_subscription(
            String,
            BRIDGE_STATUS_TOPIC,
            self._bridge_status_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        node.create_subscription(MissionEvent, EVENT_TOPIC, self._event_callback, 20)
        node.create_subscription(PointStamped, CLICKED_POINT_TOPIC, self._clicked_point_callback, 20)
        node.create_subscription(PoseStamped, WAYPOINT_PICK_TOPIC, self._waypoint_pick_callback, 20)

    def _status_callback(self, message):
        self.rtk.on_mission_status(message)
        state = message.state
        index = int(message.current_waypoint_idx)
        distance = float(message.distance_to_target_m)
        now = time.time()
        with self.condition:
            previous = self.previous_status
            self.latest_status = message
            if previous is None or previous.state != state or previous.current_waypoint_idx != index:
                self.status_history.append((now, state, index, distance))
                print("[STATUS] {} waypoint {}/{} dist {:.3f}m dwell_rem {:.1f}s".format(
                    state, index, message.total_waypoints, distance,
                    message.dwell_remaining_sec))

            if state in ("ARRIVED", "DWELLING") and index >= 0 and distance >= 0.0:
                if distance <= self.arrival_threshold:
                    self.arrivals[index] = distance
                if state == "DWELLING":
                    self.dwell_started.setdefault(index, now)

            if previous is not None and previous.state == "DWELLING":
                if state != "DWELLING" or index != previous.current_waypoint_idx:
                    previous_index = previous.current_waypoint_idx
                    started = self.dwell_started.get(previous_index)
                    if started is not None:
                        self.dwell_completed[previous_index] = now - started

            self.previous_status = message
            self.condition.notify_all()

    def _bridge_status_callback(self, message):
        with self.condition:
            self.bridge_state = message.data
            self.condition.notify_all()
        print("[QUEUE] {}".format(message.data))

    def _event_callback(self, message):
        with self.condition:
            self.event_history.append((time.time(), message.event, message.waypoint_idx, message.message))
            self.condition.notify_all()
        print("[EVENT] {} waypoint #{} {}".format(
            message.event, message.waypoint_idx, message.message))

    def _publish_command(self, command, phase, payload=None, success=False, message=""):
        command_message = OperatorCommand()
        command_message.header.stamp = self.node.get_clock().now().to_msg()
        command_message.command_id = "ui_{}".format(time.time_ns())
        command_message.source = "test_nav_control_ros2"
        command_message.command = command
        command_message.phase = phase
        command_message.payload_json = json.dumps(payload or {}, ensure_ascii=False, separators=(",", ":"))
        command_message.success = success
        command_message.message = message
        self.command_publisher.publish(command_message)

    def _clicked_point_callback(self, message):
        """Handle PointStamped from Foxglove/RViz Point tool (position only)."""
        try:
            point = self._point_in_map_frame(message)
        except Exception as exc:
            print("Failed to transform clicked point: {}".format(exc))
            return
        self._accept_pick(point, None, message.header.frame_id)

    def _waypoint_pick_callback(self, message):
        """Handle PoseStamped from Foxglove/RViz Pose tool (position + yaw)."""
        try:
            point, yaw = self._pose_in_map_frame(message)
        except Exception as exc:
            print("Failed to transform waypoint pose: {}".format(exc))
            return
        self._accept_pick(point, yaw, message.header.frame_id)

    def _accept_pick(self, point, yaw, frame_id):
        """Common handler for both waypoint pick topics. yaw is None if unspecified."""
        with self.condition:
            if not self.collection_active:
                return
            if self.single_point_mode:
                single_point_mode = True
            elif self.waiting_dwell:
                print("A clicked point is already waiting for dwell time; please enter dwell time first.")
                return
            else:
                single_point_mode = False

        self._publish_command(
            "WAYPOINT_CLICK", "ACCEPTED",
            {"frame_id": frame_id, "x": point[0], "y": point[1],
             "yaw": yaw, "single_point": single_point_mode},
            True,
        )
        with self.condition:
            if single_point_mode:
                self.collected_waypoints = [{
                    "x": point[0],
                    "y": point[1],
                    "yaw": yaw if yaw is not None else 0.0,
                    "dwell_sec": 0.0,
                    "name": "single_point_test",
                }]
                self.collection_active = False
                if self.queue_mode and not self.direct_collection:
                    self._publish_waypoint_goal(self.collected_waypoints[0])
                self.single_point_mode = False
                self.condition.notify_all()
            else:
                self.pending_point = point
                self.pending_yaw = yaw
                self.waiting_dwell = True

        if yaw is None:
            print("Received waypoint #{}, map x={:.3f}, y={:.3f} (yaw unspecified)".format(
                len(self.collected_waypoints) - 1, point[0], point[1]))
        else:
            print("Received waypoint #{}, map x={:.3f}, y={:.3f}, yaw {:.1f} deg".format(
                len(self.collected_waypoints) - 1, point[0], point[1],
                math.degrees(yaw)))

        if single_point_mode:
            print("Single-point navigation target recorded; starting execution (tagged as single_point_test).")
        elif yaw is None:
            print("Enter dwell time (seconds) for this waypoint; optionally append yaw in degrees "
                  "(e.g. \"2 90\" for 2s dwell and 90 deg yaw; defaults to 0 deg).")
        else:
            print("Enter dwell time (seconds) for this waypoint; yaw is set from arrow ({:.1f} deg), "
                  "or append an angle after a space to override.".format(math.degrees(yaw)))

    @staticmethod
    def _quat_rotate(quat, vector):
        ux, uy, uz, w = quat
        cross1 = (uy * vector[2] - uz * vector[1],
                  uz * vector[0] - ux * vector[2],
                  ux * vector[1] - uy * vector[0])
        cross2 = (uy * cross1[2] - uz * cross1[1],
                  uz * cross1[0] - ux * cross1[2],
                  ux * cross1[1] - uy * cross1[0])
        return (vector[0] + 2.0 * (w * cross1[0] + cross2[0]),
                vector[1] + 2.0 * (w * cross1[1] + cross2[1]),
                vector[2] + 2.0 * (w * cross1[2] + cross2[2]))

    @staticmethod
    def _quat_multiply(quat_a, quat_b):
        ax, ay, az, aw = quat_a
        bx, by, bz, bw = quat_b
        return (aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz)

    def _lookup_transform(self, source_frame):
        """Look up latest map<-source_frame transform, retrying briefly while TF warms up."""
        last_exc = None
        for _ in range(10):
            try:
                return self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time())
            except tf2.TransformException as exc:
                last_exc = exc
                time.sleep(0.2)
        raise last_exc

    def _point_in_map_frame(self, message):
        frame = message.header.frame_id or self.map_frame
        if frame == self.map_frame:
            return message.point.x, message.point.y
        transform = self._lookup_transform(frame).transform
        quat = (transform.rotation.x, transform.rotation.y,
                transform.rotation.z, transform.rotation.w)
        rotated = self._quat_rotate(quat, (message.point.x, message.point.y, message.point.z))
        return (rotated[0] + transform.translation.x,
                rotated[1] + transform.translation.y)

    def _pose_in_map_frame(self, message):
        """Return ((x, y), yaw) in map frame, with yaw in radians."""
        frame = message.header.frame_id or self.map_frame
        pose = message.pose
        if frame != self.map_frame:
            transform = self._lookup_transform(frame).transform
            quat = (transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w)
            rotated = self._quat_rotate(quat, (pose.position.x, pose.position.y, pose.position.z))
            pose_orientation = (pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w)
            orientation = self._quat_multiply(quat, pose_orientation)
            pose = ((rotated[0] + transform.translation.x,
                     rotated[1] + transform.translation.y), orientation)
        else:
            pose = ((pose.position.x, pose.position.y),
                    (pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w))
        ox, oy, oz, ow = pose[1]
        # Extract 2D yaw around Z axis.
        yaw = math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))
        return pose[0], yaw

    @staticmethod
    def _parse_dwell_and_yaw(line, default_yaw):
        """Parse '<dwell_sec> [yaw_deg]' input into (dwell_sec, yaw_rad)."""
        parts = line.split()
        if not parts or len(parts) > 2:
            raise ValueError("Format must be \"<dwell_sec>\" or \"<dwell_sec> <yaw_deg>\"")
        dwell = float(parts[0])
        if dwell < 0.0:
            raise ValueError("Dwell time cannot be negative")
        if len(parts) == 1:
            return dwell, (default_yaw if default_yaw is not None else 0.0)
        yaw_deg = float(parts[1])
        return dwell, math.radians(yaw_deg)

    def _publish_waypoint_goal(self, data):
        waypoint = Waypoint()
        waypoint.x = data["x"]
        waypoint.y = data["y"]
        waypoint.yaw = data.get("yaw", 0.0)
        waypoint.dwell_sec = data.get("dwell_sec", 0.0)
        waypoint.name = data.get("name", "waypoint")
        self.waypoint_goal_publisher.publish(waypoint)
        print("Published /waypoint/goal: x={:.3f}, y={:.3f}, yaw={:.3f}, dwell={:.1f}s".format(
            waypoint.x, waypoint.y, waypoint.yaw, waypoint.dwell_sec))

    def _wait_for_service(self, name):
        if self._clients[name].wait_for_service(timeout_sec=10.0):
            return True
        print("Service unavailable: {}".format(name))
        return False

    def _call_service(self, name, request, timeout_sec=15.0):
        """Call service synchronously; callbacks are driven by the background executor."""
        try:
            return self._clients[name].call(request, timeout_sec=timeout_sec)
        except Exception as exc:
            print("Service call failed {}: {}".format(name, exc))
            return None

    def _call_trigger(self, service, label):
        if not self._motion_ready() or not self._confirm("Confirm {}".format(label)):
            return
        if not self._wait_for_service(service):
            return
        response = self._call_service(service, Trigger.Request())
        if response is None:
            return
        print("{}: success={} message={}".format(label, response.success, response.message))
        if service == CANCEL_SERVICE and response.success:
            self._acknowledge_rtk_cancel()

    def _acknowledge_rtk_cancel(self):
        self.rtk.acknowledge_cancel()
        with self.condition:
            if self._rtk_execution_id:
                self._rtk_execution_id = None
                self.execution_active = False
            self.condition.notify_all()

    def _start_waypoint_capture(self):
        if not self.queue_mode:
            return True
        if not self._wait_for_service(BRIDGE_START_SERVICE):
            return False
        response = self._call_service(BRIDGE_START_SERVICE, Trigger.Request())
        if response is None:
            return False
        self._publish_command("QUEUE_START_CAPTURE", "ACCEPTED" if response.success else "REJECTED", {}, response.success, response.message)
        print("Queue capture started: success={} message={}".format(
            response.success, response.message))
        return response.success

    def _commit_waypoint_capture(self):
        if not self.queue_mode:
            return True
        if not self._wait_for_service(BRIDGE_COMMIT_SERVICE):
            return False
        response = self._call_service(BRIDGE_COMMIT_SERVICE, Trigger.Request())
        if response is None:
            return False
        self._publish_command("QUEUE_COMMIT", "ACCEPTED" if response.success else "REJECTED", {}, response.success, response.message)
        print("Queue execution committed: success={} message={}".format(
            response.success, response.message))
        return response.success

    def _reset_waypoint_queue(self):
        if not self.queue_mode:
            return
        if self._wait_for_service(BRIDGE_RESET_SERVICE):
            response = self._call_service(BRIDGE_RESET_SERVICE, Trigger.Request())
            if response is not None:
                self._publish_command("QUEUE_RESET", "ACCEPTED" if response.success else "REJECTED", {}, response.success, response.message)
                print("Waypoint queue reset: success={} message={}".format(
                    response.success, response.message))

    def _collection_input_loop(self):
        print("Waypoint collection started (two methods supported):")
        print("  A) Foxglove/RViz Pose tool (drag arrow) -> publishes to {} (position + yaw)"
              .format(WAYPOINT_PICK_TOPIC))
        print("     Set the 3D panel Publish Pose topic to {}"
              .format(WAYPOINT_PICK_TOPIC))
        print("  B) Point tool (click) -> publishes to {} (position only; enter yaw in terminal)"
              .format(CLICKED_POINT_TOPIC))
        print("After each pick, enter dwell seconds (optionally followed by yaw deg, e.g. \"2 90\"); "
              "type d + Enter when done, or q + Enter to abort.")
        while rclpy.ok():
            try:
                line = input("pick > ").strip()
            except (EOFError, KeyboardInterrupt):
                line = "q"
            self._publish_command("UI_INPUT", "RECEIVED", {"context": "waypoint_collection", "input": line}, True)

            with self.condition:
                waiting_dwell = self.waiting_dwell
                pending_point = self.pending_point
                pending_yaw = self.pending_yaw

            if waiting_dwell:
                if line.lower() == "q":
                    with self.condition:
                        self.collection_aborted = True
                        self.collection_active = False
                    return
                if line.lower() == "d":
                    print("Current clicked point still needs a dwell time before finishing.")
                    continue
                try:
                    dwell, yaw = self._parse_dwell_and_yaw(line, pending_yaw)
                except ValueError as exc:
                    print("Invalid input: {}".format(exc))
                    continue
                self._publish_command(
                    "WAYPOINT_DWELL", "ACCEPTED",
                    {"dwell_sec": dwell, "yaw": yaw}, True)
                data = {
                    "x": pending_point[0],
                    "y": pending_point[1],
                    "yaw": yaw,
                    "dwell_sec": dwell,
                    "name": "clicked_{}".format(len(self.collected_waypoints)),
                }
                with self.condition:
                    self.collected_waypoints.append(data)
                    self.pending_point = None
                    self.pending_yaw = None
                    self.waiting_dwell = False
                if self.queue_mode and not self.direct_collection:
                    self._publish_waypoint_goal(data)
                print("Recorded {} waypoint(s) (dwell {:.1f}s, yaw {:.1f} deg); pick next or type d + Enter.".format(
                    len(self.collected_waypoints), dwell, math.degrees(yaw)))
                continue

            if line.lower() == "q":
                with self.condition:
                    self.collection_aborted = True
                    self.collection_active = False
                    self.single_point_mode = False
                self._reset_waypoint_queue()
                return
            if line.lower() == "d":
                with self.condition:
                    if not self.collected_waypoints:
                        print("No waypoints recorded yet.")
                        continue
                    self.collection_active = False
                return
            print("Enter dwell seconds for the picked waypoint, d to finish, or q to abort.")

    def collect_single_point(self, temporary=False):
        self.mode_publisher.publish(String(data="single_point_test"))
        self.direct_collection = temporary
        if not temporary and not self._start_waypoint_capture():
            return [], False
        with self.condition:
            self.collection_active = True
            self.single_point_mode = True
            self.waiting_dwell = False
            self.pending_point = None
            self.pending_yaw = None
            self.collected_waypoints = []
            self.collection_aborted = False
        print("Single-point navigation mode active; pick one target in Foxglove/RViz: "
              "use Pose tool ({}) for position + yaw, or Point tool ({}) for yaw=0.".format(
                  WAYPOINT_PICK_TOPIC, CLICKED_POINT_TOPIC))
        with self.condition:
            while self.collection_active and not self.collection_aborted and rclpy.ok():
                self.condition.wait(timeout=0.5)
            result = list(self.collected_waypoints), not self.collection_aborted
        self.direct_collection = False
        return result

    def collect_clicked_waypoints(self):
        # Buffer multi-point picks locally and submit as a single mission once the user enters 'd'.
        self.direct_collection = True
        with self.condition:
            self.collection_active = True
            self.single_point_mode = False
            self.waiting_dwell = False
            self.pending_point = None
            self.pending_yaw = None
            self.collected_waypoints = []
            self.collection_aborted = False
        input_thread = threading.Thread(target=self._collection_input_loop)
        input_thread.daemon = True
        input_thread.start()
        input_thread.join()
        with self.condition:
            result = list(self.collected_waypoints), not self.collection_aborted
        self.direct_collection = False
        return result

    def _motion_ready(self):
        if not self.allow_motion:
            print("Read-only mode; pass --allow-motion to execute missions or control the robot.")
            return False
        return True

    def _confirm(self, prompt):
        result = input("{} (y/n): ".format(prompt)).strip().lower() == "y"
        self._publish_command("UI_CONFIRM", "ACCEPTED", {"prompt": prompt, "result": result}, result)
        return result

    def _get_map_name(self):
        """Query nav2 map_server for yaml_filename and return the map stem name, or None."""
        try:
            if not self._map_params_client.wait_for_service(timeout_sec=3.0):
                return None
            request = GetParameters.Request()
            request.names = ["yaml_filename"]
            response = self._map_params_client.call(request, timeout_sec=5.0)
        except Exception:
            return None
        if response is None or not response.values:
            return None
        value = response.values[0]
        if value.type == ParameterType.PARAMETER_STRING and value.string_value:
            return os.path.splitext(os.path.basename(value.string_value))[0]
        return None

    def _call_start_mission(self, waypoints, loop=False, temporary=False, preflight=None):
        self.last_start_attempted = False
        self.last_start_response = None
        if not self._wait_for_service(START_SERVICE):
            return False
        if preflight is not None:
            preflight()
        request = StartMission.Request()
        request.loop = bool(loop)
        request.temporary = bool(temporary)
        self.expected_dwells = {}
        self.arrivals = {}
        self.dwell_started = {}
        self.dwell_completed = {}
        request_waypoints = []
        for index, data in enumerate(waypoints):
            waypoint = Waypoint()
            waypoint.x = data["x"]
            waypoint.y = data["y"]
            waypoint.yaw = data.get("yaw", 0.0)
            waypoint.dwell_sec = data.get("dwell_sec", 0.0)
            waypoint.name = data.get("name", "waypoint_{}".format(index))
            request_waypoints.append(waypoint)
            self.expected_dwells[index] = waypoint.dwell_sec
        request.waypoints = request_waypoints
        self.last_start_attempted = True
        response = self._call_service(START_SERVICE, request)
        self.last_start_response = response
        if response is None:
            return False
        print("success={} message={} mission_id={}".format(
            response.success, response.message, response.mission_id))
        return response.success

    def _monitor_execution(self, queue_mode):
        with self.condition:
            while rclpy.ok():
                state = self.latest_status.state if self.latest_status else None
                bridge_done = queue_mode and self.bridge_state in TERMINAL_STATES
                if state in TERMINAL_STATES or bridge_done:
                    break
                self.condition.wait(timeout=0.5)
            self.execution_active = False
        self._print_execution_result()

    def execute_collected_waypoints(self, waypoints, loop=False, temporary=False, force_direct=False):
        if not self._motion_ready():
            return False
        self._rtk_execution_id = None
        self.current_waypoints = list(waypoints)
        self.current_loop = loop
        if self.queue_mode and not temporary and not force_direct:
            if not self._commit_waypoint_capture():
                self._reset_waypoint_queue()
                return False
            queue_mode = True
        else:
            if not self._call_start_mission(waypoints, loop, temporary):
                return False
            queue_mode = False
        self.execution_active = True
        self.execution_thread = threading.Thread(
            target=self._monitor_execution, args=(queue_mode,), daemon=True)
        self.execution_thread.start()
        return True

    def collect_rtk_goal(self):
        try:
            latitude = float(input("WGS84 latitude (decimal deg): ").strip())
            longitude = float(input("WGS84 longitude (decimal deg): ").strip())
            height = input("Altitude (m, empty=use datum): ").strip()
            altitude = float(height) if height else None
            yaw = float(input("Map yaw (deg, default 0): ").strip() or "0")
            dwell = float(input("Dwell time (sec, default 0): ").strip() or "0")
            preview = self.rtk.preview(latitude, longitude, altitude, yaw, dwell)
            print("RTK goal preview WGS84=({:.9f}, {:.9f}, {:.3f})".format(
                latitude, longitude, preview["altitude"]))
            print("  target {} = ({:.3f}, {:.3f}), yaw={:.1f}deg dwell={:.1f}s".format(
                self.map_frame, preview["x"], preview["y"], yaw, dwell))
            if not self.allow_motion:
                print("Read-only preview; restart with --allow-motion and confirm to dispatch.")
                return
            if not self._confirm("Dispatch this single RTK goal (non-temporary, will not replace an active mission)"):
                return
            with self.condition:
                if self.execution_active:
                    raise RuntimeError("A mission is currently active; cancel it first before dispatching")

            def preflight():
                self.rtk.validate_preview(preview, self.arrival_threshold)
                self.rtk.begin_submission(preview)

            waypoints = [{key: preview[key] for key in ("x", "y", "yaw", "dwell_sec", "name")}]
            accepted = self._call_start_mission(waypoints, preflight=preflight)
            if not self.last_start_attempted:
                return
            self.rtk.record_submission(preview, self.last_start_response)
            if not accepted or not self.rtk.mission_id:
                if self.last_start_response is None or accepted:
                    print("RTK mission acceptance unconfirmed; check status and use [5] Cancel / [8] Reset before retrying.")
                return
            with self.condition:
                self.current_waypoints = waypoints
                self.current_loop = False
                self._rtk_execution_id = self.rtk.mission_id
                self.execution_active = True
            self.execution_thread = threading.Thread(
                target=self._monitor_rtk_execution, args=(self.rtk.mission_id,), daemon=True)
            self.execution_thread.start()
        except (EOFError, KeyboardInterrupt):
            print("RTK goal input cancelled; nothing dispatched.")
        except (ValueError, RuntimeError) as exc:
            print(f"RTK goal not dispatched: {exc}")

    def _monitor_rtk_execution(self, mission_id):
        with self.condition:
            while rclpy.ok() and self._rtk_execution_id == mission_id:
                state = self.rtk.mission_state(mission_id)
                if state in TERMINAL_STATES:
                    self.execution_active = False
                    self._rtk_execution_id = None
                    print(f"[RTK] {mission_id} final state={state}")
                    return
                self.condition.wait(timeout=0.5)
            if self._rtk_execution_id == mission_id:
                self.execution_active = False
                self._rtk_execution_id = None

    def _print_waypoints(self, waypoints, loop=False):
        print("Waypoints: {}, loop: {}".format(len(waypoints), "yes" if loop else "no"))
        for index, waypoint in enumerate(waypoints):
            print("  #{} {}: x={:.3f} y={:.3f} yaw={:.3f} dwell={:.1f}s".format(
                index, waypoint.get("name", "waypoint"), float(waypoint["x"]),
                float(waypoint["y"]), float(waypoint.get("yaw", 0.0)),
                float(waypoint.get("dwell_sec", 0.0))))

    def save_waypoints(self):
        waypoints = list(self.current_waypoints or self.collected_waypoints)
        if not waypoints:
            print("No waypoints available to save")
            return
        default_path = os.path.expanduser(
            "~/.go2_waypoints/waypoints_{}.json".format(time.strftime("%Y%m%d_%H%M%S")))
        path = input("Save file [{}]: ".format(default_path)).strip() or default_path
        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        map_name = self._get_map_name()
        payload = {"version": 1, "map_frame": self.map_frame, "map_name": map_name,
                   "loop": self.current_loop, "waypoints": waypoints}
        with open(path, "w") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
        print("Saved {} waypoint(s) to {}".format(len(waypoints), path))

    def collect_manual_waypoints(self):
        waypoints = []
        print("Enter waypoints manually; leave name empty to finish. Yaw is in degrees.")
        while True:
            name = input("Waypoint name (empty=done): ").strip()
            if not name:
                break
            try:
                x = float(input("x (m): ").strip())
                y = float(input("y (m): ").strip())
                yaw = math.radians(float(input("yaw (degrees): ").strip()))
                dwell = float(input("dwell (seconds): ").strip())
                if dwell < 0.0:
                    raise ValueError("dwell cannot be negative")
            except ValueError as exc:
                print("Invalid waypoint input: {}".format(exc))
                continue
            waypoints.append({"x": x, "y": y, "yaw": round(yaw, 4),
                              "dwell_sec": dwell, "name": name})
        if not waypoints:
            print("No waypoints entered")
            return
        loop = input("Loop mission? (y/n): ").strip().lower() == "y"
        self.current_waypoints = waypoints
        self.current_loop = loop
        self._print_waypoints(waypoints, loop)
        if self._confirm("Execute entered waypoints now"):
            self.execute_collected_waypoints(waypoints, loop, force_direct=True)

    def load_waypoints_and_run(self):
        if not self._motion_ready():
            return
        saved_dir = os.path.expanduser("~/.go2_waypoints")
        saved_files = sorted(
            glob.glob(os.path.join(saved_dir, "*.json")),
            key=os.path.getmtime, reverse=True)
        if saved_files:
            print("Saved waypoint files:")
            for idx, saved_path in enumerate(saved_files, start=1):
                mtime = time.strftime(
                    "%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(saved_path)))
                print("  [{}] {} ({})".format(idx, os.path.basename(saved_path), mtime))
        path = input("Load waypoint JSON file (index or path): ").strip()
        if not path:
            return
        if path.isdigit():
            choice_idx = int(path)
            if 1 <= choice_idx <= len(saved_files):
                path = saved_files[choice_idx - 1]
            else:
                print("Invalid index: {}".format(path))
                return
        path = os.path.expanduser(path)
        if not os.path.isabs(path):
            candidates = [
                os.path.abspath(path),
                os.path.join(os.path.expanduser("~"), path),
                os.path.join(os.path.expanduser("~/.go2_waypoints"), path),
            ]
            path = next((candidate for candidate in candidates
                         if os.path.isfile(candidate)), candidates[-1])
        try:
            with open(path, "r") as stream:
                payload = json.load(stream)
            waypoints = payload["waypoints"]
            loop = bool(payload.get("loop", False))
            if not waypoints:
                raise ValueError("waypoints list is empty")
            for waypoint in waypoints:
                float(waypoint["x"])
                float(waypoint["y"])
                waypoint.setdefault("yaw", 0.0)
                waypoint.setdefault("dwell_sec", 0.0)
                waypoint.setdefault("name", "waypoint")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            print("Invalid waypoint file: {}".format(exc))
            return
        saved_map_name = payload.get("map_name")
        current_map_name = self._get_map_name()
        if saved_map_name and current_map_name and saved_map_name != current_map_name:
            print("WARNING: Waypoint file was saved on map '{}', but current map is '{}'. "
                  "Coordinates may not match.".format(
                      saved_map_name, current_map_name))
            if not self._confirm("Continue loading anyway"):
                return
        elif saved_map_name is None:
            print("WARNING: Waypoint file does not record a map_name; cannot verify map compatibility.")
        self.current_waypoints = waypoints
        self.current_loop = loop
        self._print_waypoints(waypoints, loop)
        if self._confirm("Load and execute this waypoint file"):
            self.execute_collected_waypoints(waypoints, loop, force_direct=True)

    def _print_execution_result(self):
        with self.condition:
            status = self.latest_status
            arrivals = dict(self.arrivals)
            dwell_completed = dict(self.dwell_completed)
        print("\n--- Waypoint Execution Summary ---")
        print("Final state: {}".format(status.state if status else "NO_STATUS"))
        for index, expected in sorted(self.expected_dwells.items()):
            distance = arrivals.get(index)
            dwell = dwell_completed.get(index, 0.0)
            distance_ok = distance is not None and distance <= self.arrival_threshold
            dwell_ok = dwell + 0.75 >= expected
            print("Waypoint #{}: dist={} threshold={} dwell={:.1f}/{:.1f}s [{}]".format(
                index, "{:.3f}m".format(distance) if distance is not None else "N/A",
                self.arrival_threshold, dwell, expected,
                "PASS" if distance_ok and dwell_ok else "CHECK"))

    def _reset_all(self):
        if not self._motion_ready() or not self._confirm("Confirm navigation state reset"):
            return
        if self._wait_for_service(MISSION_RESET_SERVICE):
            response = self._call_service(MISSION_RESET_SERVICE, Trigger.Request())
            if response is not None:
                print("mission_manager reset: success={} message={}".format(
                    response.success, response.message))
                if response.success:
                    self._acknowledge_rtk_cancel()
        self._reset_waypoint_queue()
        with self.condition:
            self.collection_active = False
            self.waiting_dwell = False
            self.pending_point = None
            self.pending_yaw = None
            self.collected_waypoints = []
            self.collection_aborted = False
            self.single_point_mode = False
            self.current_waypoints = []
            self.current_loop = False

    def print_status(self):
        with self.condition:
            status = self.latest_status
        if status is None:
            print("No /waypoint_manager/status received yet")
        else:
            print("state={} waypoint={}/{} progress={:.1f}% distance={:.3f}m dwell={:.1f}s".format(
                status.state, status.current_waypoint_idx, status.total_waypoints,
                status.progress_percent, status.distance_to_target_m,
                status.dwell_remaining_sec))
        for line in self.rtk.status_lines():
            print(line)

    def run(self):
        print("ONS2 ROS2 multi-waypoint console. Pick topics: {} (Pose arrow with yaw)"
              " or {} (Point click, yaw=0)".format(
                  WAYPOINT_PICK_TOPIC, CLICKED_POINT_TOPIC))
        print("[1] Pick waypoints on map (enter dwell [yaw_deg] per point, 'd' to finish)")
        print("[2] Status  [3] Pause  [4] Resume  [5] Cancel  [6] Return home")
        print("[7] Single/temporary point (resumes main queue if paused)")
        print("[8] Reset (stop navigation and clear mission & /waypoint/goal queue)")
        print("[9] Save current waypoints JSON  [10] Load waypoints JSON & run")
        print("[11] Enter waypoints manually & run  [12/r] RTK lat/lon single goal  [0] Quit")
        while rclpy.ok():
            try:
                choice = input("Select > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            self._publish_command("UI_MENU", "RECEIVED", {"choice": choice}, True)
            if choice == "0":
                break
            if choice == "1":
                waypoints, completed = self.collect_clicked_waypoints()
                if completed and waypoints:
                    loop = self._confirm("Loop these waypoints")
                    self._print_waypoints(waypoints, loop)
                    self.execute_collected_waypoints(waypoints, loop, force_direct=True)
            elif choice == "2":
                self.print_status()
            elif choice == "3":
                self._call_trigger(PAUSE_SERVICE, "pause mission")
            elif choice == "4":
                self._call_trigger(RESUME_SERVICE, "resume mission")
            elif choice == "5":
                self._call_trigger(CANCEL_SERVICE, "cancel mission")
            elif choice == "6":
                self._call_trigger(RETURN_HOME_SERVICE, "return home")
            elif choice == "7":
                with self.condition:
                    paused = self.latest_status is not None and self.latest_status.state == "PAUSED"
                waypoints, completed = self.collect_single_point(temporary=paused)
                if completed and waypoints:
                    self.execute_collected_waypoints(waypoints, temporary=paused)
            elif choice == "8":
                self._reset_all()
            elif choice == "9":
                self.save_waypoints()
            elif choice == "10":
                self.load_waypoints_and_run()
            elif choice == "11":
                self.collect_manual_waypoints()
            elif choice.lower() in ("12", "r"):
                self.collect_rtk_goal()
            else:
                print("Invalid option")


class TerminalGui:
    """Curses-based terminal UI for interactive waypoint collection and mission control."""

    HELP_ROWS = (
        " [1] Pick waypoints (map) [2] Status          [3] Pause mission",
        " [4] Resume mission       [5] Cancel mission  [6] Return home",
        " [7] Single/temp point    [8] Reset nav       [9] Save waypoints JSON",
        " [0] Load JSON & run      [m] Manual input    [r] RTK lat/lon goal",
        " [q] Quit",
    )
    MENU_KEYS = "1234567890mr"

    def __init__(self, nav):
        self.nav = nav
        self._log_lines = deque(maxlen=400)
        self._log_lock = threading.Lock()
        self._partial = ""
        self._input_lock = threading.Lock()
        self._input_request = None
        self._idle = threading.Event()
        self._idle.set()
        self._closing = False
        self._original_stdout = None
        self._original_stderr = None
        self._original_input = None

    def _write_log(self, text):
        with self._log_lock:
            self._partial = (self._partial + text)[-4000:]
            while "\n" in self._partial:
                line, self._partial = self._partial.split("\n", 1)
                self._log_lines.append(line)

    def _log(self, text):
        self._write_log(text + "\n")

    def _install(self):
        import builtins

        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        self._original_input = builtins.input
        gui = self

        class _Writer:
            def __init__(self, prefix):
                self._prefix = prefix

            def write(self, text):
                gui._write_log(self._prefix + text if self._prefix else text)

            def flush(self):
                pass

        sys.stdout = _Writer("")
        sys.stderr = _Writer("[stderr] ")
        builtins.input = self._gui_input

    def _restore(self):
        import builtins

        if self._original_stdout is not None:
            sys.stdout = self._original_stdout
        if self._original_stderr is not None:
            sys.stderr = self._original_stderr
        if self._original_input is not None:
            builtins.input = self._original_input
        with self._log_lock:
            if self._partial:
                self._log_lines.append(self._partial)
                self._partial = ""

    def _gui_input(self, prompt=""):
        """Intercept input() and route it through the bottom input bar of the curses UI."""
        if self._closing:
            raise EOFError
        request = {
            "prompt": str(prompt),
            "buffer": "",
            "done": threading.Event(),
            "cancelled": False,
        }
        with self._input_lock:
            if self._input_request is not None:
                raise EOFError("Input request already pending")
            self._input_request = request
        while not request["done"].wait(timeout=0.2):
            if self._closing:
                with self._input_lock:
                    self._input_request = None
                raise EOFError
        if request["cancelled"]:
            raise EOFError
        return request["buffer"]

    def run(self):
        import curses

        try:
            return curses.wrapper(self._curses_main)
        except KeyboardInterrupt:
            return None

    def _curses_main(self, stdscr):
        import curses

        try:
            curses.curs_set(0)
        except curses.error:
            pass
        stdscr.keypad(True)
        stdscr.timeout(150)
        self._install()
        self._log("Terminal GUI started: press keys for actions (see controls below).")
        self._log("Pick topics: {} (Pose arrow with yaw) / {} (Point click, yaw=0)".format(
            WAYPOINT_PICK_TOPIC, CLICKED_POINT_TOPIC))
        self._log("Mode: {}.".format(
            "motion enabled (--allow-motion); RTK goals still require confirmation"
            if self.nav.allow_motion else "read-only (without --allow-motion)"))
        try:
            while rclpy.ok() and not self._closing:
                key = self._read_key(stdscr)
                request = self._input_request
                if request is not None:
                    self._handle_input_key(key, request)
                elif key is not None:
                    self._handle_menu_key(key)
                self._redraw(stdscr)
        finally:
            self._closing = True
            self._restore()

    def _read_key(self, stdscr):
        import curses

        try:
            key = stdscr.get_wch()
        except AttributeError:
            key = stdscr.getch()
        except curses.error:
            return None
        if key == curses.ERR or key == -1:
            return None
        return key

    def _handle_input_key(self, key, request):
        import curses

        if key is None:
            return
        if key in (curses.KEY_BACKSPACE, 8, 127):
            request["buffer"] = request["buffer"][:-1]
        elif key in (10, 13, "\n", "\r"):
            self._finish_input(request, False)
        elif key in (27, "\x1b", "\x03"):
            self._finish_input(request, True)
        elif isinstance(key, str) and key.isprintable():
            request["buffer"] += key

    def _finish_input(self, request, cancelled):
        request["cancelled"] = cancelled
        with self._input_lock:
            self._input_request = None
        request["done"].set()

    def _handle_menu_key(self, key):
        if isinstance(key, int):
            return
        text = str(key).lower()
        if text in ("q", "\x03"):
            self._closing = True
            self._log("Exit requested...")
            return
        if text not in self.MENU_KEYS:
            return
        if not self._idle.is_set():
            self._log("[GUI] Previous action is still active; please complete it first.")
            return
        self._idle.clear()
        threading.Thread(target=self._guard_action, args=(text,), daemon=True).start()

    def _guard_action(self, key):
        try:
            self.nav._publish_command(
                "UI_MENU", "RECEIVED", {"choice": key, "source": "gui"}, True)
            self._run_action(key)
        except Exception as exc:
            self._log("[GUI] Action error: {}".format(exc))
        finally:
            self._idle.set()

    def _run_action(self, key):
        """Dispatch menu actions matching NavControlTest.run()."""
        nav = self.nav
        if key == "1":
            waypoints, completed = nav.collect_clicked_waypoints()
            if completed and waypoints:
                loop = nav._confirm("Loop these waypoints")
                nav._print_waypoints(waypoints, loop)
                nav.execute_collected_waypoints(waypoints, loop, force_direct=True)
        elif key == "2":
            nav.print_status()
        elif key == "3":
            nav._call_trigger(PAUSE_SERVICE, "pause mission")
        elif key == "4":
            nav._call_trigger(RESUME_SERVICE, "resume mission")
        elif key == "5":
            nav._call_trigger(CANCEL_SERVICE, "cancel mission")
        elif key == "6":
            nav._call_trigger(RETURN_HOME_SERVICE, "return home")
        elif key == "7":
            with nav.condition:
                paused = nav.latest_status is not None and nav.latest_status.state == "PAUSED"
            waypoints, completed = nav.collect_single_point(temporary=paused)
            if completed and waypoints:
                nav.execute_collected_waypoints(waypoints, temporary=paused)
        elif key == "8":
            nav._reset_all()
        elif key == "9":
            nav.save_waypoints()
        elif key == "0":
            nav.load_waypoints_and_run()
        elif key == "m":
            nav.collect_manual_waypoints()
        elif key == "r":
            nav.collect_rtk_goal()

    def _redraw(self, stdscr):
        import curses

        height, width = stdscr.getmaxyx()
        if height < 15 or width < 40:
            return
        stdscr.erase()
        nav = self.nav
        with nav.condition:
            status = nav.latest_status
            bridge_state = nav.bridge_state
            collecting = nav.collection_active
            point_count = len(nav.collected_waypoints)
            executing = nav.execution_active
        if status is not None:
            line1 = "State {} wp {}/{} dist {:.2f}m dwell {:.1f}s progress {:.0f}%".format(
                status.state, status.current_waypoint_idx, status.total_waypoints,
                status.distance_to_target_m, status.dwell_remaining_sec,
                status.progress_percent)
        else:
            line1 = "State: waiting for /waypoint_manager/status"
        line2 = "Queue {} | Pick {} | Collected {} | {} | {}".format(
            bridge_state or "-", "active" if collecting else "idle", point_count,
            "monitoring" if executing else "standby",
            "motion enabled" if nav.allow_motion else "read-only")
        headers = [line1, line2, *nav.rtk.status_lines()]
        log_top = len(headers) + 1
        log_rows = max(height - log_top - len(self.HELP_ROWS) - 2, 1)
        for row, text in enumerate(headers):
            try:
                stdscr.addstr(row, 0, text[:width - 1], curses.A_REVERSE)
            except curses.error:
                pass
        try:
            stdscr.addstr(len(headers), 0, "-" * (width - 1), curses.A_DIM)
        except curses.error:
            pass
        with self._log_lock:
            lines = list(self._log_lines)
        for offset, line in enumerate(lines[-log_rows:]):
            try:
                stdscr.addstr(log_top + offset, 0, line[:width - 1])
            except curses.error:
                pass
        help_top = log_top + log_rows
        try:
            stdscr.addstr(help_top, 0, "Controls:", curses.A_BOLD)
        except curses.error:
            pass
        for index, row_text in enumerate(self.HELP_ROWS):
            try:
                stdscr.addstr(help_top + 1 + index, 0, row_text[:width - 1], curses.A_BOLD)
            except curses.error:
                pass
        request = self._input_request
        if request is not None:
            shown = request["prompt"] + request["buffer"]
            if len(shown) > width - 2:
                shown = "..." + shown[-(width - 3):]
            try:
                stdscr.addstr(height - 1, 0, shown, curses.A_REVERSE)
            except curses.error:
                pass
        else:
            try:
                stdscr.addstr(
                    height - 1, 0,
                    "Press key to select action (see Controls above); 'q' to quit"[:width - 1],
                    curses.A_DIM)
            except curses.error:
                pass
        stdscr.refresh()


def _shutdown_operator(node, executor, spin_thread, nav):
    with nav.condition:
        nav._rtk_execution_id = None
        nav.condition.notify_all()
    # Join ROS workers before destroying their C++ objects; quitting the UI does not cancel a task.
    executor.shutdown()
    spin_thread.join(timeout=3.0)
    nav.tf_listener.unregister()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if nav.execution_thread is not None:
        nav.execution_thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(
        description="ONS2 ROS2 interactive Foxglove/RViz & RTK waypoint mission console"
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--arrival-threshold", type=float, default=0.15)
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument(
        "--queue-mode",
        action="store_true",
        help="Publish picked waypoints to /waypoint/goal for sequential dispatch via waypoint_goal_bridge",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable curses TUI and fall back to plain CLI prompts",
    )
    args = parser.parse_args()
    rclpy.init(args=sys.argv[:1])
    node = rclpy.create_node("test_nav_control")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    nav = NavControlTest(
        node,
        args.map_frame,
        args.arrival_threshold,
        args.allow_motion,
        args.queue_mode,
    )
    try:
        if args.no_gui:
            nav.run()
        else:
            try:
                import curses  # noqa: F401
            except ImportError:
                print("Standard library 'curses' unavailable; falling back to plain CLI mode.")
                nav.run()
            else:
                TerminalGui(nav).run()
                print("Terminal GUI exited.")
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_operator(node, executor, spin_thread, nav)


if __name__ == "__main__":
    main()
