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

import copy
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_interfaces.msg import MissionStatus, Waypoint
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


WAYPOINT_GOAL_TOPIC = "/waypoint/goal"
SINGLE_GOAL_TOPIC = "/move_base_simple/goal"
STATUS_TOPIC = "/waypoint_manager/status"
START_CAPTURE_SERVICE = "/waypoint/goal/start"
COMMIT_SERVICE = "/waypoint/goal/commit"
RESET_SERVICE = "/waypoint/goal/reset"
BRIDGE_STATUS_TOPIC = "/waypoint/goal/status"
ACTIVE_STATES = {"NAVIGATING", "ARRIVED", "ALIGNING", "DWELLING", "PAUSED", "RETURNING_HOME"}
TERMINAL_STATES = {"IDLE", "COMPLETED", "CANCELLED", "FAILED"}


class WaypointGoalBridge:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.lock = threading.RLock()
        self.queue = []
        self.capture_active = False
        self.executing = False
        self.advancing = False
        self.current_index = -1
        self.active_mission_id = ""
        self.latest_status = None

        self.goal_pub = rospy.Publisher(SINGLE_GOAL_TOPIC, PoseStamped, queue_size=1)
        self.bridge_status_pub = rospy.Publisher(
            BRIDGE_STATUS_TOPIC, String, queue_size=1, latch=True
        )
        rospy.Subscriber(WAYPOINT_GOAL_TOPIC, Waypoint, self._waypoint_callback, queue_size=20)
        rospy.Subscriber(STATUS_TOPIC, MissionStatus, self._status_callback, queue_size=10)
        rospy.Service(START_CAPTURE_SERVICE, Trigger, self._start_capture)
        rospy.Service(COMMIT_SERVICE, Trigger, self._commit)
        rospy.Service(RESET_SERVICE, Trigger, self._reset)
        rospy.loginfo(
            "[waypoint_goal_bridge] ready: input=%s output=%s",
            WAYPOINT_GOAL_TOPIC,
            SINGLE_GOAL_TOPIC,
        )

    def _start_capture(self, _request):
        with self.lock:
            if self.executing:
                return TriggerResponse(False, "Cannot start capture while executing")
            self.queue = []
            self.capture_active = True
            self.current_index = -1
            self.active_mission_id = ""
        self._publish_bridge_status("CAPTURING")
        rospy.loginfo("[waypoint_goal_bridge] capture started")
        return TriggerResponse(True, "Waypoint capture started")

    def _waypoint_callback(self, message):
        with self.lock:
            if not self.capture_active or self.executing:
                rospy.logwarn_throttle(
                    2.0,
                    "[waypoint_goal_bridge] ignoring waypoint outside capture phase",
                )
                return
            waypoint = copy.deepcopy(message)
            if waypoint.name == "":
                waypoint.name = "waypoint_{}".format(len(self.queue))
            if waypoint.dwell_sec < 0.0:
                rospy.logwarn("[waypoint_goal_bridge] ignoring negative dwell waypoint")
                return
            self.queue.append(waypoint)
            count = len(self.queue)
        rospy.loginfo(
            "[waypoint_goal_bridge] captured #%d x=%.3f y=%.3f yaw=%.3f dwell=%.3f",
            count - 1,
            waypoint.x,
            waypoint.y,
            waypoint.yaw,
            waypoint.dwell_sec,
        )

    def _commit(self, _request):
        with self.lock:
            if self.executing:
                return TriggerResponse(False, "Waypoint queue is already executing")
            if not self.queue:
                return TriggerResponse(False, "Waypoint queue is empty")
            if self.latest_status and self.latest_status.state in ACTIVE_STATES:
                return TriggerResponse(
                    False,
                    "Mission manager is active in {}".format(self.latest_status.state),
                )
            self.capture_active = False
            self.executing = True
            self.advancing = False
            self.current_index = 0
            self.active_mission_id = ""
        self._publish_bridge_status("EXECUTING")
        self._publish_current_goal()
        return TriggerResponse(True, "Waypoint queue execution started")

    def _reset(self, _request):
        with self.lock:
            self.queue = []
            self.capture_active = False
            self.executing = False
            self.advancing = False
            self.current_index = -1
            self.active_mission_id = ""
        self._publish_bridge_status("IDLE")
        rospy.loginfo("[waypoint_goal_bridge] queue reset")
        return TriggerResponse(True, "Waypoint queue reset")

    def _status_callback(self, message):
        with self.lock:
            self.latest_status = message
            if not self.executing:
                return
            if not self.active_mission_id:
                if message.state == "NAVIGATING":
                    self.active_mission_id = message.mission_id
                    rospy.loginfo(
                        "[waypoint_goal_bridge] waypoint #%d mission started: %s",
                        self.current_index,
                        self.active_mission_id,
                    )
                return
            if message.mission_id != self.active_mission_id:
                return
            if message.state == "FAILED":
                self.executing = False
                self._publish_bridge_status("FAILED")
                rospy.logerr(
                    "[waypoint_goal_bridge] waypoint #%d failed: %s",
                    self.current_index,
                    message.mission_id,
                )
                return
            if message.state == "CANCELLED":
                self.executing = False
                self._publish_bridge_status("CANCELLED")
                rospy.logwarn(
                    "[waypoint_goal_bridge] waypoint #%d cancelled",
                    self.current_index,
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
            rospy.loginfo(
                "[waypoint_goal_bridge] waypoint #%d completed; dwelling %.3f s",
                index,
                dwell,
            )
            rospy.sleep(dwell)
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
                rospy.loginfo("[waypoint_goal_bridge] waypoint queue completed")
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
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        message.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint.yaw)
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(message)
        rospy.loginfo(
            "[waypoint_goal_bridge] published waypoint #%d to %s x=%.3f y=%.3f yaw=%.3f",
            self.current_index,
            SINGLE_GOAL_TOPIC,
            waypoint.x,
            waypoint.y,
            waypoint.yaw,
        )


def main():
    rospy.init_node("waypoint_goal_bridge")
    WaypointGoalBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
