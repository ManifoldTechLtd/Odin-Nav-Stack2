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

"""Gated cmd_vel relay for SRU local planner backend.

Forwards /sru/cmd_vel to /cmd_vel only when /sru/cmd_vel_enable is True,
yielding /cmd_vel control to atomic_move during waypoint alignment.
Defaults to closed (False) on startup for safety.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SRU_CMD_VEL_TOPIC = "/sru/cmd_vel"
SRU_ENABLE_TOPIC = "/sru/cmd_vel_enable"
OUTPUT_CMD_VEL_TOPIC = "/cmd_vel"


class SruCmdVelRelay(object):
    def __init__(self):
        self.enabled = False
        self.forwarded = 0
        self.suppressed = 0
        self.pub = rospy.Publisher(OUTPUT_CMD_VEL_TOPIC, Twist, queue_size=1)
        rospy.Subscriber(SRU_CMD_VEL_TOPIC, Twist, self._cmd_vel_callback, queue_size=1)
        rospy.Subscriber(SRU_ENABLE_TOPIC, Bool, self._enable_callback, queue_size=1)
        rospy.loginfo(
            "[sru_cmd_vel_relay] ready: %s -> %s (gate=%s, default closed)",
            SRU_CMD_VEL_TOPIC, OUTPUT_CMD_VEL_TOPIC, SRU_ENABLE_TOPIC)

    def _enable_callback(self, msg):
        enable = bool(msg.data)
        if enable != self.enabled:
            rospy.loginfo(
                "[sru_cmd_vel_relay] gate %s",
                "OPEN (forwarding SRU cmd_vel)" if enable
                else "CLOSED (hand over to atomic_move)")
        self.enabled = enable

    def _cmd_vel_callback(self, msg):
        if not self.enabled:
            self.suppressed += 1
            return
        self.forwarded += 1
        self.pub.publish(msg)
        rospy.loginfo_throttle(
            30.0, "[sru_cmd_vel_relay] forwarded=%d suppressed=%d enabled=%s",
            self.forwarded, self.suppressed, self.enabled)


if __name__ == "__main__":
    rospy.init_node("sru_cmd_vel_relay")
    SruCmdVelRelay()
    rospy.spin()
