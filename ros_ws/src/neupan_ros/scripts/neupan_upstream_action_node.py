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

# Relax Python GC thresholds (or disable/freeze via NEUPAN_GC_MODE) to avoid
# frequent GC collections during dense PoseStamped construction when running
# across different Python/C-extension environments.
import gc as _gc
import os as _os
_gc_mode = _os.environ.get("NEUPAN_GC_MODE", "threshold").strip().lower()
if _gc_mode == "off":
    _gc.disable()
elif _gc_mode == "freeze":
    _gc.freeze()
else:
    _gc.set_threshold(50000, 50, 50)

import importlib.util
import os
import sys
import traceback
from math import atan2

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty

from nav_interfaces.msg import (
    navAction,
    navFeedback,
    navPathAction,
    navPathFeedback,
    navPathResult,
    navResult,
)


def _load_upstream_neupan_core():
    candidates = []
    configured_path = os.environ.get("NEUPAN_UPSTREAM_ROS_FILE")
    if configured_path:
        candidates.append(configured_path)

    probe = os.path.abspath(os.path.dirname(__file__))
    for _ in range(7):
        probe = os.path.dirname(probe)
        if not probe or probe == "/":
            break
        candidates.append(os.path.join(probe, "NeuPAN", "neupan", "ros", "neupan_ros.py"))

    for candidate in candidates:
        if not candidate or not os.path.isfile(candidate):
            continue
        candidate = os.path.abspath(candidate)
        upstream_root = os.path.abspath(
            os.path.join(os.path.dirname(candidate), "../..")
        )
        if upstream_root not in sys.path:
            sys.path.insert(0, upstream_root)
        spec = importlib.util.spec_from_file_location(
            "neupan_upstream_ros_wrapper", candidate
        )
        if spec is None or spec.loader is None:
            continue
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rospy.loginfo("[neupan_action_compat] upstream wrapper: %s", candidate)
        return module.neupan_core
    raise RuntimeError(
        "Cannot locate upstream NeuPAN ROS wrapper. Set "
        "NEUPAN_UPSTREAM_ROS_FILE to the absolute neupan_ros.py path."
    )


UpstreamNeupanCore = _load_upstream_neupan_core()


class NeupanActionCompat(UpstreamNeupanCore):
    def __init__(self):
        super().__init__()

        self.goal = None
        self.arrive = False
        self._stop_sub = rospy.Subscriber(
            "/neupan/stop", Empty, self._stop_callback, queue_size=1
        )

        # Hot path update (enabled/disabled by mission_manager via latched /neupan/hot_path_enable):
        # when enabled, preempting a navPath goal replaces the reference path in place without
        # zeroing velocity or interrupting the control loop.
        self._hot_path_enabled = False
        self._hot_path_sub = rospy.Subscriber(
            "/neupan/hot_path_enable", Bool, self._hot_path_callback, queue_size=1
        )

        # Subscribe to latched /initial_path to repopulate waypoints if both
        # initial_path and waypoints were cleared by /neupan/stop while idle.
        try:
            from nav_msgs.msg import Path as _RosPath
            self._initial_path_sub = rospy.Subscriber(
                "/initial_path", _RosPath, self._on_initial_path, queue_size=1
            )
        except Exception as _exc:
            self._initial_path_sub = None
            rospy.logwarn("[neupan_action_compat] /initial_path subscription failed: %s", _exc)

        nav_action_topic = self.config.get("topic", {}).get(
            "nav_action", "/neupan/nav_goal"
        )
        nav_path_action_topic = self.config.get("topic", {}).get(
            "nav_path_action", "/neupan/nav_path"
        )
        self._goal_server = actionlib.SimpleActionServer(
            nav_action_topic,
            navAction,
            execute_cb=self._execute_goal,
            auto_start=False,
        )
        self._path_server = actionlib.SimpleActionServer(
            nav_path_action_topic,
            navPathAction,
            execute_cb=self._execute_path,
            auto_start=False,
        )
        self._goal_server.start()
        self._path_server.start()
        rospy.loginfo(
            "[neupan_action_compat] generic action servers ready: %s, %s",
            nav_action_topic,
            nav_path_action_topic,
        )

    def _hot_path_callback(self, msg):
        enable = bool(msg.data)
        if enable != self._hot_path_enabled:
            rospy.loginfo(
                "[neupan_action_compat] hot path update %s",
                "ENABLED" if enable else "DISABLED",
            )
        self._hot_path_enabled = enable

    def _stop_callback(self, _msg):
        self._clear_navigation("Stopped by user command")
        # Clear waypoints on explicit stop so run() does not rebuild the path
        # from stale waypoints and contend with atomic_move over /cmd_vel.
        self.neupan_planner.ipath.waypoints = []
        if self._goal_server.is_active():
            result = navResult()
            result.status = 2
            self._goal_server.set_preempted(result, "Stopped by user command")
        if self._path_server.is_active():
            result = navPathResult()
            result.status = 2
            self._path_server.set_preempted(result, "Stopped by user command")

    def _clear_navigation(self, reason):
        rospy.loginfo("[neupan_action_compat] clear navigation: %s", reason)
        self.neupan_planner.ipath.initial_path = None
        # Do not clear ipath.waypoints here: run() executes concurrently on the main
        # thread, and clearing waypoints during periodic_replan preemption would trigger
        # an assertion failure inside set_ipath_with_state if run() is mid-iteration.
        self.neupan_planner.reset(reset_velocity=True)
        self.goal = None
        self.arrive = False
        self.last_arrive_flag = False
        self.vel_pub.publish(Twist())

    def _handle_planner_exception(self, source, exc):
        rospy.logerr(
            "[neupan_action_compat] planner exception in %s: %s\n%s",
            source,
            exc,
            traceback.format_exc(),
        )
        try:
            self._clear_navigation("planner exception in {}".format(source))
        except Exception as clear_exc:
            rospy.logerr(
                "[neupan_action_compat] planner cleanup failed: %s\n%s",
                clear_exc,
                traceback.format_exc(),
            )
        if self._goal_server.is_active():
            result = navResult()
            result.status = 1
            self._goal_server.set_aborted(result, "NeuPAN planner exception")
        if self._path_server.is_active():
            result = navPathResult()
            result.status = 1
            self._path_server.set_aborted(result, "NeuPAN planner exception")

    def _path_to_initial_points(self, path):
        """Convert nav_msgs/Path into a list of 4x1 point arrays for NeuPAN."""
        initial_point_list = []
        for index, pose_stamped in enumerate(path.poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            if self.config["include_initial_path_direction"]:
                theta = self.quat_to_yaw(pose_stamped.pose.orientation)
            elif index + 1 < len(path.poses):
                next_pose = path.poses[index + 1].pose.position
                theta = atan2(next_pose.y - y, next_pose.x - x)
            elif initial_point_list:
                theta = initial_point_list[-1][2, 0]
            else:
                theta = 0.0
            initial_point_list.append(np.array([x, y, theta, 1]).reshape(4, 1))
        return initial_point_list

    def _on_initial_path(self, msg):
        """Restore waypoints from latched /initial_path when idle with empty initial_path and waypoints."""
        try:
            if not getattr(msg, "poses", None):
                return
            if self.neupan_planner.ipath.initial_path is not None:
                return
            if getattr(self.neupan_planner.ipath, "waypoints", None):
                return
            if self.goal is not None:
                return
            if self._path_server.is_active() or self._goal_server.is_active():
                return
            points = self._path_to_initial_points(msg)
            if not points:
                return
            self.neupan_planner.ipath.waypoints = [point[0:3].copy() for point in points]
            rospy.loginfo(
                "[neupan_action_compat] Restored waypoints from /initial_path (%d points)",
                len(points),
            )
        except Exception as exc:
            rospy.logwarn("[neupan_action_compat] Failed to restore waypoints: %s", exc)

    def _set_path_from_action(self, path, hot=False):
        if not path.poses:
            return False

        initial_point_list = []
        for index, pose_stamped in enumerate(path.poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            if self.config["include_initial_path_direction"]:
                theta = self.quat_to_yaw(pose_stamped.pose.orientation)
            elif index + 1 < len(path.poses):
                next_pose = path.poses[index + 1].pose.position
                theta = atan2(next_pose.y - y, next_pose.x - x)
            elif initial_point_list:
                theta = initial_point_list[-1][2, 0]
            else:
                theta = 0.0
            initial_point_list.append(np.array([x, y, theta, 1]).reshape(4, 1))

        self.neupan_planner.set_initial_path(initial_point_list)
        self.neupan_planner.ipath.waypoints = [point[0:3].copy() for point in initial_point_list]
        # Maintain map-frame path copy for straight-line angular deadband and cross-track gates.
        self._global_path_xy = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        )
        if getattr(self, "_curv", None) is not None:
            self._curv.set_path(self._global_path_xy)
        if hot:
            self.neupan_planner.reset()
        else:
            self.neupan_planner.reset(reset_velocity=True)
        last_pose = path.poses[-1].pose
        self.goal = np.array(
            [[last_pose.position.x], [last_pose.position.y], [0.0]], dtype=float
        )
        self.arrive = False
        self.last_arrive_flag = False
        if hasattr(self, "_note_recovery_path_installed"):
            self._note_recovery_path_installed(path.header.stamp)
        rospy.loginfo(
            "[neupan_action_compat] accepted generic path: poses=%d end=(%.2f, %.2f)",
            len(path.poses),
            last_pose.position.x,
            last_pose.position.y,
        )
        return True

    def _execute_goal(self, action_goal):
        result = navResult()
        try:
            self._clear_navigation("new goal accepted")
            self.goal_callback(action_goal.target_pose)
            feedback = navFeedback()
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                if not self._goal_server.is_active():
                    return
                if self._goal_server.is_preempt_requested():
                    self._clear_navigation("goal action preempted")
                    self._goal_server.set_preempted(result, "Goal action preempted")
                    return
                if self.arrive:
                    result.status = 0
                    self._goal_server.set_succeeded(result, "Goal reached")
                    return
                self._publish_goal_feedback(self._goal_server, feedback)
                rate.sleep()
        except Exception as exc:
            self._handle_planner_exception("goal action", exc)

    def _execute_path(self, action_goal):
        result = navPathResult()
        try:
            if not self._set_path_from_action(action_goal.path):
                result.status = 1
                self._path_server.set_aborted(result, "Received an empty path")
                return

            feedback = navPathFeedback()
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                if not self._path_server.is_active():
                    return
                if self._path_server.is_preempt_requested():
                    if (
                        self._hot_path_enabled
                        and self._path_server.is_new_goal_available()
                    ):
                        new_goal = self._path_server.accept_new_goal()
                        rospy.loginfo(
                            "[neupan_action_compat] hot path update: poses=%d",
                            len(new_goal.path.poses),
                        )
                        if not self._set_path_from_action(new_goal.path, hot=True):
                            result.status = 1
                            self._path_server.set_aborted(
                                result, "Received an empty path"
                            )
                            return
                        continue
                    self._clear_navigation("path action preempted")
                    self._path_server.set_preempted(result, "Path action preempted")
                    return
                if self.arrive:
                    result.status = 0
                    self._path_server.set_succeeded(result, "Path reached")
                    return
                self._publish_goal_feedback(self._path_server, feedback)
                ticks = getattr(self, "_path_loop_ticks", 0) + 1
                self._path_loop_ticks = ticks
                if ticks % 50 == 0:
                    rospy.loginfo(
                        "[neupan_action_compat] path execute loop alive, ticks=%d",
                        ticks,
                    )
                rate.sleep()
        except Exception as exc:
            self._handle_planner_exception("path action", exc)

    def _publish_goal_feedback(self, server, feedback):
        if self.robot_state is None or self.goal is None:
            now = rospy.Time.now()
            if (
                not hasattr(self, "_fb_skip_warn_at")
                or (now - self._fb_skip_warn_at).toSec() > 5.0
            ):
                self._fb_skip_warn_at = now
                rospy.logwarn(
                    "[neupan_action_compat] feedback SKIPPED (watchdog will trip): "
                    "robot_state=%s goal=%s",
                    "None" if self.robot_state is None else "ok",
                    "None" if self.goal is None else "ok",
                )
            return
        dx = self.goal[0, 0] - self.robot_state[0, 0]
        dy = self.goal[1, 0] - self.robot_state[1, 0]
        feedback.distance_remaining = float(np.hypot(dx, dy))
        server.publish_feedback(feedback)


if __name__ == "__main__":
    try:
        NeupanActionCompat().run()
    except rospy.ROSInterruptException:
        pass
