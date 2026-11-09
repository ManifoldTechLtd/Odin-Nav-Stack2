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

"""NeuPAN ROS 2 action server node wrapping the upstream NeuPAN core via ros2_shim.

Concurrency model:
  - Replaces _default_callback_group with ReentrantCallbackGroup so ActionServer
    internals, /neupan/stop, and /neupan/hot_path_enable run concurrently under
    MultiThreadedExecutor while an action execution loop is active.
"""

import importlib.util
import os
import sys
import threading
import time
import traceback
from math import atan2

import numpy as np
import rclpy
import rclpy.node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty


def _load_upstream_neupan_core():
    """Locate and load the upstream NeuPAN ROS core wrapper."""
    candidates = []
    configured_path = os.environ.get("NEUPAN_UPSTREAM_ROS_FILE")
    if configured_path:
        candidates.append(configured_path)

    here = os.path.abspath(os.path.dirname(__file__))
    probe = here
    for _ in range(7):
        probe = os.path.dirname(probe)
        if not probe or probe == "/":
            break
        candidates.append(
            os.path.join(probe, "NeuPAN", "neupan", "ros", "neupan_ros.py")
        )

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
        print("[neupan_action_compat] upstream wrapper: %s" % candidate, flush=True)
        return module.neupan_core
    raise RuntimeError(
        "Cannot locate upstream NeuPAN ROS wrapper. Set "
        "NEUPAN_UPSTREAM_ROS_FILE to the absolute neupan_ros.py path."
    )


UpstreamNeupanCore = _load_upstream_neupan_core()

try:
    from nav_interfaces.action import Nav as NavAction
    from nav_interfaces.action import NavPath as NavPathAction
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Failed to import nav_interfaces.action (build nav_interfaces in ros2_ws and source install/setup.bash first): %s"
        % exc
    )


class NeupanActionCompat(UpstreamNeupanCore, rclpy.node.Node):
    """Upstream NeuPAN core combined with rclpy action servers."""

    def __init__(self):
        rclpy.node.Node.__init__(self, "neupan_node")
        self._default_callback_group = ReentrantCallbackGroup()
        from neupan.ros import ros2_shim

        self._shim = ros2_shim
        ros2_shim.bind_node(self)
        self._lock = threading.RLock()
        self._control_enabled = False
        self._control_generation = 0
        self._goal_preempt = threading.Event()
        self._path_preempt = threading.Event()
        self._planner_abort_reason = None
        self._active_goal_handle = None
        self._pending_goal_handle = None
        self._active_path_handle = None
        self._pending_path_handle = None
        self._preinstalled_path_handle = None

        # Initialize the gate before core subscriptions can invoke callbacks.
        UpstreamNeupanCore.__init__(self)

        self.goal = None
        self.arrive = False

        self._stop_sub = self.create_subscription(
            Empty, "/neupan/stop", self._stop_callback, 1
        )

        # Hot path update mode: when enabled, a new navPath goal replaces the reference
        # path in place without zeroing velocity or interrupting the control loop.
        self._configure_hot_path()

        nav_action_topic = self.config.get("topic", {}).get(
            "nav_action", "/neupan/nav_goal"
        )
        nav_path_action_topic = self.config.get("topic", {}).get(
            "nav_path_action", "/neupan/nav_path"
        )
        self._goal_server = ActionServer(
            self,
            NavAction,
            nav_action_topic,
            execute_callback=self._execute_goal,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            handle_accepted_callback=lambda gh: self._handle_accepted(gh, "goal"),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )
        self._path_server = ActionServer(
            self,
            NavPathAction,
            nav_path_action_topic,
            execute_callback=self._execute_path,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            handle_accepted_callback=lambda gh: self._handle_accepted(gh, "path"),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            "[neupan_action_compat] generic action servers ready: %s, %s"
            % (nav_action_topic, nav_path_action_topic)
        )

    def _handle_accepted(self, goal_handle, kind):
        with self._lock:
            if kind == "goal":
                if self._active_goal_handle is None:
                    self._active_goal_handle = goal_handle
                    goal_handle.execute()
                else:
                    self._pending_goal_handle = goal_handle
                    self._goal_preempt.set()
            else:
                if self._active_path_handle is None:
                    self._active_path_handle = goal_handle
                    goal_handle.execute()
                else:
                    self._pending_path_handle = goal_handle
                    self._path_preempt.set()

    def _configure_hot_path(self):
        self._hot_path_enabled = self._navigation_mode == "replan_periodic"
        self._hot_path_sub = self.create_subscription(
            Bool, "/neupan/hot_path_enable", self._hot_path_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.get_logger().info(
            "[neupan_action_compat] hot path initial=%s mode=%s; retained state subscribed"
            % (self._hot_path_enabled, self._navigation_mode)
        )

    def _hot_path_callback(self, msg):
        with self._lock:
            enable = bool(msg.data)
            if enable != self._hot_path_enabled:
                self.get_logger().info(
                    "[neupan_action_compat] hot path update %s"
                    % ("ENABLED" if enable else "DISABLED")
                )
            self._hot_path_enabled = enable

    def _stop_callback(self, _msg):
        with self._lock:
            self._clear_navigation("Stopped by user command")
            self._discard_pending_goals(status=2)
            if self._active_goal_handle is not None:
                self._goal_preempt.set()
            if self._active_path_handle is not None:
                self._path_preempt.set()

    def _discard_pending_goals(self, status):
        pending = (
            (self._pending_goal_handle, NavAction),
            (self._pending_path_handle, NavPathAction),
        )
        self._pending_goal_handle = None
        self._pending_path_handle = None
        for handle, action_type in pending:
            if handle is not None and handle.is_active:
                result = action_type.Result()
                result.status = status
                # ACCEPTED -> ABORTED is illegal in ROS2. Enter execution with
                # a disposal-only callback, never the navigation callback.
                handle.execute(
                    lambda discarded, result=result:
                    self._finish_discarded_goal(discarded, result))

    def _finish_discarded_goal(self, handle, result):
        if handle.is_active:
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        return result

    def _clear_navigation(self, reason):
        self.get_logger().info("[neupan_action_compat] clear navigation: %s" % reason)
        with self._lock:
            self._control_enabled = False
            self._control_generation += 1
            self._preinstalled_path_handle = None
            # Retain waypoints for an in-flight solve, but never regenerate them
            # while stopped. Only an explicit new path/goal reopens the gate.
            self.neupan_planner.reset(reset_velocity=True, clear_path=True)
            self.goal = None
            self.arrive = False
            self.last_arrive_flag = False
            self.vel_pub.publish(Twist())

    def _control_cycle_token(self):
        with self._lock:
            if not self._control_enabled:
                return None
            for handle in (self._active_goal_handle, self._active_path_handle):
                if handle is not None and handle.is_cancel_requested:
                    return None
            return self._control_generation

    def _publish_control_velocity(self, action, token):
        with self._lock:
            if token is None or token != self._control_cycle_token():
                return False
            self.vel_pub.publish(self.generate_twist_msg(action))
            return True

    def _on_navigation_path_installed(self):
        with self._lock:
            self._control_generation += 1
            self._control_enabled = True

    def path_callback(self, path):
        with self._lock:
            return super().path_callback(path)

    def waypoints_callback(self, path):
        with self._lock:
            return super().waypoints_callback(path)

    def goal_callback(self, goal):
        with self._lock:
            return super().goal_callback(goal)

    def _handle_planner_exception(self, source, exc):
        self.get_logger().error(
            "[neupan_action_compat] planner exception in %s: %s\n%s"
            % (source, exc, traceback.format_exc())
        )
        with self._lock:
            try:
                self._clear_navigation("planner exception in {}".format(source))
            except Exception as clear_exc:
                self.get_logger().error(
                    "[neupan_action_compat] planner cleanup failed: %s\n%s"
                    % (clear_exc, traceback.format_exc())
                )
            self._planner_abort_reason = "NeuPAN planner exception"
            self._discard_pending_goals(status=1)
            if self._active_goal_handle is not None:
                self._goal_preempt.set()
            if self._active_path_handle is not None:
                self._path_preempt.set()

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
        self._on_navigation_path_installed()
        if hasattr(self, "_note_recovery_path_installed"):
            self._note_recovery_path_installed(path.header.stamp)
        self.get_logger().info(
            "[neupan_action_compat] accepted generic path: poses=%d end=(%.2f, %.2f)"
            % (len(path.poses), last_pose.position.x, last_pose.position.y)
        )
        return True

    def _execute_goal(self, goal_handle):
        result = NavAction.Result()
        try:
            with self._lock:
                if self._goal_preempt.is_set() or goal_handle.is_cancel_requested:
                    return self._finish_goal(goal_handle, result, preempt=True)
                self._clear_navigation("new goal accepted")
                self.goal_callback(goal_handle.request.target_pose)
            feedback = NavAction.Feedback()
            while rclpy.ok():
                with self._lock:
                    preempted = self._goal_preempt.is_set()
                if preempted or goal_handle.is_cancel_requested:
                    self._clear_navigation("goal action preempted")
                    return self._finish_goal(goal_handle, result, preempt=True)
                if self.arrive:
                    result.status = 0
                    goal_handle.succeed()
                    return result
                self._publish_goal_feedback(goal_handle, feedback)
                time.sleep(0.1)
            result.status = 1
            goal_handle.abort()
            return result
        except Exception as exc:
            self._handle_planner_exception("goal action", exc)
            result.status = 1
            goal_handle.abort()
            return result
        finally:
            self._release_and_promote("goal", goal_handle)

    def _execute_path(self, goal_handle):
        result = NavPathAction.Result()
        try:
            with self._lock:
                preinstalled = self._preinstalled_path_handle is goal_handle
                if preinstalled:
                    self._preinstalled_path_handle = None
                if self._path_preempt.is_set() or goal_handle.is_cancel_requested:
                    if preinstalled:
                        self._clear_navigation("preinstalled path preempted before execution")
                    return self._finish_path(goal_handle, result, preempt=True)
                if not preinstalled and not self._set_path_from_action(goal_handle.request.path):
                    result.status = 1
                    goal_handle.abort()
                    return result

            feedback = NavPathAction.Feedback()
            while rclpy.ok():
                with self._lock:
                    preempted = self._path_preempt.is_set()
                    pending = self._pending_path_handle
                    if preempted or goal_handle.is_cancel_requested:
                        if (
                            self._hot_path_enabled
                            and pending is not None
                            and not goal_handle.is_cancel_requested
                            and not self._planner_abort_reason
                            and self._control_enabled
                        ):
                            # Promotion and installation must be atomic with stop;
                            # otherwise an already-stopped goal can reopen the gate.
                            new_gh = pending
                            self._pending_path_handle = None
                            self._path_preempt.clear()
                            self._active_path_handle = new_gh
                            self.get_logger().info(
                                "[neupan_action_compat] hot path update: poses=%d"
                                % len(new_gh.request.path.poses)
                            )
                            if not self._set_path_from_action(new_gh.request.path, hot=True):
                                result.status = 1
                                goal_handle.abort()
                                return result
                            # The new execute callback must not cold-install this
                            # already-installed path and erase its warm velocity.
                            self._preinstalled_path_handle = new_gh
                            self._finish_path(goal_handle, result, preempt=True)
                            new_gh.execute()
                            return result
                        self._clear_navigation("path action preempted")
                        return self._finish_path(goal_handle, result, preempt=True)
                    if self.arrive:
                        result.status = 0
                        goal_handle.succeed()
                        return result
                self._publish_goal_feedback(goal_handle, feedback)
                time.sleep(0.1)
            result.status = 1
            goal_handle.abort()
            return result
        except Exception as exc:
            self._handle_planner_exception("path action", exc)
            result.status = 1
            goal_handle.abort()
            return result
        finally:
            self._release_and_promote("path", goal_handle)

    def _finish_goal(self, goal_handle, result, preempt=False):
        if self._planner_abort_reason:
            result.status = 1
            goal_handle.abort()
        else:
            result.status = 2
            # ROS2 only permits CANCELED after a client cancel request. Internal
            # replacement/stop has no CANCELING transition, so abort the handle.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        return result

    def _finish_path(self, goal_handle, result, preempt=False):
        return self._finish_goal(goal_handle, result, preempt=preempt)

    def _release_and_promote(self, kind, finished_handle):
        """Promote the next queued goal once the active goal finishes."""
        with self._lock:
            if kind == "goal":
                if self._active_goal_handle is not finished_handle:
                    return
                self._active_goal_handle = None
                self._goal_preempt.clear()
                self._planner_abort_reason = None
                nxt = self._pending_goal_handle
                self._pending_goal_handle = None
                if nxt is not None and rclpy.ok():
                    self._active_goal_handle = nxt
                    nxt.execute()
            else:
                if self._active_path_handle is not finished_handle:
                    return
                self._active_path_handle = None
                self._path_preempt.clear()
                self._planner_abort_reason = None
                nxt = self._pending_path_handle
                self._pending_path_handle = None
                if nxt is not None and rclpy.ok():
                    self._active_path_handle = nxt
                    nxt.execute()

    def _publish_goal_feedback(self, goal_handle, feedback):
        if self.robot_state is None or self.goal is None:
            return
        dx = self.goal[0, 0] - self.robot_state[0, 0]
        dy = self.goal[1, 0] - self.robot_state[1, 0]
        feedback.distance_remaining = float(np.hypot(dx, dy))
        goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()
    node = NeupanActionCompat()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    import threading

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
