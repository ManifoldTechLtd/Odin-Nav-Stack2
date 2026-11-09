"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework, which subscribes to the laser scan and localization information, and publishes the velocity command to the robot.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

Modified by Haowen Yu.
Copyright (c) 2026 Manifold Tech Ltd <yuhaowen@manifoldtech.cn>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

import json
import os
from collections import deque
from functools import partial
from math import sin, cos, atan2, pi, inf, sqrt, radians, degrees
import numpy as np
import yaml
from loguru import logger
import time

from neupan import neupan
from neupan.util.adaptive_d_max import AdaptiveDMax, robot_half_width
from neupan.util.stuck_escape import StuckEscapePolicy
from neupan.ros.ec_trace import ec, EC
from neupan.util.curvature_speed import CurvatureSpeedGovernor
from neupan.util import get_transform

# Shared ROS 1 / ROS 2 core: select ROS 1 rospy/tf or ROS 2 ros2_shim based on ROS_VERSION.
if os.environ.get("ROS_VERSION") == "2":
    from neupan.ros import ros2_shim
    rospy = ros2_shim.rospy
    tf = ros2_shim.tf
else:
    import rospy
    import tf

from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_interfaces.msg import RecoveryCommand, RecoveryRequest
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Float32, String
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2

# Parameters that can be tuned online via /neupan/<name> Float32 topics.
# NOTE: ro_obs and bk are intentionally NOT tunable online — they're not
# cvxpy Parameters because that would violate DPP. Change them in
# config/restart.
_TUNABLE_PARAMS = (
    "q_s",
    "q_theta",
    "p_u",
    "p_w",
    "p_dw",
    "eta",
    "d_max",
    "d_min",
)


def _fmt(value, digits=3):
    """Format an optional metric for logs ("n/a" while unavailable)."""
    if value is None:
        return "n/a"
    return "{:.{}f}".format(value, digits)


class _TracedRate:
    """Wrapper around rospy.Rate measuring actual sleep duration for loop_gap decomposition."""

    def __init__(self, rate, owner):
        self._rate = rate
        self._owner = owner

    def sleep(self):
        t0 = time.perf_counter()
        self._rate.sleep()
        self._owner._trace_sleep_ms = (time.perf_counter() - t0) * 1000.0


class neupan_core:
    def __init__(self) -> None:

        rospy.init_node("neupan_node", anonymous=True)

        # Config file selection via NEUPAN_ROS_CONFIG (falls back to configs/config.yaml).
        config_dir = os.path.join(os.path.dirname(__file__), "configs")
        config_path = os.path.join(config_dir, "config.yaml")
        env_config = os.environ.get("NEUPAN_ROS_CONFIG", "").strip()
        if env_config:
            candidate = (
                env_config
                if os.path.isabs(env_config)
                else os.path.join(config_dir, env_config)
            )
            if os.path.isfile(candidate):
                config_path = candidate
            else:
                logger.warning(
                    f"NEUPAN_ROS_CONFIG={env_config} not found, "
                    f"falling back to {config_path}"
                )
        logger.info(f"Loading ROS config: {config_path}")
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)
        logger.info(f"Configuration:\n {yaml.dump(self.config)}")

        self.scan_angle_range = np.array(self.config["scan_angle"], dtype=np.float32)
        self.scan_range = np.array(self.config["scan_range"], dtype=np.float32)

        pan = {"dune_checkpoint": self.config["dune_checkpoint"]}
        planner_config = self.config["planner_config_file"]
        if os.path.isabs(planner_config):
            planner_path = planner_config
        else:
            planner_path = os.path.join(os.path.dirname(config_path), planner_config)
            if not os.path.isfile(planner_path):
                planner_path = os.path.join(os.path.dirname(__file__), planner_config)
        self.neupan_planner = neupan.init_from_yaml(
            os.path.abspath(planner_path),
            pan=pan,
        )

        # Optional reference speed override via NEUPAN_REF_SPEED environment variable.
        _ref_env = os.environ.get("NEUPAN_REF_SPEED", "").strip()
        if _ref_env:
            try:
                _v = float(_ref_env)
                self.neupan_planner.set_reference_speed(_v)
                logger.warning(f"[site] NEUPAN_REF_SPEED override -> ref_speed={_v}")
            except ValueError:
                logger.warning(f"[site] invalid NEUPAN_REF_SPEED={_ref_env!r}, ignored")
        self._ref_speed_base = float(self.neupan_planner.ref_speed)
        self._curv = CurvatureSpeedGovernor(
            self.config.get("curvature_speed", {}) or {}, self._ref_speed_base
        )
        self._global_path_xy = None
        self._ref_speed_eff = self._ref_speed_base
        if self._curv.enable:
            logger.warning(
                f"[curvature_speed] ENABLED a_lat={self._curv.a_lat_max} "
                f"v_min={self._curv.v_min} lookahead={self._curv.lookahead_sec}s"
            )

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False

        # publisher
        self.vel_pub = rospy.Publisher(
            self.config["topic"]["cmd_vel"], Twist, queue_size=10
        )
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/neupan_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/neupan_initial_path", Path, queue_size=10
        )  # initial path
        self.control_trace_pub = rospy.Publisher(
            "/neupan/control_trace", String, queue_size=100
        )
        self.escape_trace_pub = rospy.Publisher(
            "/neupan/escape_trace", String, queue_size=100
        )
        self._control_trace_seq = 0
        self._last_scan_t = None
        self._last_scan_points = 0
        self._trace_last_loop_start_t = None
        self._trace_loop_gap_ms = None
        self._trace_tf_ms = 0.0
        self._trace_planner_ms = 0.0
        self._trace_visualization_ms = 0.0
        self._trace_sleep_ms = 0.0
        self._trace_scan_cb_ms = 0.0
        self._trace_escape_info = {}

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )
        self.arrive_pub = rospy.Publisher(self.config['topic']['arrive'], Empty, queue_size=1)
        self.replan_pub = rospy.Publisher(
            self.config["topic"].get("replan", "/neupan/replan"), Empty, queue_size=1
        )
        self._navigation_mode = rospy.get_param("/navigation_mode", "escape")
        if self._navigation_mode not in ("escape", "replan_periodic", "neupan_only"):
            rospy.logerr("Invalid /navigation_mode=%s; falling back to escape", self._navigation_mode)
            self._navigation_mode = "escape"
        recovery_cfg = self.config.get("recovery_contract", {}) or {}
        self._recovery_shadow_enable = bool(recovery_cfg.get("shadow_enable", False))
        self._recovery_behavior_enable = bool(recovery_cfg.get("behavior_enable", False))
        if self._navigation_mode != "escape":
            self._recovery_shadow_enable = False
            self._recovery_behavior_enable = False
        self._recovery_shadow_count = int(recovery_cfg.get("request_count", 5))
        self._recovery_side_min_clearance = float(
            recovery_cfg.get("side_min_clearance", 0.57)
        )
        self._recovery_retry_sec = float(recovery_cfg.get("retry_sec", 0.2))
        self._recovery_request_seq = 0
        self._recovery_request_id = None
        self._recovery_local_side = 0
        self._recovery_candidate_side = None
        self._recovery_shadow_sent = False
        self._recovery_waiting_commit = False
        self._recovery_still_blocked = False
        self._recovery_commit_pending_stamp = None
        self._recovery_last_installed_stamp = None
        self._recovery_retry_after = 0.0
        self.recovery_request_pub = rospy.Publisher(
            "/neupan/recovery_request", RecoveryRequest, queue_size=1
        )
        rospy.Subscriber(
            "/neupan/recovery_command", RecoveryCommand,
            self.recovery_command_callback, queue_size=1
        )
        self.last_arrive_flag: bool = False

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber(self.config["topic"]["scan"], LaserScan, self.scan_callback)

        _odom_topic_env = os.environ.get("NEUPAN_ODOM_TOPIC", "").strip()
        if _odom_topic_env:
            self.config.setdefault("topic", {})["odom"] = _odom_topic_env
        _odom_fb_env = os.environ.get("NEUPAN_ODOM_FEEDBACK", "").strip().lower()
        if _odom_fb_env:
            self.config.setdefault("odom_feedback", {})["enable"] = (
                _odom_fb_env in ("1", "true", "yes", "on")
            )

        odom_fb_cfg = self.config.get("odom_feedback", {}) or {}
        self._odom_feedback_enable = bool(odom_fb_cfg.get("enable", False))
        self._odom_stale_threshold = float(
            odom_fb_cfg.get("stale_threshold_sec", 0.2)
        )
        # Odin odometry twist.linear is reported in the world frame by default;
        # set linear_is_body_frame=true if a platform reports twist.linear in the body frame.
        self._odom_linear_is_body = bool(
            odom_fb_cfg.get("linear_is_body_frame", False)
        )
        og_cfg = self.config.get("output_gain", {}) or {}
        self._out_gain_enable = bool(og_cfg.get("enable", False))
        self._out_gain_w = float(og_cfg.get("w_gain", 1.0))
        self._out_gain_v_fwd = float(og_cfg.get("v_fwd_gain", 1.0))
        self._out_gain_v_bwd = float(og_cfg.get("v_bwd_gain", 1.0))
        self._last_odom_v = 0.0
        self._last_odom_w = 0.0
        self._last_odom_t = None  # ROS time of last odom; None until first msg


        smith_cfg = self.config.get("smith_predictor", {}) or {}
        self._smith_enable = bool(smith_cfg.get("enable", True))
        self._smith_fixed_lag_sec = float(smith_cfg.get("fixed_lag_sec", 0.04))
        self._smith_use_solve_time = bool(smith_cfg.get("use_solve_time", True))
        self._smith_max_tau_sec = float(smith_cfg.get("max_tau_sec", 0.30))
        self._smith_velocity_source = str(
            smith_cfg.get("velocity_source", "odom")
        ).lower()
        self._last_solve_sec = 0.0  # updated each cycle in run()

        needs_odom = self._odom_feedback_enable or (
            self._smith_enable and self._smith_velocity_source == "odom"
        )
        if needs_odom:
            odom_topic = self.config.get("topic", {}).get("odom", "/odin1/odometry")
            rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
            rospy.loginfo(
                f"[odom] subscribing {odom_topic} "
                f"(F1={self._odom_feedback_enable}, F2={self._smith_enable}, "
                f"linear_frame={'body' if self._odom_linear_is_body else 'world->body'})"
            )
        if self._odom_feedback_enable:
            rospy.loginfo("[F1] odom_feedback ENABLED")
        else:
            rospy.loginfo("[F1] odom_feedback DISABLED (using cmd feedback)")
        if self._smith_enable:
            rospy.loginfo(
                f"[F2] smith_predictor ENABLED "
                f"(fixed_lag={self._smith_fixed_lag_sec:.3f}s, "
                f"use_solve_time={self._smith_use_solve_time}, "
                f"max_tau={self._smith_max_tau_sec:.2f}s, "
                f"velocity={self._smith_velocity_source})"
            )
        else:
            rospy.loginfo("[F2] smith_predictor DISABLED")
        if self._out_gain_enable:
            rospy.loginfo(
                f"[out_gain] ENABLED v_fwd={self._out_gain_v_fwd:.3f} "
                f"v_bwd={self._out_gain_v_bwd:.3f} w={self._out_gain_w:.3f}"
            )
        else:
            rospy.loginfo("[out_gain] DISABLED")

        # three types of initial path:
        # 1. from given path
        # 2. from waypoints
        # 3. from goal position
        rospy.Subscriber(self.config["topic"]["path"], Path, self.path_callback)
        rospy.Subscriber(
            self.config["topic"]["waypoints"], Path, self.waypoints_callback
        )
        rospy.Subscriber(self.config["topic"]["goal"], PoseStamped, self.goal_callback)

        for name in _TUNABLE_PARAMS:
            rospy.Subscriber(
                f"/neupan/{name}",
                Float32,
                partial(self._adjust_param_callback, name=name),
            )

        self._last_cmd_v = 0.0
        self._last_cmd_w = 0.0
        self._last_cmd_t = None

        
        self._cmd_w_lpf_alpha = float(
            self.config.get("cmd_w_lpf_alpha", 0.7)
        )

        # Escape and adaptive-d_max policies live in neupan.util so that the
        # simulation benchmark drives the exact same code (see those modules).
        adaptive_cfg = self.config.get("adaptive_d_max", {}) or {}
        try:
            baseline_d_max = float(
                self.neupan_planner.pan.nrmp_layer.d_max.detach().item()
            )
        except Exception:
            baseline_d_max = None
        self._adaptive = AdaptiveDMax(adaptive_cfg, baseline_d_max=baseline_d_max)

        stuck_cfg = self.config.get("stuck_escape", {}) or {}
        self._escape = StuckEscapePolicy(
            stuck_cfg, criterion=stuck_cfg.get("criterion", "displacement")
        )
        if self._navigation_mode != "escape":
            self._escape.enable = False
        rospy.loginfo(
            "[navigation_mode] mode=%s escape=%s recovery=%s",
            self._navigation_mode, self._escape.enable,
            self._recovery_behavior_enable,
        )
        self._escape_criterion = self._escape.criterion
        self._escape_ref_path_installed = False
        self._escape_ref_path_obj = None

        # Gated hysteresis deadband on angular velocity during clean straight-line tracking.
        db_cfg = self.config.get("w_deadband", {}) or {}
        self._db_enable = bool(db_cfg.get("enable", False))
        self._db_enter = float(db_cfg.get("enter_rad", 0.02))
        self._db_exit = float(db_cfg.get("exit_rad", 0.05))
        self._db_zeroed = False
        self._db_straight_heading_max_rad = radians(
            float(db_cfg.get("straight_heading_err_max_deg", 8.0))
        )
        self._db_straight_curv_max = float(
            db_cfg.get("straight_curvature_max", 0.15)
        )
        self._db_straight_obs_min_m = float(
            db_cfg.get("straight_obstacle_min_dist_m", 0.9)
        )
        self._db_lookahead_m = float(db_cfg.get("straight_lookahead_m", 1.0))
        if self._db_enable:
            rospy.loginfo(
                "[w_deadband] ENABLED enter=%.3f exit=%.3f rad/s "
                "(gate: heading<%.1fdeg curv<%.2f obs>%.2fm)",
                self._db_enter, self._db_exit,
                degrees(self._db_straight_heading_max_rad),
                self._db_straight_curv_max,
                self._db_straight_obs_min_m,
            )

        # Dynamic follow target obstacle masking (disabled by default).
        follow_cfg = self.config.get("follow_target", {}) or {}
        self._follow_enable = bool(follow_cfg.get("enable", False))
        self._follow_topic = str(
            follow_cfg.get("topic", "/waypoint_manager/follow_target")
        )
        self._follow_stale_sec = float(follow_cfg.get("stale_sec", 2.0))
        self._follow_suspend_escape = bool(follow_cfg.get("suspend_escape", True))
        self._follow_target_map = None  # (x, y, radius) in map frame
        self._follow_target_t = 0.0
        if self._follow_enable:
            try:
                from nav_interfaces.msg import FollowTarget
            except ImportError:
                rospy.logerr(
                    "[follow_target] nav_interfaces/FollowTarget unavailable, "
                    "disabling target masking"
                )
                self._follow_enable = False
            else:
                rospy.Subscriber(
                    self._follow_topic, FollowTarget, self.follow_target_callback
                )
                rospy.loginfo(
                    f"[follow_target] ENABLED topic={self._follow_topic} "
                    f"stale={self._follow_stale_sec}s "
                    f"suspend_escape={self._follow_suspend_escape}"
                )
        else:
            rospy.loginfo("[follow_target] DISABLED")

        try:
            self._q_theta_normal = float(
                self.neupan_planner.pan.nrmp_layer.q_theta.detach().item()
            )
        except Exception:
            self._q_theta_normal = 0.2
        try:
            self._q_s_normal = float(
                self.neupan_planner.pan.nrmp_layer.q_s.detach().item()
            )
        except Exception:
            self._q_s_normal = 2.0

    def _adjust_param_callback(self, msg, name):
        """Generic Float32 callback dispatching to update_adjust_parameters()."""
        try:
            value = float(msg.data)
        except (TypeError, ValueError):
            rospy.logwarn(f"/neupan/{name}: invalid payload, ignored")
            return
        logger.info(f"Update {name} to {value}")
        if name == "q_theta":
            self._q_theta_normal = value
            effective = (
                value * self._escape.grace_q_theta_scale
                if self._escape.grace_active
                else value
            )
            self.neupan_planner.update_adjust_parameters(q_theta=effective)
        elif name == "q_s":
            self._q_s_normal = value
            effective = (
                value * self._escape.grace_q_s_scale
                if self._escape.grace_active
                else value
            )
            self.neupan_planner.update_adjust_parameters(q_s=effective)
        else:
            self.neupan_planner.update_adjust_parameters(**{name: value})

    def _control_cycle_token(self):
        """Standalone control is enabled; action adapters may gate each cycle."""
        return True

    def _publish_control_velocity(self, action, token):
        self.vel_pub.publish(self.generate_twist_msg(action))
        return True

    def _on_navigation_path_installed(self):
        """Allow action adapters to reactivate only on an explicit path/goal."""
        pass

    def run(self):

        r = _TracedRate(rospy.Rate(50), self)

        while not rospy.is_shutdown():
            loop_start_t = time.perf_counter()
            _ec_ticks = getattr(self, "_ec_loop_ticks", 0) + 1
            self._ec_loop_ticks = _ec_ticks
            if _ec_ticks % 250 == 0:
                ec(EC.NEUPAN_LOOP_ALIVE, "main_loop",
                   "ticks={} gap={}".format(
                       _ec_ticks,
                       "{:.1f}ms".format(self._trace_loop_gap_ms)
                       if getattr(self, "_trace_loop_gap_ms", None) is not None else "na"),
                   node="neupan", also_print=False)
            if self._trace_last_loop_start_t is None:
                self._trace_loop_gap_ms = None
            else:
                self._trace_loop_gap_ms = (
                    (loop_start_t - self._trace_last_loop_start_t) * 1000.0
                )
            self._trace_last_loop_start_t = loop_start_t
            self._trace_tf_ms = 0.0
            self._trace_planner_ms = 0.0
            self._trace_visualization_ms = 0.0

            control_token = self._control_cycle_token()
            if control_token is None:
                self._publish_control_trace("IDLE")
                r.sleep()
                continue

            tf_start_t = time.perf_counter()
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.config["frame"]["map"],
                    self.config["frame"]["base"],
                    rospy.Time(0),
                )
                self._trace_tf_ms = (time.perf_counter() - tf_start_t) * 1000.0

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)
                # Pose (and path progress) history for stall detection.
                now_pose = time.time()
                self._escape.note_pose(now_pose, float(x), float(y))
                if self._escape_criterion == "progress":
                    self._refresh_escape_reference_path()
                    if self._escape_ref_path_installed:
                        self._escape.note_progress(now_pose, float(x), float(y))

                if self._escape.grace_active:
                    self._update_post_escape_grace()
                elif self._escape.pb_enable:
                    self._update_path_block_grace()

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                self._trace_tf_ms = (time.perf_counter() - tf_start_t) * 1000.0
                rospy.loginfo_throttle(
                    3,
                    "waiting for tf for the transform from {} to {}".format(
                        self.config["frame"]["base"], self.config["frame"]["map"]
                    ),
                )
                self._publish_control_trace("WAIT_TF")
                r.sleep()
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(3, "waiting for robot state")
                r.sleep()
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                try:
                    self.neupan_planner.set_initial_path_from_state(self.robot_state)
                except Exception as exc:
                    handler = getattr(self, "_handle_planner_exception", None)
                    if handler is not None:
                        handler("initial path initialization", exc)
                    else:
                        logger.exception("initial path initialization failed")
                        self.vel_pub.publish(Twist())
                    r.sleep()
                    continue

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(3, "waiting for neupan initial path")
                self._publish_control_trace("WAIT_PATH")
                r.sleep()
                continue

            # Everything below drives the actual control-cycle output
            # (planning, escape override, cmd_vel publishing).
            # It is wrapped in a top-level guard so that any exception NOT
            # already caught by a more specific handler above cannot kill
            # this loop silently (which previously froze the robot with no
            # error ever reaching /rosout: the action server kept publishing
            # stale feedback from a dead control loop, so mission_manager's
            # watchdog never noticed).
            try:
                rospy.loginfo_once("initial Path Received")
                self.ref_path_pub.publish(
                    self.generate_path_msg(self.neupan_planner.initial_path)
                )

                if self.obstacle_points is None:
                    rospy.logwarn_throttle(
                        1, "No obstacle points, only path tracking task will be performed"
                    )

                # refresh d_max periodically based on local corridor width.
                if self._adaptive.enable:
                    self._update_adaptive_d_max()

                # Override priority: stuck-escape > NeuPAN MPC.
                escape_cmd = self._stuck_escape_decide_and_command()

                bypass_rate_limit = False
                control_mode = "MPC"
                if escape_cmd is not None:
                    action = escape_cmd
                    control_mode = (
                        "RECOVERY_WAIT"
                        if self._trace_escape_info.get("recovery_wait")
                        else "ESCAPE"
                    )
                    bypass_rate_limit = True
                    info = {
                        "stop": False,
                        "arrive": False,
                        "opt_state_list": [self.robot_state],
                        "ref_state_list": [self.robot_state],
                        "escape": True,
                    }
                else:
                    if not self.last_arrive_flag:
                        t_start = time.time()

                    if self._curv.enable and self._global_path_xy is not None:
                        self._ref_speed_eff = self._curv.update(x, y)
                        self.neupan_planner.set_reference_speed(self._ref_speed_eff)

                    # Smith Predictor — when enabled, plan from the state we
                    # PREDICT the robot will be in by the time this command takes
                    # effect (now + last_solve_time + fixed_actuator_lag) instead
                    # of from "where the robot is now". 
                    planner_state = self._predict_state_for_solve(self.robot_state)

                    planner_start_t = time.perf_counter()
                    if (planner_start_t - getattr(self, "_ec_last_solve_pre_t", 0.0)) >= 2.0:
                        self._ec_last_solve_pre_t = planner_start_t
                        try:
                            _n_pts = len(self.obstacle_points) if self.obstacle_points is not None else -1
                        except Exception:
                            _n_pts = -1
                        ec(EC.HEAVY_ALLOC_PRE, "solve_pre", "pts_in={}".format(_n_pts),
                           node="neupan", also_print=False)
                    try:
                        action, info = self.neupan_planner(planner_state, self.obstacle_points)
                        self._trace_planner_ms = (
                            (time.perf_counter() - planner_start_t) * 1000.0
                        )
                        if (planner_start_t - getattr(self, "_ec_last_solve_done_t", 0.0)) >= 5.0:
                            self._ec_last_solve_done_t = planner_start_t
                            ec(EC.MPC_SOLVE, "mpc_solve",
                               "solve={:.1f}ms pts_in={}".format(
                                   self._trace_planner_ms,
                                   len(self.obstacle_points) if self.obstacle_points is not None else -1),
                               node="neupan", also_print=False)
                    except Exception as exc:
                        self._trace_planner_ms = (
                            (time.perf_counter() - planner_start_t) * 1000.0
                        )
                        ec(EC.NEUPAN_SOLVE_FAIL, "solve_exception",
                           "{}: {}".format(type(exc).__name__, str(exc)[:120]),
                           node="neupan")
                        handler = getattr(self, "_handle_planner_exception", None)
                        if handler is not None:
                            handler("planner execution", exc)
                        else:
                            logger.exception("planner execution failed")
                            self.vel_pub.publish(Twist())
                        r.sleep()
                        continue

                    if not self.last_arrive_flag:
                        t_end = time.time()
                        self._last_solve_sec = t_end - t_start
                        rospy.loginfo(
                            f"neupan planning time: {self._last_solve_sec*1000:.1f} ms"
                        )

                    try:
                        pan = self.neupan_planner.pan
                        if hasattr(pan, "perf"):
                            perf = pan.perf
                            npts = pan.perf_n_points
                            rospy.loginfo_throttle(
                                2.0,
                                "[perf] solve={:.1f}ms | pf={:.1f}ms dune={:.1f}ms "
                                "nrmp={:.1f}ms | pts in={} dune_out={} nrmp_in={} "
                                "| min_dist={:.3f} | iter={} iters_run={}".format(
                                    self._last_solve_sec * 1000.0,
                                    perf["point_flow_ms"],
                                    perf["dune_ms"],
                                    perf["nrmp_ms"],
                                    npts["input"],
                                    npts["dune_out"],
                                    npts["nrmp_in"],
                                    float(self.neupan_planner.min_distance)
                                    if hasattr(self.neupan_planner, "min_distance")
                                    else -1.0,
                                    getattr(pan, "iter_num", -1),
                                    getattr(pan, "perf_iters", -1),
                                ),
                            )
                    except Exception:
                        pass

                if self._control_cycle_token() != control_token:
                    self._publish_control_trace("IDLE")
                    r.sleep()
                    continue

                self.stop = info["stop"]
                self.arrive = info["arrive"]

                if info["arrive"] and not self.last_arrive_flag:
                    self.arrive_pub.publish(Empty())
                    rospy.loginfo("arrive at the target")

                self.last_arrive_flag = info["arrive"]


                # Preserve the planner/escape output before smoothing and acceleration
                # limiting; trace publication below makes both values bag-visible.
                raw_action = action.copy()
                if bypass_rate_limit:
                    self._last_cmd_v = float(action[0, 0])
                    self._last_cmd_w = float(action[1, 0])
                    self._last_cmd_t = time.time()
                    self._escape.note_cmd(self._last_cmd_v, self._last_cmd_w)

                    v_fb, w_fb = self._velocity_for_feedback(
                        self._last_cmd_v, self._last_cmd_w
                    )
                    try:
                        self.neupan_planner.cur_vel_array[0, 0] = v_fb
                        self.neupan_planner.cur_vel_array[1, 0] = w_fb
                    except Exception as e:  # pragma: no cover - defensive
                        rospy.logwarn_throttle(5, f"escape feedback skipped: {e}")
                else:
                    # Clamp negative forward velocity during strafe-mode stall/grace windows.
                    if (self._escape.strafe_enable
                            and action[0, 0] < 0.0
                            and (self._escape.count > 0
                                 or self._escape.grace_active
                                 or self._escape.pb_active)):
                        action[0, 0] = 0.0
                    action = self._apply_w_deadband(action)
                    action = self._rate_limit_action(action)
                    if self._out_gain_enable:
                        action = self._apply_output_gain(action)


                visualization_start_t = time.perf_counter()
                opt_states = info.get("opt_state_list")
                if opt_states:
                    self.plan_pub.publish(self.generate_path_msg(opt_states))
                ref_states = info.get("ref_state_list")
                if ref_states:
                    self.ref_state_pub.publish(self.generate_path_msg(ref_states))
                if not info["arrive"] or not self.last_arrive_flag:
                    if not self._publish_control_velocity(action, control_token):
                        self._publish_control_trace("IDLE")
                        r.sleep()
                        continue

                self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
                self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
                self.robot_marker_pub.publish(self.generate_robot_marker_msg())
                # Covers path/ref/marker serialization and ROS publication after the
                # action is known. It is observational only; output cmd is unchanged.
                self._trace_visualization_ms = (
                    (time.perf_counter() - visualization_start_t) * 1000.0
                )
                self._publish_control_trace(control_mode, raw_action, action, info)

                if info["stop"]:
                    rospy.logwarn_throttle(
                        0.5,
                        "neupan stop with the min distance "
                        + str(self.neupan_planner.min_distance.detach().item())
                        + " threshold "
                        + str(self.neupan_planner.collision_threshold),
                    )
            except Exception as exc:
                handler = getattr(self, "_handle_planner_exception", None)
                if handler is not None:
                    handler("main control loop", exc)
                else:
                    logger.exception("main control loop failed")
                    self.vel_pub.publish(Twist())

            r.sleep()

    def _trace_age_sec(self, stamp):
        """Wall-clock age for a callback timestamp; ``None`` means unseen."""
        return None if stamp is None else max(0.0, time.time() - stamp)

    @staticmethod
    def _trace_action(action):
        """Return a JSON-safe velocity pair without changing control data."""
        if action is None:
            return None, None
        return float(action[0, 0]), float(action[1, 0])

    @staticmethod
    def _trace_action_y(action):
        if action is None or np.shape(action)[0] < 3:
            return None
        return float(action[2, 0])

    def _publish_control_trace(self, mode, raw_action=None, output_action=None, info=None):
        """Publish one non-blocking, bag-alignable NeuPAN control trace."""
        raw_v, raw_w = self._trace_action(raw_action)
        out_v, out_w = self._trace_action(output_action)
        info = info or {}
        trace_info = dict(self._trace_escape_info)
        trace_info.update(info)
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "seq": self._control_trace_seq,
            "mode": mode,
            "solve_ms": round(self._last_solve_sec * 1000.0, 3),
            # Phase timings let the next bag distinguish planner cost from TF,
            # ROS publication/marker cost, and whole-loop scheduling gaps.
            "loop_gap_ms": (round(self._trace_loop_gap_ms, 3)
                            if self._trace_loop_gap_ms is not None else None),
            "tf_ms": round(self._trace_tf_ms, 3),
            "planner_ms": round(self._trace_planner_ms, 3),
            "visualization_ms": round(self._trace_visualization_ms, 3),
            "sleep_ms": round(self._trace_sleep_ms, 3),
            "scan_cb_ms": round(self._trace_scan_cb_ms, 3),
            "scan_age_sec": self._trace_age_sec(self._last_scan_t),
            "odom_age_sec": self._trace_age_sec(self._last_odom_t),
            "scan_points": self._last_scan_points,
            "raw_v": raw_v,
            "raw_w": raw_w,
            "raw_vy": self._trace_action_y(raw_action),
            "output_v": out_v,
            "output_w": out_w,
            "output_vy": self._trace_action_y(output_action),
            "escape_count": trace_info.get("count"),
            "rotation_blocked": trace_info.get("rotation_blocked"),
            "bounded_rot": trace_info.get("bounded_rot"),
            "open_rot": trace_info.get("open_rot"),
            "sweep_cap_deg": trace_info.get("sweep_cap_deg"),
            "front_path_bypassed": trace_info.get("front_path_bypassed"),
            "creep_active": trace_info.get("creep_active"),
            "corridor_fast_enabled": getattr(self._escape, "corridor_fast_escape_enable", False),
            "nearest_obs": trace_info.get("nearest_obs"),
            "front_obs": trace_info.get("front_obs"),
            "recovery_request_id": self._recovery_request_id,
            "recovery_candidate_side": self._recovery_candidate_side,
            "recovery_wait": trace_info.get("recovery_wait", False),
            "recovery_wait_reason": trace_info.get("recovery_wait_reason"),
            "recovery_candidate_forced": trace_info.get("candidate_forced", False),
        }
        self._control_trace_seq += 1
        self.control_trace_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _publish_escape_trace(self, decision):
        """Expose actual escape command and gate state for post-run diagnosis."""
        if decision is None:
            return
        info = decision.info or {}
        cmd_v, cmd_w = self._trace_action(decision.cmd)
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "active": self._escape.active,
            "entered": decision.entered,
            "exited": decision.exited,
            "requested_replan": decision.request_replan,
            "intended_v": -self._escape.back_speed if self._escape.active else None,
            "intended_w": (self._escape.direction * self._escape.rotate_speed
                           if self._escape.active else None),
            "strafe_enabled": getattr(self._escape, "strafe_enable", False),
            "intended_vy": (
                self._escape.strafe_direction_sign * self._escape.direction
                * self._escape.strafe_speed
                if (getattr(self._escape, "strafe_enable", False)
                    and self._escape.active) else None),
            "cmd_vy": self._trace_action_y(decision.cmd),
            "strafe_side_min": info.get("strafe_side_min"),
            "strafe_side_blocked": info.get("strafe_side_blocked"),
            "actual_cmd_v": cmd_v,
            "actual_cmd_w": cmd_w,
            "rotation_blocked": info.get("rotation_blocked"),
            "nearest_obs": info.get("nearest_obs"),
            "front_obs": info.get("front_obs"),
            "yaw_delta_rad": info.get("rotation_done"),
            "stuck": info.get("stalled"),
            "count": info.get("count"),
            "in_corridor": info.get("in_corridor"),
            "corridor_hold": info.get("corridor_hold"),
            "ref_speed_eff": round(self._ref_speed_eff, 3),
            "yaw_rate": round(getattr(self._escape, "last_yaw_rate", 0.0), 3),
            "rotation_exempt": bool(getattr(self._escape, "last_rotation_exempt", False)),
            "d_max": getattr(self._adaptive, "last_d_max", None),
            "half_width": getattr(self._adaptive, "last_half_width", None),
            "guard_trips": getattr(self._adaptive, "guard_trips", None),
            "recovery_request_id": self._recovery_request_id,
            "recovery_candidate_side": self._recovery_candidate_side,
            "recovery_candidate_forced": info.get("candidate_forced", False),
            "recovery_wait": info.get("recovery_wait", False),
            "recovery_wait_reason": info.get("recovery_wait_reason"),
        }
        self.escape_trace_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _rate_limit_action(self, action):
        """
        Clamp the change in (v, omega) since the last published command using
        wall-clock dt and ``robot.max_acce``. The clamped values are written
        back into ``planner.cur_vel_array[:, 0]`` so that the MPC's next
        linearization point matches reality (this is the key piece the earlier
        ``neupan_ros_modify`` attempt was missing).

        Returns a 2x1 numpy array suitable for downstream publishing.
        """
        if action is None:
            return action

        v_cmd = float(action[0, 0])
        w_cmd = float(action[1, 0])

        now = time.time()
        if self._last_cmd_t is None:
            dt = float(self.neupan_planner.robot.dt)
        else:
            dt = max(now - self._last_cmd_t, 1e-3)

        if self._cmd_w_lpf_alpha > 0.0:
            a = self._cmd_w_lpf_alpha
            w_cmd = a * self._last_cmd_w + (1.0 - a) * w_cmd

        max_acce = self.neupan_planner.robot.max_acce  # (2, 1) numpy
        dv_max = float(max_acce[0, 0]) * dt
        dw_max = float(max_acce[1, 0]) * dt

        v_cmd = float(np.clip(v_cmd, self._last_cmd_v - dv_max, self._last_cmd_v + dv_max))
        w_cmd = float(np.clip(w_cmd, self._last_cmd_w - dw_max, self._last_cmd_w + dw_max))

        self._last_cmd_v = v_cmd
        self._last_cmd_w = w_cmd
        self._last_cmd_t = now
        self._escape.note_cmd(v_cmd, w_cmd)


        v_fb, w_fb = self._velocity_for_feedback(v_cmd, w_cmd)
        try:
            self.neupan_planner.cur_vel_array[0, 0] = v_fb
            self.neupan_planner.cur_vel_array[1, 0] = w_fb
        except Exception as e:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5, f"rate_limit feedback skipped: {e}")

        limited = np.array([[v_cmd], [w_cmd]], dtype=float)
        return limited

    def _apply_output_gain(self, action):
        """Apply calibrated output gain scaling to the final MPC command."""
        if action is None:
            return action
        v = float(action[0, 0])
        w = float(action[1, 0])
        v *= self._out_gain_v_fwd if v >= 0.0 else self._out_gain_v_bwd
        w *= self._out_gain_w
        try:
            vmax = float(self.neupan_planner.robot.max_speed[0, 0])
            wmax = float(self.neupan_planner.robot.max_speed[1, 0])
            v = float(np.clip(v, -vmax, vmax))
            w = float(np.clip(w, -wmax, wmax))
        except Exception:
            pass
        return np.array([[v], [w]], dtype=float)


    @staticmethod
    def _wrap_to_pi(rad):
        return (rad + pi) % (2 * pi) - pi


    def _update_adaptive_d_max(self):
        """Refresh ``d_max`` from the local corridor width (policy in
        ``neupan.util.adaptive_d_max``; this only adds ROS logging).

        The planner's reported obstacle distance is fed to the policy's safety
        net first: if the robot keeps ending up very close to an obstacle, the
        baseline ``d_max`` is restored immediately, whatever the estimate says.
        """
        now = time.time()
        try:
            md = self.neupan_planner.min_distance
            md = float(md.detach().cpu()) if hasattr(md, "detach") else float(md)
        except Exception:
            md = None
        restored = self._adaptive.note_min_distance(now, md)
        if restored is not None:
            try:
                self.neupan_planner.update_adjust_parameters(d_max=restored)
            except Exception as e:  # pragma: no cover - defensive
                rospy.logwarn_throttle(5, f"adaptive guard restore skipped: {e}")
            rospy.logwarn(
                "[adaptive_d_max] GUARD tripped: min_dist={:.3f}m below {:.2f}m "
                "-> d_max restored to baseline {:.3f} (cooldown {:.1f}s, trip #{})"
                .format(md, self._adaptive.guard_min_dist_m, restored,
                        self._adaptive.guard_cooldown_sec,
                        self._adaptive.guard_trips)
            )

        applied = self._adaptive.update(
            time.time(),
            self.robot_state,
            self.obstacle_points,
            robot_half_width(self.neupan_planner.robot),
            lambda d_max: self.neupan_planner.update_adjust_parameters(d_max=d_max),
        )
        if applied is not None:
            rospy.loginfo_throttle(
                2.0,
                "[adaptive_d_max] half_width={:.2f} -> d_max={:.3f}".format(
                    self._adaptive.last_half_width, applied
                ),
            )
            ec(EC.ADAPTIVE_DMAX, "adaptive", "half_width={:.2f} d_max={:.3f}".format(
                self._adaptive.last_half_width, applied), node="neupan", also_print=False)

    # ----------------------------------------------------------------------
    # Stuck-escape (opt-in)
    # ----------------------------------------------------------------------
    def _clear_recovery_episode(self):
        self._recovery_request_id = None
        self._recovery_local_side = 0
        self._recovery_candidate_side = None
        self._recovery_shadow_sent = False
        self._recovery_waiting_commit = False
        self._recovery_still_blocked = False
        self._recovery_commit_pending_stamp = None

    def _publish_recovery_state(self, state, reason):
        if self._recovery_request_id is None or self.robot_state is None:
            return
        request = RecoveryRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.config["frame"]["map"]
        request.request_id = self._recovery_request_id
        request.state = state
        request.local_side = self._recovery_local_side
        request.robot_pose.position.x = float(self.robot_state[0, 0])
        request.robot_pose.position.y = float(self.robot_state[1, 0])
        request.robot_pose.orientation = self.yaw_to_quat(
            float(self.robot_state[2, 0])
        )
        request.reason = reason
        self.recovery_request_pub.publish(request)

    def _update_recovery_shadow(self, info):
        if (not self._recovery_shadow_enable or self.robot_state is None
                or "count" not in info):
            return
        count = int(info.get("count") or 0)
        if count == 0 and not self._escape.active:
            if (self._recovery_behavior_enable and self._recovery_shadow_sent
                    and not self._recovery_waiting_commit):
                self._publish_recovery_state(
                    RecoveryRequest.RECOVERED, "stuck_count_recovered"
                )
            self._clear_recovery_episode()
            return
        if (count < self._recovery_shadow_count or self._recovery_shadow_sent
                or time.time() < self._recovery_retry_after):
            return
        left_min, _ = self._escape.side_clearance(
            self.robot_state, self.obstacle_points, +1
        )
        right_min, _ = self._escape.side_clearance(
            self.robot_state, self.obstacle_points, -1
        )
        local_side = 1 if left_min >= right_min else -1
        self._recovery_request_seq += 1
        stamp = rospy.Time.now()
        request = RecoveryRequest()
        request.header.stamp = stamp
        request.header.frame_id = self.config["frame"]["map"]
        request.request_id = "shadow_{}_{}".format(
            stamp.to_nsec(), self._recovery_request_seq
        )
        request.state = RecoveryRequest.BLOCKED
        request.local_side = local_side
        request.robot_pose.position.x = float(self.robot_state[0, 0])
        request.robot_pose.position.y = float(self.robot_state[1, 0])
        request.robot_pose.orientation = self.yaw_to_quat(
            float(self.robot_state[2, 0])
        )
        request.reason = "count={}/{} left={:.3f} right={:.3f}".format(
            count, self._escape.trigger_frames, left_min, right_min
        )
        self._recovery_request_id = request.request_id
        self._recovery_local_side = local_side
        self._recovery_shadow_sent = True
        self.recovery_request_pub.publish(request)
        rospy.logwarn(
            "[recovery_shadow] request id={} count={} local_side={:+d} "
            "left={:.2f} right={:.2f} (no behavior change)".format(
                request.request_id, count, local_side, left_min, right_min
            )
        )

    def _note_recovery_path_installed(self, stamp):
        stamp_ns = stamp.to_nsec() if stamp is not None else 0
        self._recovery_last_installed_stamp = stamp_ns
        if (self._recovery_commit_pending_stamp is not None
                and self._recovery_commit_pending_stamp == stamp_ns):
            self._clear_recovery_episode()

    def recovery_command_callback(self, msg):
        if not self._recovery_shadow_enable:
            return
        if msg.request_id != self._recovery_request_id:
            rospy.logwarn_throttle(
                2.0, "[recovery_shadow] ignored stale command id={}".format(
                    msg.request_id
                )
            )
            return
        rospy.logwarn(
            "[recovery_shadow] response id={} command={} local/candidate={:+d}/{:+d} "
            "heading={:+.1f}deg flags={} reason={} poses={} behavior={}".format(
                msg.request_id, msg.command,
                self._recovery_local_side, int(msg.side),
                msg.signed_heading * 180.0 / pi,
                msg.flags, msg.reason, len(msg.path.poses),
                self._recovery_behavior_enable,
            )
        )
        if not self._recovery_behavior_enable:
            return
        if msg.command == RecoveryCommand.SIDE and msg.side in (-1, 1):
            self._recovery_candidate_side = int(msg.side)
        elif msg.command == RecoveryCommand.COMMIT:
            stamp_ns = msg.path.header.stamp.to_nsec()
            if not msg.path.poses or stamp_ns == 0:
                self._publish_recovery_state(
                    RecoveryRequest.ABORTED, "commit_missing_path_or_stamp"
                )
                self._clear_recovery_episode()
                return
            if self._recovery_last_installed_stamp == stamp_ns:
                self._clear_recovery_episode()
            else:
                self._recovery_commit_pending_stamp = stamp_ns
        elif msg.command == RecoveryCommand.WAIT and msg.reason != "still_blocked":
            self._publish_recovery_state(
                RecoveryRequest.ABORTED, "retry_after_wait_response"
            )
            self._clear_recovery_episode()
            self._recovery_retry_after = time.time() + self._recovery_retry_sec
        elif msg.command == RecoveryCommand.ABORT:
            self._clear_recovery_episode()

    def _stuck_escape_decide_and_command(self):
        """Run one step of the shared escape policy and translate the result
        into ROS side effects (logging, replan requests, MPC state reset).

        The decision logic itself lives in ``neupan.util.stuck_escape`` so the
        simulation benchmark exercises the same code path.
        """
        if not self._escape.enable or self.robot_state is None:
            self._trace_escape_info = {}
            return None

        # Suspend stuck_escape while a fresh follow_target is active.
        if self._follow_suspend_escape and self._follow_target_active():
            rospy.loginfo_throttle(
                5.0, "[escape] follow target active; stuck_escape suspended"
            )
            self._trace_escape_info = {}
            return None

        if self._recovery_behavior_enable and self._recovery_waiting_commit:
            wait_reason = "waiting_commit"
            if self._recovery_still_blocked:
                front_obs = self._escape.min_obs_distance_in_cone(
                    self.robot_state, self.obstacle_points
                )
                wait_reason = "still_blocked"
                if front_obs > self._escape.exit_obs_distance:
                    self._recovery_still_blocked = False
                    wait_reason = "cleared_while_waiting"
                    self._publish_recovery_state(
                        RecoveryRequest.CLEARED, wait_reason
                    )
            self._trace_escape_info = {
                "recovery_wait": True,
                "recovery_wait_reason": wait_reason,
                "candidate_side": self._recovery_candidate_side,
            }
            return np.zeros((2, 1), dtype=float)

        decision = self._escape.decide(
            time.time(), self.robot_state, self.obstacle_points,
            preferred_direction=self._recovery_candidate_side,
            require_preferred_direction=self._recovery_behavior_enable,
            preferred_min_clearance=self._recovery_side_min_clearance,
        )
        info = decision.info
        self._trace_escape_info = info
        self._update_recovery_shadow(info)
        actual_v, actual_w = self._trace_action(decision.cmd)

        if decision.entered:
            if self._recovery_behavior_enable and self._recovery_request_id:
                self._publish_recovery_state(
                    RecoveryRequest.ESCAPING,
                    "candidate_side={}".format(self._recovery_candidate_side),
                )
            rospy.logwarn(
                "[escape] ENTER: stuck for {} frames "
                "({}={}, L_min={:.2f}m/{}pts, R_min={:.2f}m/{}pts, "
                "front={:.2f}m), dir={:+}{}".format(
                    self._escape.trigger_frames,
                    info.get("metric_name"), _fmt(info.get("metric")),
                    info.get("left_min", float("inf")), info.get("left_count", 0),
                    info.get("right_min", float("inf")), info.get("right_count", 0),
                    info.get("front_obs", float("inf")),
                    info.get("direction", 1.0),
                    " [LIMIT-CYCLE FLIPPED]" if info.get("limit_cycle_flip") else "",
                    " [STRAFE]" if info.get("strafe") else "",
                )
            )
        elif decision.exited:
            if self._recovery_behavior_enable and self._recovery_request_id:
                self._recovery_waiting_commit = True
                self._recovery_still_blocked = not bool(info.get("cleared"))
                if info.get("cleared"):
                    self._publish_recovery_state(
                        RecoveryRequest.CLEARED, "escape_cleared"
                    )
                else:
                    self._publish_recovery_state(
                        RecoveryRequest.STILL_BLOCKED,
                        "escape_time_up_without_clearance",
                    )
            # Reset the MPC nominal velocity so it does not have to ramp up
            # from the escape command against the p_u/p_w smoothness
            # penalties -- that ramp used to make the dog crawl for ~1 s after
            # each escape, often re-entering the stuck pattern.
            try:
                self.neupan_planner.cur_vel_array[0, 0] = 0.0
                self.neupan_planner.cur_vel_array[1, 0] = 0.0
            except Exception as e:  # pragma: no cover - defensive
                rospy.logwarn_throttle(5, f"escape exit vel reset skipped: {e}")
            self._last_cmd_v = 0.0
            self._last_cmd_w = 0.0
            rospy.loginfo(
                "[escape] exit: elapsed={:.2f}s, front_obs={:.2f}m, rot={:.0f}deg, "
                "time_up={}, cleared={}{}".format(
                    info.get("elapsed", 0.0), info.get("front_obs", float("inf")),
                    info.get("rotation_done", 0.0) * 180 / pi,
                    info.get("time_up"), info.get("cleared"),
                    " [SIDE-BLOCKED {:.2f}m]".format(info["strafe_side_min"])
                    if info.get("strafe_side_blocked") else "",
                )
            )
        elif self._escape.active:
            rospy.loginfo_throttle(
                0.5,
                "[escape] active: actual(v={:.2f}, w={:+.2f}), intended_w={:+.2f}, "
                "gate_blocked={}, nearest={:.2f}m, elapsed={:.2f}s, front_obs={:.2f}m, "
                "rot={:.0f}/{:.0f}deg".format(
                    actual_v if actual_v is not None else 0.0,
                    actual_w if actual_w is not None else 0.0,
                    self._escape.direction * self._escape.rotate_speed,
                    info.get("rotation_blocked", False),
                    info.get("nearest_obs", float("inf")),
                    info.get("elapsed", 0.0),
                    info.get("front_obs", float("inf")),
                    info.get("rotation_done", 0.0) * 180 / pi,
                    self._escape.min_rotation * 180 / pi,
                ),
            )
        elif info.get("count") is not None and (
            info.get("count") > 0
            or info.get("front_obs", float("inf")) < self._escape.front_obs_threshold
        ):
            _ec_esc_t = time.time()
            if _ec_esc_t - getattr(self, "_ec_esc_last_t", 0.0) >= 5.0:
                self._ec_esc_last_t = _ec_esc_t
                ec(EC.ESCAPE_WATCH, "escape_watch",
                   "v={:+.2f} w={:+.2f} front={:.2f} stuck={} count={}/{}".format(
                       self._last_cmd_v, self._last_cmd_w,
                       info.get("front_obs", float("inf")), info.get("stalled"),
                       info.get("count"), self._escape.trigger_frames),
                   node="neupan", also_print=False)
            rospy.loginfo_throttle(
                1.0,
                "[escape] watching: {}({:.1f}s)={} v={:+.2f} w={:+.2f} "
                "front_obs={:.2f} stuck={} count={}/{}".format(
                    info.get("metric_name"), self._escape.window_sec,
                    _fmt(info.get("metric")),
                    self._last_cmd_v, self._last_cmd_w,
                    info.get("front_obs", float("inf")), info.get("stalled"),
                    info.get("count"), self._escape.trigger_frames,
                ),
            )

        if decision.grace_scales is not None:
            q_theta_scale, q_s_scale = decision.grace_scales
            ec(EC.GRACE_TRIGGER, "grace", "q_theta_scale={:.3f} q_s_scale={:.3f}".format(
                q_theta_scale, q_s_scale), node="neupan")
            try:
                self.neupan_planner.update_adjust_parameters(
                    q_theta=self._q_theta_normal * q_theta_scale,
                    q_s=self._q_s_normal * q_s_scale,
                )
            except Exception as e:  # pragma: no cover - defensive
                rospy.logwarn_throttle(5, f"escape grace scale skipped: {e}")
            rospy.loginfo(
                "[escape] grace ON: q_theta {:.3f}->{:.3f}, q_s {:.3f}->{:.3f} "
                "(need {:.1f}m + front clear, max {:.1f}s)".format(
                    self._q_theta_normal, self._q_theta_normal * q_theta_scale,
                    self._q_s_normal, self._q_s_normal * q_s_scale,
                    self._escape.grace_min_distance, self._escape.grace_max,
                )
            )

        if decision.request_replan:
            if self._recovery_behavior_enable and self._recovery_request_id:
                rospy.loginfo(
                    "[recovery] suppressed legacy replan during recovery episode"
                )
            else:
                self.arrive_pub.publish(Empty())
                self.replan_pub.publish(Empty())
                self._escape_ref_path_installed = False
                rospy.loginfo(
                    "[escape] requested replan ({})".format(
                        "after escape" if decision.exited else
                        "early, count={}".format(info.get("count"))
                    )
                )

        self._publish_escape_trace(decision)
        if (self._recovery_behavior_enable and decision.exited
                and self._recovery_request_id):
            return np.zeros((2, 1), dtype=float)
        return decision.cmd

    def _refresh_escape_reference_path(self):
        """Install the planner's reference path for the ``progress`` criterion when the path object changes."""
        path = getattr(self.neupan_planner, "initial_path", None)
        if path is None:
            return
        if self._escape_ref_path_installed and path is self._escape_ref_path_obj:
            return
        self._escape.set_reference_path(path)
        self._escape_ref_path_obj = path
        self._escape_ref_path_installed = True

    def _update_post_escape_grace(self):
        """Restore the nominal q_theta/q_s once the grace window closes
        (policy in ``neupan.util.stuck_escape``)."""
        restore = self._escape.update_grace(
            time.time(), self.robot_state, self.obstacle_points
        )
        if restore is None:
            return
        try:
            self.neupan_planner.update_adjust_parameters(
                q_theta=self._q_theta_normal, q_s=self._q_s_normal,
            )
        except Exception as e:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5, f"grace restore skipped: {e}")
        rospy.loginfo(
            "[escape] grace OFF: q_theta->{:.3f}, q_s->{:.3f}".format(
                self._q_theta_normal, self._q_s_normal
            )
        )

    def _compute_path_block(self) -> bool:
        """Return True when the first 5 nominal reference states lie within `escape.pb_dist` of an obstacle point."""
        pts = self.obstacle_points
        ref = (getattr(self.neupan_planner, "info", None) or {}).get(
            "ref_state_list"
        )
        if pts is None or not ref:
            return False
        dmin_sq = float("inf")
        for st in ref[:5]:
            if st is None:
                continue
            dx = pts[0] - float(st[0, 0])
            dy = pts[1] - float(st[1, 0])
            d_sq = float((dx * dx + dy * dy).min())
            if d_sq < dmin_sq:
                dmin_sq = d_sq
        return dmin_sq < self._escape.pb_dist ** 2

    def _update_path_block_grace(self):
        """Apply or restore path-block grace scaling on q_theta and q_s."""
        pb = self._escape.note_path_block(self._compute_path_block())
        if pb == getattr(self, "_pb_applied", False):
            return
        self._pb_applied = pb
        try:
            if pb:
                self.neupan_planner.update_adjust_parameters(
                    q_theta=self._q_theta_normal * self._escape.grace_q_theta_scale,
                    q_s=self._q_s_normal * self._escape.grace_q_s_scale,
                )
                rospy.loginfo(
                    "[grace] path-block ON: q_theta {:.3f}->{:.3f}, q_s {:.3f}->{:.3f} "
                    "(path blocked by points < {:.2f}m)".format(
                        self._q_theta_normal,
                        self._q_theta_normal * self._escape.grace_q_theta_scale,
                        self._q_s_normal,
                        self._q_s_normal * self._escape.grace_q_s_scale,
                        self._escape.pb_dist,
                    )
                )
            else:
                self.neupan_planner.update_adjust_parameters(
                    q_theta=self._q_theta_normal, q_s=self._q_s_normal
                )
                rospy.loginfo(
                    "[grace] path-block OFF: q_theta->{:.3f}, q_s->{:.3f}".format(
                        self._q_theta_normal, self._q_s_normal
                    )
                )
        except Exception as e:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5, f"path-block grace skipped: {e}")
            self._pb_applied = False

    def _straight_state_ok(self):
        """Return True during clean straight-line tracking (small heading error, low curvature, no nearby obstacles)."""
        path = getattr(self, "_global_path_xy", None)
        if path is None or len(path) < 4 or self.robot_state is None:
            return False
        px = float(self.robot_state[0, 0])
        py = float(self.robot_state[1, 0])
        yaw = float(self.robot_state[2, 0])
        d2 = (path[:, 0] - px) ** 2 + (path[:, 1] - py) ** 2
        i = int(np.argmin(d2))
        seg = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
        step = float(np.median(seg[seg > 1e-6])) if np.any(seg > 1e-6) else 0.05
        k = max(1, int(round(self._db_lookahead_m / step)))
        last = len(path) - 1

        def tangent(idx):
            a = path[min(idx, last)]
            b = path[min(idx + k, last)]
            if np.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
                return None
            return atan2(b[1] - a[1], b[0] - a[0])

        def wrap(a):
            return (a + pi) % (2.0 * pi) - pi

        t0 = tangent(i)
        t1 = tangent(min(i + k, last))
        if t0 is None or t1 is None:
            return False
        if abs(wrap(t0 - yaw)) > self._db_straight_heading_max_rad:
            return False
        arc = max(k * step, 1e-3)
        if abs(wrap(t1 - t0)) / arc > self._db_straight_curv_max:
            return False
        pts = self.obstacle_points
        if pts is not None and getattr(pts, "size", 0):
            d_obs = float(
                np.sqrt(((pts[0] - px) ** 2 + (pts[1] - py) ** 2).min())
            )
            if d_obs < self._db_straight_obs_min_m:
                return False
        return True

    def _apply_w_deadband(self, action):
        """Apply gated hysteresis deadband on angular velocity during clean straight-line tracking."""
        if not self._db_enable or action is None:
            return action
        if not self._straight_state_ok():
            self._db_zeroed = False
            return action
        w = float(action[1, 0])
        if self._db_zeroed:
            if abs(w) >= self._db_exit:
                self._db_zeroed = False
        elif abs(w) < self._db_enter:
            self._db_zeroed = True
        if not self._db_zeroed:
            return action
        out = action.copy()
        out[1, 0] = 0.0
        return out

    # scan callback
    def follow_target_callback(self, msg):
        """Cache the latest FollowTarget masking circle in the map frame."""
        if not msg.valid or msg.radius <= 0.0:
            if self._follow_target_map is not None:
                rospy.loginfo("[follow_target] target invalid, restoring obstacle treatment")
            self._follow_target_map = None
            return

        target = PoseStamped()
        target.header = msg.header
        if not target.header.frame_id:
            target.header.frame_id = "odom"
        target.pose.position.x = msg.x
        target.pose.position.y = msg.y
        target.pose.orientation.w = 1.0

        map_frame = self.config["frame"]["map"]
        try:
            if target.header.frame_id != map_frame:
                target = self.listener.transformPose(map_frame, target)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(2.0, f"[follow_target] TF transform failed: {exc}")
            return

        self._follow_target_map = (
            float(target.pose.position.x),
            float(target.pose.position.y),
            float(msg.radius),
        )
        self._follow_target_t = time.time()

    def _follow_target_active(self):
        """Return True when follow target masking is enabled and the cached target is fresh."""
        return (
            self._follow_enable
            and self._follow_target_map is not None
            and (time.time() - self._follow_target_t) <= self._follow_stale_sec
        )

    def _mask_follow_target(self, points):
        """Remove obstacle points inside the active follow target circle in map coordinates."""
        if points is None or not self._follow_target_active():
            return points
        tx, ty, radius = self._follow_target_map
        dx = points[0, :] - tx
        dy = points[1, :] - ty
        keep = (dx * dx + dy * dy) > (radius * radius)
        removed = int(points.shape[1] - keep.sum())
        if removed > 0:
            rospy.loginfo_throttle(
                1.0,
                f"[follow_target] masked {removed} obstacle points inside target circle "
                f"(center=({tx:.2f}, {ty:.2f}), r={radius:.2f}m)",
            )
        filtered = points[:, keep]
        return filtered if filtered.shape[1] > 0 else None

    def _note_scan_cb(self, t0):
        """Record scan_callback wall duration (subscriber-thread GIL load)."""
        self._trace_scan_cb_ms = (time.perf_counter() - t0) * 1000.0

    def scan_callback(self, scan_msg):
        # Reception timestamp is intentionally wall-clock: it measures callback
        # freshness even when sensor header stamps or TF are delayed.
        self._last_scan_t = time.time()
        t_cb0 = time.perf_counter()

        if self.robot_state is None:
            self._note_scan_cb(t_cb0)
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        # x, y, z, yaw, pitch, roll = self.lidar_offset

        if self.config["flip_angle"]:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.config["scan_downsample"] == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        self._last_scan_points = len(points)
        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            self._note_scan_cb(t_cb0)
            return None

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.config["frame"]["map"],
                self.config["frame"]["lidar"],
                rospy.Time(0),
            )

            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = self._mask_follow_target(
                rot_matrix @ point_array + trans_matrix
            )
            rospy.loginfo_once("Scan obstacle points Received")

            self._note_scan_cb(t_cb0)
            return

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                3,
                "waiting for tf for the transform from {} to {}".format(
                    self.config["frame"]["lidar"], self.config["frame"]["map"]
                ),
            )
            self._note_scan_cb(t_cb0)
            return

    def path_callback(self, path):

        initial_point_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y

            if self.config["include_initial_path_direction"]:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once(
                    "Using the points gradient as the initial path direction"
                )

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]

            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if (
            self.neupan_planner.initial_path is None
            or self.config["refresh_initial_path"]
        ):
            rospy.loginfo_throttle(0.1, "initial path update from given path")
            self._global_path_xy = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in path.poses]
            )
            self._curv.set_path(self._global_path_xy)
            self.neupan_planner.set_initial_path(initial_point_list)
            self.neupan_planner.reset()
            self._on_navigation_path_installed()

    def waypoints_callback(self, path):
        """
        Utilize multiple waypoints (goals) to set the initial path
        """

        waypoints_list = [self.robot_state]

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y

            if self.config["include_initial_path_direction"]:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once(
                    "Using the points gradient as the initial path direction"
                )

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]

            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if (
            self.neupan_planner.initial_path is None
            or self.config["refresh_initial_path"]
        ):
            rospy.loginfo_throttle(0.1, "initial path update from waypoints")
            self.neupan_planner.update_initial_path_from_waypoints(waypoints_list)
            self.neupan_planner.reset()
            self._on_navigation_path_installed()

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        rospy.loginfo("set neupan goal: {}".format([x, y, theta]))
        rospy.loginfo_throttle(0.1, "initial path update from goal position")
        self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.neupan_planner.reset()
        self._on_navigation_path_installed()

    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    # ----------------------------------------------------------------------
    # Odom velocity feedback
    # ----------------------------------------------------------------------
    def odometry_callback(self, msg):
        """Cache the latest body-frame forward speed and yaw-rate from odometry.

        Used by `_velocity_for_feedback` (F1) and Smith predictor (F2) to
        feed the MPC linearization point. When `linear_is_body_frame` is false,
        rotates world-frame `twist.linear` into the body frame using the message
        pose yaw (`v_forward = vx * cos(yaw) + vy * sin(yaw)`).
        """
        if self._odom_linear_is_body:
            v_fwd = float(msg.twist.twist.linear.x)
        else:
            yaw = self.quat_to_yaw(msg.pose.pose.orientation)
            vx_w = float(msg.twist.twist.linear.x)
            vy_w = float(msg.twist.twist.linear.y)
            v_fwd = vx_w * cos(yaw) + vy_w * sin(yaw)
        self._last_odom_v = v_fwd
        self._last_odom_w = float(msg.twist.twist.angular.z)
        self._last_odom_t = time.time()
        self._escape.note_rotation(time.time(), self._last_odom_w)
        self._escape.note_motion(time.time(), self._last_odom_v)

    def _velocity_for_feedback(self, v_cmd, w_cmd):
        """Return (v, w) to write into NeuPAN's cur_vel_array warm-start.
        Prefers odom-measured values when F1 is enabled and odom is fresh;
        falls back to the just-published cmd otherwise (= legacy behavior)."""
        if not self._odom_feedback_enable:
            return v_cmd, w_cmd
        if self._last_odom_t is None:
            rospy.logwarn_throttle(
                5.0, "[F1] no odom received yet; using cmd feedback"
            )
            return v_cmd, w_cmd
        age = time.time() - self._last_odom_t
        if age > self._odom_stale_threshold:
            rospy.logwarn_throttle(
                2.0,
                f"[F1] odom stale ({age:.2f}s > {self._odom_stale_threshold:.2f}s); "
                "falling back to cmd feedback",
            )
            return v_cmd, w_cmd
        return self._last_odom_v, self._last_odom_w

    # ----------------------------------------------------------------------
    # Smith Predictor (state forward-roll by control delay tau)
    # ----------------------------------------------------------------------
    def _predict_state_for_solve(self, state):
        """Forward-roll the robot state by tau = last_solve_time + fixed_lag
        using current measured (or commanded) velocity, so the MPC plans
        starting from where the robot WILL be when its next command takes
        effect — not where it currently is. Compensates the ~100-200ms
        cmd-to-effect delay that creates phase lag in the MPC loop.

        When disabled, returns the state unchanged (= legacy behavior).
        Diff-drive only.
        """
        if not self._smith_enable or state is None:
            return state

        tau = self._smith_fixed_lag_sec
        if self._smith_use_solve_time:
            tau += float(self._last_solve_sec)
        if tau > self._smith_max_tau_sec:
            tau = self._smith_max_tau_sec
        if tau <= 1e-3:
            return state

        v, w = self._smith_pick_velocity()

        theta = float(state[2, 0])
        rolled = state.copy()
        rolled[0, 0] = float(state[0, 0]) + v * cos(theta) * tau
        rolled[1, 0] = float(state[1, 0]) + v * sin(theta) * tau
        rolled[2, 0] = float(state[2, 0]) + w * tau
        return rolled

    def _smith_pick_velocity(self):
        """Choose (v, w) to forward-roll during the predictor's tau window.
        Honors `smith_predictor.velocity_source` (`odom`|`cmd`) with
        automatic fallback when odom is stale/missing."""
        if self._smith_velocity_source == "cmd":
            return self._last_cmd_v, self._last_cmd_w
        # default: odom (with fallback)
        if (
            self._last_odom_t is not None
            and (time.time() - self._last_odom_t) <= self._odom_stale_threshold
        ):
            return self._last_odom_v, self._last_odom_w
        rospy.logwarn_throttle(
            5.0,
            "[F2] odom unavailable/stale; predictor using last cmd as fallback",
        )
        return self._last_cmd_v, self._last_cmd_w

    # generate ros message
    def generate_path_msg(self, path_list):

        path = Path()
        path.header.frame_id = self.config["frame"]["map"]
        path.header.stamp = rospy.Time.now()

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.config["frame"]["map"]

            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):

        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        if self.stop or self.arrive:
            # print('stop flag true')
            return Twist()

        else:
            action = Twist()

            action.linear.x = speed
            action.angular.z = steer
            if np.shape(vel)[0] > 2:
                action.linear.y = float(vel[2, 0])

            return action

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return marker_array
        else:
            points = self.neupan_planner.dune_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.config["frame"]["map"]
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.config["marker_size"]
                marker.scale.y = self.config["marker_size"]
                marker.scale.z = self.config["marker_size"]
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return marker_array
        else:
            points = self.neupan_planner.nrmp_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.config["frame"]["map"]
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.config["marker_size"]
                marker.scale.y = self.config["marker_size"]
                marker.scale.z = self.config["marker_size"]
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):

        marker = Marker()

        marker.header.frame_id = self.config["frame"]["map"]
        marker.header.stamp = rospy.get_rostime()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.config["marker_z"]

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw


if __name__ == "__main__":
    try:
        neupan_node = neupan_core()
        neupan_node.run()
    except rospy.ROSInterruptException:
        pass
