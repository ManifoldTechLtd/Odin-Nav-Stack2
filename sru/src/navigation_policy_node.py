"""Navigation policy node for the Odin1 depth sensor (ROS1 and ROS2).

The node consumes depth + odometry + a goal pose and publishes body-frame
velocity commands at a fixed cadence. ROS API differences are absorbed by
sru.ros_shim so a single source tree serves both ROS versions.
"""

import time

import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from sru import constants
from sru import ros_shim as ros
from sru.model import LearningModel
from sru.visualization import VisualizationManager


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _stamp_to_sec(stamp):
    """Message header stamp -> float seconds (layout handled by the shim)."""
    return ros.stamp_to_sec(stamp)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class NavigationPolicyNode(object):
    """RL navigation controller node (ROS version agnostic)."""

    def __init__(self,
                 preprocess_model_path,
                 policy_model_path,
                 depth_topic,
                 odom_topic,
                 goal_topic,
                 cmd_vel_topic,
                 min_depth=constants.DEFAULT_MIN_DEPTH,
                 max_depth=constants.DEFAULT_MAX_DEPTH,
                 control_frequency=constants.DEFAULT_CONTROL_FREQUENCY,
                 policy_scale=None,
                 use_sim=False,
                 depth_stale_sec=constants.DEPTH_STALE_SEC):
        # Configuration
        self.use_sim = use_sim
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.control_frequency = float(control_frequency)
        self.depth_stale_sec = float(depth_stale_sec)
        self.odom_ready = False
        self.arrive_goal_threshold = constants.ARRIVE_GOAL_THRESHOLD

        # ----- Publishers ------------------------------------------------
        self.cmd_vel_publisher = ros.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.base_vel_publisher = ros.Publisher(
            '~base_vel', Twist, queue_size=10)

        # Visualization
        self.twist_marker_publisher = ros.Publisher(
            '~vis/twist_cmd_marker', Marker, queue_size=10)
        self.goal_vector_marker_publisher = ros.Publisher(
            '~vis/goal_vector_marker', Marker, queue_size=10)

        # ----- Utilities --------------------------------------------------
        self.bridge = CvBridge()
        self.model = LearningModel(
            preprocess_model_path=preprocess_model_path,
            policy_model_path=policy_model_path,
            policy_scale=policy_scale,
        )
        self.visualization_manager = VisualizationManager()

        # ----- Robot state -----------------------------------------------
        self.map_frame_id = None
        self.robot_frame_id = None
        self.robot_odom_time = 0.0
        self.robot_pos_w = None
        self.robot_orientation_w = None
        self.linear_vel_w = None
        self.angular_vel_w = None
        self.linear_vel = None
        self.angular_vel = None
        self.gravity_vector = None
        self.depth_image = None
        self.depth_rx_time = 0.0        # wall clock of the newest depth frame

        # ----- Navigation state ------------------------------------------
        self.target_pos_w = None
        self.last_target_pos = None
        self.last_action = self._reset_last_action()
        self.prev_cmd = np.zeros(3)

        # ----- Subscribers (created LAST so callbacks fire only after init)
        self.odom_subscriber = ros.Subscriber(
            odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.depth_subscriber = ros.Subscriber(
            depth_topic, Image, self.depth_callback, queue_size=2)
        self.target_position_subscriber = ros.Subscriber(
            goal_topic, PoseStamped, self.target_position_callback, queue_size=1)

        # ----- Timers ----------------------------------------------------
        # Control loop is TIMER-driven at control_frequency and consumes the
        # newest depth frame, decoupling the output cadence from depth-arrival
        # jitter. Publishing cmd_vel only inside depth_callback would let depth
        # sync jitter / processing spikes produce >0.5 s gaps, making the
        # chassis bridge time out and inject zero-velocity jerks.
        ros.Timer(ros.Duration(1.0 / self.control_frequency),
                    self._timer_control)
        ros.Timer(ros.Duration(constants.TARGET_VECTOR_PUBLISH_INTERVAL),
                    self._timer_publish_target_vector)

        ros.loginfo('\033[92mNavigation policy node is ready.\033[0m')

    # =====================================================================
    # Callbacks
    # =====================================================================
    def odom_callback(self, odom_msg):
        # 1) odom timestamp
        self.robot_odom_time = _stamp_to_sec(odom_msg.header.stamp)

        # 2) Frame IDs (once)
        if self.map_frame_id is None:
            self.map_frame_id = odom_msg.header.frame_id
        if self.robot_frame_id is None:
            self.robot_frame_id = odom_msg.child_frame_id

        # 3) Robot pose
        self.robot_pos_w = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
        ]
        self.robot_orientation_w = [
            odom_msg.pose.pose.orientation.w,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
        ]

        # 4) Velocities from odom
        self.linear_vel_w = [
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z,
        ]
        self.angular_vel_w = [
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z,
        ]

        # 5) Convert to base frame (real-hw) or pass-through (sim)
        if self.use_sim:
            self.linear_vel = list(self.linear_vel_w)
            self.angular_vel = list(self.angular_vel_w)
        else:
            self.linear_vel = self.convert_vel_frame(
                self.linear_vel_w, self.robot_orientation_w)
            self.angular_vel = self.convert_vel_frame(
                self.angular_vel_w, self.robot_orientation_w)

        # 6) Publish converted base velocity for debug
        self.publish_base_vel(self.linear_vel, self.angular_vel)

        # 7) Projected gravity in base frame
        self.gravity_vector = self.projected_gravity_vector(self.robot_orientation_w)

        # 8) Mark odom available
        if not self.odom_ready:
            self.odom_ready = True

    def depth_callback(self, depth_msg):
        if not self.odom_ready:
            ros.logwarn_throttle(
                5.0, '\033[93mOdometry not ready, skipping depth callback.\033[0m')
            return

        try:
            depth_array = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_array = np.nan_to_num(
                depth_array, nan=0.0, posinf=self.max_depth * 2.0, neginf=0.0)
            depth_array = depth_array.astype(np.float32, copy=False)
            depth_array[depth_array > self.max_depth] = 0.0
            depth_array[depth_array < self.min_depth] = 0.0
            self.depth_image = depth_array
        except Exception as e:
            ros.logerr('Error converting depth image: {}'.format(e))
            return

        self.depth_rx_time = time.time()

    def _timer_control(self, _event):
        """Fixed-cadence control tick (see the timer comment in __init__)."""
        if not self.odom_ready:
            return
        # Stale/missing depth -> explicit zero. This is safer than holding the
        # previous command and faster than letting the bridge time out.
        if self.depth_image is None or \
                (time.time() - self.depth_rx_time) > self.depth_stale_sec:
            ros.logwarn_throttle(
                2.0, '\033[93mDepth image stale or missing; '
                     'publishing zero cmd_vel.\033[0m')
            self.cmd_vel_publisher.publish(Twist())
            return
        self.generate_cmd_vel()

    def generate_cmd_vel(self):
        is_arrived = self._check_goal_reached(self.target_pos_w, self.robot_pos_w)
        if is_arrived:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.target_pos_w = None
            self.last_action = self._reset_last_action()
            # Idle (no goal) also lands here every cycle; throttle to avoid
            # flooding the log at 5 Hz.
            ros.loginfo_throttle(5.0, 'Target position reset.')

        else:
            cmd, action, _target_vec_b = self.model.predict(
                self.linear_vel, self.angular_vel, self.gravity_vector,
                self.last_action, self.target_pos_w, self.robot_pos_w,
                self.robot_orientation_w, self.depth_image,
                False,
            )

            self.last_action = action.tolist()

            twist = Twist()
            model_cmd = np.array([
                cmd[0].item(),
                cmd[1].item() * constants.LATERAL_VELOCITY_SCALE,
                cmd[2].item(),
            ])
            filter_coef = np.array(constants.LOW_PASS_FILTER_COEF)
            filt_model = filter_coef * model_cmd + (1 - filter_coef) * self.prev_cmd
            twist.linear.x = float(filt_model[0])
            twist.linear.y = float(filt_model[1])
            twist.angular.z = float(filt_model[2])
            self.prev_cmd = filt_model
            self.cmd_vel_publisher.publish(twist)

            self.visualization_manager.publish_twist_marker(
                twist, self.robot_odom_time, self.robot_frame_id,
                self.twist_marker_publisher,
            )

        ros.loginfo_throttle(
            1.0,
            'Published cmd_vel: linear_x={:.4f}, linear_y={:.4f}, angular_z={:.4f}'.format(
                twist.linear.x, twist.linear.y, twist.angular.z
            ),
        )

    # =====================================================================
    # Velocity / orientation helpers
    # =====================================================================
    def publish_base_vel(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel[0]
        twist.linear.y = linear_vel[1]
        twist.angular.z = angular_vel[2]
        self.base_vel_publisher.publish(twist)

    @staticmethod
    def convert_vel_frame(vel_vec, orientation_w):
        # quaternion is (w, x, y, z); scipy expects (x, y, z, w)
        quat_xyzw = [orientation_w[1], orientation_w[2], orientation_w[3], orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()
        return inv_rot.apply(np.array(vel_vec)).tolist()

    @staticmethod
    def projected_gravity_vector(robot_orientation_w):
        quat_xyzw = [robot_orientation_w[1], robot_orientation_w[2],
                     robot_orientation_w[3], robot_orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()
        gravity = np.array([0.0, 0.0, -constants.GRAVITY_MAGNITUDE])
        proj = inv_rot.apply(gravity)
        proj = proj / (np.linalg.norm(proj) + 1e-6)
        return proj.tolist()

    # =====================================================================
    # Timer callbacks
    # =====================================================================
    def _timer_publish_target_vector(self, _event):
        self.publish_target_vector()

    def publish_target_vector(self):
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            return
        tgt = self.target_pos_w if self.target_pos_w is not None else self.last_target_pos
        if tgt is None:
            return

        _, target_vec_b = self.model.normalize_target_position(
            tgt, self.robot_pos_w, self.robot_orientation_w
        )

        self.visualization_manager.publish_target_vector_marker(
            target_vec_b, self.robot_odom_time, self.robot_frame_id,
            self.goal_vector_marker_publisher,
        )

    # =====================================================================
    # Goal callbacks & checks
    # =====================================================================
    def _check_goal_reached(self, target_pos_w, robot_pos_w):
        if target_pos_w is None or robot_pos_w is None:
            return True
        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        if dist > self.arrive_goal_threshold:
            return False
        ros.loginfo('Arrived at the goal position.')
        return True

    def target_position_callback(self, msg):
        # If the goal arrives before odom, skip frame check.
        if self.map_frame_id is not None and msg.header.frame_id and \
                msg.header.frame_id != self.map_frame_id:
            ros.logerr(
                '\033[91mTarget frame_id "{}" does not match odometry frame_id "{}"\033[0m'
                .format(msg.header.frame_id, self.map_frame_id)
            )
            return

        goal_z = msg.pose.position.z
        if abs(goal_z) < 1e-3:
            if self.robot_pos_w is not None:
                goal_z = self.robot_pos_w[2]
            else:
                ros.logwarn('Robot position not available, using received z for target.')

        self.target_pos_w = [msg.pose.position.x, msg.pose.position.y, goal_z]
        self.last_target_pos = list(self.target_pos_w)

        ros.loginfo('Received target position: {}'.format(self.target_pos_w))

    # =====================================================================
    # Misc reset helpers
    # =====================================================================
    def _reset_last_action(self):
        self.prev_cmd = np.zeros(3)
        return [0.0, 0.0, 0.0]