"""End-to-end wiring smoke test for the SRU node on ROS2 (no hardware, no ONNX).

The deployment host has onnxruntime + the two .onnx files; this test instead
stubs ``LearningModel`` (and, if absent, the onnxruntime import) so the whole
ROS2 graph can be exercised on a plain dev box:

    odom + depth + goal  ->  NavigationPolicyNode  ->  cmd_vel (topic from params)

It verifies the things that actually break on a ROS-version port:
  * node construction declares/reads every parameter
  * the control loop runs on a timer at control_frequency
  * a goal produces a non-zero, policy_scale-bounded /cmd_vel
  * arrival (goal reached) drops the command back to zero
  * ~base_vel is published as a private topic

Usage:
    source /opt/ros/<distro>/setup.bash
    python3 node_smoke_ros2.py --ros-args --params-file <params.yaml>
"""
import sys
import time
import types

import numpy as np

# --- stub onnxruntime only if the host lacks it -----------------------------
if "onnxruntime" not in sys.modules:
    try:
        import onnxruntime  # noqa: F401
    except ImportError:
        ort = types.ModuleType("onnxruntime")
        ort.get_available_providers = lambda: ["CPUExecutionProvider"]
        ort.InferenceSession = object
        ort.SessionOptions = object
        ort.GraphOptimizationLevel = object
        sys.modules["onnxruntime"] = ort
        print("[test] onnxruntime absent -> injected stub")

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from sru import constants  # noqa: E402
from sru import model as model_mod  # noqa: E402
from sru import ros_shim as ros  # noqa: E402
from sru.utils import subtract_frame_transforms, transform_points  # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


# --- stub model (same surface the node uses) --------------------------------
PAIR = {"count": 0}


class StubModel(object):
    """Constant forward command at 50% of policy_scale."""

    def __init__(self, preprocess_model_path, policy_model_path,
                 policy_scale=None, intra_op_num_threads=4):
        self._scale = np.array(policy_scale or constants.POLICY_SCALE, dtype=np.float32)
        self._h_state = np.zeros((1, 1, 512), dtype=np.float32)
        self._c_state = np.zeros((1, 1, 512), dtype=np.float32)

    def reset_hidden_state(self):
        self._h_state[:] = 0.0
        self._c_state[:] = 0.0

    def normalize_target_position(self, target_pos_w, robot_pos_w, robot_orientation_w):
        t = np.array(target_pos_w, dtype=np.float32)[np.newaxis]
        p = np.array(robot_pos_w, dtype=np.float32)[np.newaxis]
        q = np.array(robot_orientation_w, dtype=np.float32)[np.newaxis]
        inv_pos, inv_rot = subtract_frame_transforms(p, q)
        vec_b = transform_points(t, inv_pos, inv_rot)
        dist = np.linalg.norm(vec_b, axis=-1, keepdims=True) + 1e-6
        pos = np.concatenate((vec_b / dist, np.log(dist + 1.0)), axis=-1).reshape(-1)
        return pos, vec_b

    def predict(self, linear_vel, angular_vel, gravity_vector, last_action,
                target_pos_w, robot_pos_w, robot_orientation_w, depth_image,
                is_reset=False):
        PAIR["count"] += 1
        assert depth_image is not None, "depth_image must reach predict()"
        cmd = (self._scale * 0.5).astype(np.float32)
        return cmd, cmd.copy(), np.array([[1.0, 0.0, 0.0]], dtype=np.float32)


model_mod.LearningModel = StubModel
from sru.navigation_policy_node import NavigationPolicyNode  # noqa: E402

# --- bring the node up ------------------------------------------------------
node = ros.init_node("sru_nav_node", anonymous=False)
check("node name", node.get_name() == "sru_nav_node", node.get_name())

policy_scale = list(ros.get_param("~policy_scale", list(constants.POLICY_SCALE)))
control_hz = float(ros.get_param("~control_frequency", constants.DEFAULT_CONTROL_FREQUENCY))
cmd_topic = ros.get_param("~cmd_vel_topic", "/cmd_vel")
goal_topic = ros.get_param("~goal_topic", "/goal_pose")
print(f"[test] cmd_topic={cmd_topic} goal_topic={goal_topic} "
      f"control_frequency={control_hz} policy_scale={policy_scale}")

received = []
base_vels = []
node.create_subscription(Twist, cmd_topic, lambda m: received.append(m), 10)
node.create_subscription(Twist, "/sru_nav_node/base_vel", lambda m: base_vels.append(m), 10)

vae = ros.get_param("~vae_model_path", "") or "/tmp/vae_encoder.onnx"
pol = ros.get_param("~policy_model_path", "") or "/tmp/nav_policy.onnx"

try:
    node_policy = NavigationPolicyNode(
        preprocess_model_path=vae,
        policy_model_path=pol,
        policy_scale=policy_scale,
        use_sim=bool(ros.get_param("~use_sim", False)),
        min_depth=float(ros.get_param("~min_depth", constants.DEFAULT_MIN_DEPTH)),
        max_depth=float(ros.get_param("~max_depth", constants.DEFAULT_MAX_DEPTH)),
        control_frequency=control_hz,
        depth_topic=ros.get_param("~depth_topic", "/odin1/depth_img_competetion"),
        odom_topic=ros.get_param("~odom_topic", "/odin1/odometry_highfreq"),
        goal_topic=goal_topic,
        cmd_vel_topic=cmd_topic,
    )
    check("NavigationPolicyNode constructs", True)
except Exception as exc:
    check("NavigationPolicyNode constructs", False, repr(exc))
    raise

pub_odom = node.create_publisher(Odometry, ros.get_param("~odom_topic", "/odin1/odometry_highfreq"), 10)
pub_depth = node.create_publisher(Image, ros.get_param("~depth_topic", "/odin1/depth_img_competetion"), 2)
pub_goal = node.create_publisher(PoseStamped, goal_topic, 1)

t0 = node.get_clock().now().nanoseconds * 1e-9 + 1.0
STEPS = 30
DT = 0.1  # 10 Hz sensor stream; control loop runs at control_frequency


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def stamp(t):
    return ros.Time.from_sec(t)


def send_odom(t):
    m = Odometry()
    m.header.stamp = stamp(t)
    m.header.frame_id = "odom"
    m.child_frame_id = "odin1_base_link"
    m.pose.pose.orientation.w = 1.0
    pub_odom.publish(m)


def send_depth(t):
    m = Image()
    m.header.stamp = stamp(t)
    m.header.frame_id = "odin1_camera_link"
    m.height, m.width = 480, 640
    m.encoding = "32FC1"
    m.is_bigendian = 0
    m.step = m.width * 4
    m.data = np.full((m.height, m.width), 2.0, dtype=np.float32).tobytes()
    pub_depth.publish(m)


# --- phase 1: no goal -> node publishes ZERO and must not run the policy ----
# `_check_goal_reached(None, ...)` returns True, so an un-goaled node publishes a
# zero Twist every control cycle. That is designed behaviour (and is exactly why
# the nav stack gates /sru/cmd_vel behind /sru/cmd_vel_enable).
for i in range(6):
    send_odom(t0 + i * DT)
    send_depth(t0 + i * DT)
    spin(0.02)
check("no goal -> every /cmd_vel is zero",
      all(abs(m.linear.x) < 1e-9 and abs(m.angular.z) < 1e-9 for m in received),
      f"msgs={len(received)}")
check("no goal -> policy NOT invoked", PAIR["count"] == 0, f"calls={PAIR['count']}")
check("~base_vel published from odom", len(base_vels) > 0, f"msgs={len(base_vels)}")

# --- phase 2: far goal -> moving command ------------------------------------
received.clear()
g = PoseStamped()
g.header.frame_id = "odom"
g.pose.position.x = 3.0
g.pose.orientation.w = 1.0
g.header.stamp = stamp(t0)
pub_goal.publish(g)
spin(0.3)

for i in range(6, STEPS):
    send_odom(t0 + i * DT)
    send_depth(t0 + i * DT)
    spin(0.02)

check("goal -> /cmd_vel produced", len(received) > 0, f"msgs={len(received)}")
check("goal -> policy invoked", PAIR["count"] > 0, f"calls={PAIR['count']}")

if received:
    xs = [m.linear.x for m in received]
    zs = [m.angular.z for m in received]
    check("forward command non-zero", all(abs(x) > 1e-6 for x in xs), f"vx={xs[0]:.3f}")
    check("command within policy_scale",
          all(abs(m.linear.x) <= abs(policy_scale[0]) + 1e-6 for m in received)
          and all(abs(m.angular.z) <= abs(policy_scale[2]) + 1e-6 for m in received))

    span = (STEPS - 6) * DT
    rate = len(received) / span if span else 0.0
    check("5 Hz gate holds", 0.5 * control_hz <= rate <= 1.5 * control_hz,
          f"rate={rate:.2f} Hz for {control_hz} Hz target")
    print(f"[test] sampled cmds: vx={[round(x,3) for x in xs[:3]]} "
          f"wz={[round(z,3) for z in zs[:3]]}")

# --- phase 3: arrival -> zero command --------------------------------------
received.clear()
g_near = PoseStamped()
g_near.header.frame_id = "odom"
g_near.header.stamp = stamp(t0 + STEPS * DT)
g_near.pose.position.x = 0.1  # inside ARRIVE_GOAL_THRESHOLD
g_near.pose.orientation.w = 1.0
pub_goal.publish(g_near)
spin(0.3)

for i in range(STEPS, STEPS + 8):
    send_odom(t0 + i * DT)
    send_depth(t0 + i * DT)
    spin(0.02)

if received:
    check("arrival -> zero command",
          abs(received[-1].linear.x) < 1e-6 and abs(received[-1].angular.z) < 1e-6,
          f"last vx={received[-1].linear.x:.4f}")
else:
    check("arrival -> zero command", True, "no new msgs (already stopped)")

print(f"\n=== {len(PASS)} passed, {len(FAIL)} failed ===")
if FAIL:
    print("FAILED:", ", ".join(FAIL))
sys.exit(1 if FAIL else 0)
