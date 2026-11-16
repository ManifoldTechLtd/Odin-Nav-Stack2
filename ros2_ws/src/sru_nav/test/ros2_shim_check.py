"""ROS2 validation harness for sru.ros_shim (run on a ROS2 host).

Exercises every shim API the navigation node actually uses, so the ROS2 branch
can be trusted before touching hardware. Needs rclpy + the ROS2 message
packages only (no onnxruntime / cv2 inference).

Usage:
    source /opt/ros/<distro>/setup.bash
    PYTHONPATH=<repo>/ros2_ws/src/sru_nav python3 ros2_shim_check.py
    (the package root holds the `sru` symlink to the core source)
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from sru import ros_shim as ros

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


print(f"shim active version: {ros.active_ros_version()}")

# --- 1. node lifecycle ------------------------------------------------------
node = ros.init_node("sru_nav_node", anonymous=False)
check("init_node returns a node", node is not None, type(node).__name__)

try:
    ros.init_node("again")
    check("init_node is idempotent", True)
except Exception as exc:  # pragma: no cover
    check("init_node is idempotent", False, repr(exc))

# --- 2. parameters ---------------------------------------------------------
check("get_param default", ros.get_param("~min_depth", 0.25) == 0.25)
check("get_param yaml override", ros.get_param("~depth_topic", "/default") == "/from_yaml/depth",
      str(ros.get_param("~depth_topic", "/default")))

# --- 3. time ---------------------------------------------------------------
stamp = ros.Time.from_sec(12.5)
check("Time.from_sec sec/nanosec", stamp.sec == 12 and abs(stamp.nanosec - 500000000) < 1000,
      f"sec={stamp.sec} nanosec={stamp.nanosec}")
check("stamp_to_sec round-trip", abs(ros.stamp_to_sec(stamp) - 12.5) < 1e-6,
      str(ros.stamp_to_sec(stamp)))
now = ros.Time.now()
check("Time.now is a stamp", hasattr(now, "sec") and hasattr(now, "nanosec"))
check("Duration.to_sec", abs(ros.Duration(0.2).to_sec() - 0.2) < 1e-9)

# --- 4. pub/sub round-trip + private-name resolution ----------------------
received = {"twist": None, "priv": None, "latched": None}


def _on_twist(msg):
    received["twist"] = msg


def _on_priv(msg):
    received["priv"] = msg


def _on_latched(msg):
    received["latched"] = msg


pub = ros.Publisher("/cmd_vel", Twist, queue_size=10)
ros.Subscriber("/cmd_vel", Twist, _on_twist, queue_size=10)
priv_pub = ros.Publisher("~base_vel", Twist, queue_size=10)
ros.Subscriber("~base_vel", Twist, _on_priv, queue_size=10)
lat_pub = ros.Publisher("/sru/cmd_vel_enable", Bool, queue_size=1, latch=True)
ros.Subscriber("/sru/cmd_vel_enable", Bool, _on_latched, queue_size=1)

expected_priv = "/" + node.get_name() + "/base_vel"
check("private topic resolves like rospy", priv_pub._pub.topic_name == expected_priv,
      priv_pub._pub.topic_name)
check("absolute topic untouched", pub._pub.topic_name == "/cmd_vel", pub._pub.topic_name)

for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.05)

tw = Twist()
tw.linear.x = 0.42
tw.angular.z = -0.13
pub.publish(tw)
priv_pub.publish(tw)
lat_pub.publish(Bool(data=True))

deadline = time.time() + 5.0
while time.time() < deadline and (received["twist"] is None or received["priv"] is None
                                  or received["latched"] is None):
    rclpy.spin_once(node, timeout_sec=0.05)

check("/cmd_vel round-trip", received["twist"] is not None
      and abs(received["twist"].linear.x - 0.42) < 1e-6)
check("~base_vel round-trip", received["priv"] is not None
      and abs(received["priv"].linear.x - 0.42) < 1e-6)
check("latched publisher delivers", received["latched"] is not None
      and received["latched"].data is True)

# --- 5. timer (period given as shim Duration, like rospy.Timer) -----------
# rospy passes a TimerEvent to the callback; rclpy passes nothing. The shim must
# adapt so node callbacks keep the `def cb(self, event)` signature.
ticks = {"n": 0, "event": "unset"}


def _tick(evt):
    ticks["n"] += 1
    ticks["event"] = evt


ros.Timer(ros.Duration(0.05), _tick)
deadline = time.time() + 3.0
while time.time() < deadline and ticks["n"] < 3:
    rclpy.spin_once(node, timeout_sec=0.05)
check("Timer fires", ticks["n"] >= 3, f"ticks={ticks['n']}")
check("Timer callback receives an event object", ticks["event"] is not None
      and hasattr(ticks["event"], "current_real"),
      type(ticks["event"]).__name__)

# --- 6. logging -----------------------------------------------------------
try:
    ros.loginfo("smoke: loginfo")
    ros.logwarn("smoke: logwarn")
    ros.logerr("smoke: logerr")
    ros.logdebug("smoke: logdebug")
    ros.loginfo_throttle(30.0, "smoke: throttled")
    ros.logwarn_throttle(30.0, "smoke: throttled warn")
    ros.logerr_throttle(30.0, "smoke: throttled err")
    check("logging + throttled logging", True)
except Exception as exc:
    check("logging + throttled logging", False, repr(exc))

# --- 7. shutdown helpers --------------------------------------------------
check("is_shutdown false while running", ros.is_shutdown() is False)
try:
    ros.on_shutdown(lambda: None)
    check("on_shutdown accepts callback", True)
except Exception as exc:
    check("on_shutdown accepts callback", False, repr(exc))
check("ROSInterruptException exists", issubclass(ros.ROSInterruptException, BaseException))

# --- 8. sub.unregister ----------------------------------------------------
try:
    sub = ros.Subscriber("/cmd_vel", Twist, _on_twist, queue_size=1)
    sub.unregister()
    check("Subscriber.unregister", True)
except Exception as exc:
    check("Subscriber.unregister", False, repr(exc))

try:
    ros.sleep(0.01)
    check("sleep", True)
except Exception as exc:
    check("sleep", False, repr(exc))

print(f"\n=== {len(PASS)} passed, {len(FAIL)} failed ===")
if FAIL:
    print("FAILED:", ", ".join(FAIL))
sys.exit(1 if FAIL else 0)
