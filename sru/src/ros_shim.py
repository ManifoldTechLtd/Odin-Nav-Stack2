"""ROS version shim for the SRU navigation controller.

Single entry point for every ROS API this package needs, so the same node
source runs on both ROS1 (rospy / Noetic) and ROS2 (rclpy / Jazzy).

Platform selection
------------------
``ROS_VERSION`` environment variable (set by the ROS setup files: ``1`` for
ROS1, ``2`` for ROS2), with an import-based fallback for shells that were not
sourced.

Usage inside this package::

    from sru import ros_shim as ros

    pub = ros.Publisher('/cmd_vel', Twist, queue_size=10)
    ros.loginfo('...')
    goal.header.stamp = ros.Time.from_sec(t)

The ROS1 branch is a thin pass-through to ``rospy`` (identical behaviour to
the previous direct-rospy source). The ROS2 branch implements the same
signatures on top of ``rclpy`` and has been validated on the SW01 deployment
host (NUC + ROS2 Jazzy).

Deliberately out of scope of this shim (import paths are identical on both
versions, so call sites keep importing them directly):
``geometry_msgs`` / ``nav_msgs`` / ``sensor_msgs`` / ``std_msgs`` /
``visualization_msgs`` messages and ``cv_bridge``.
"""

import os
import sys
import time as _time

_ROS_VERSION = os.environ.get("ROS_VERSION", "").strip()
if _ROS_VERSION == "1":
    _MODE = "ros1"
elif _ROS_VERSION == "2":
    _MODE = "ros2"
else:  # un-sourced shell: probe by import
    try:
        import rclpy  # noqa: F401

        _MODE = "ros2"
    except ImportError:
        _MODE = "ros1"


if _MODE == "ros1":
    # ------------------------------------------------------------------
    # ROS1 branch: pass-through to rospy
    # ------------------------------------------------------------------
    import rospy

    ROSInterruptException = rospy.ROSInterruptException
    Duration = rospy.Duration
    Time = rospy.Time
    Publisher = rospy.Publisher
    Subscriber = rospy.Subscriber
    Timer = rospy.Timer

    init_node = rospy.init_node
    spin = rospy.spin
    is_shutdown = rospy.is_shutdown
    on_shutdown = rospy.on_shutdown
    sleep = rospy.sleep
    get_param = rospy.get_param

    loginfo = rospy.loginfo
    logwarn = rospy.logwarn
    logerr = rospy.logerr
    logdebug = rospy.logdebug
    loginfo_throttle = rospy.loginfo_throttle
    logwarn_throttle = rospy.logwarn_throttle
    logerr_throttle = rospy.logerr_throttle

    def stamp_to_sec(stamp):
        """Message header stamp -> float seconds."""
        return stamp.secs + stamp.nsecs * 1e-9

    def get_package_path(pkg_name):
        """Package directory via rospkg; ``None`` when the package is unknown."""
        try:
            import rospkg

            return rospkg.RosPack().get_path(pkg_name)
        except Exception:
            return None

else:
    # ------------------------------------------------------------------
    # ROS2 branch: rclpy implementation with rospy-like signatures
    # ------------------------------------------------------------------
    import rclpy
    from builtin_interfaces.msg import Time as _TimeMsg
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy

    class ROSInterruptException(KeyboardInterrupt):
        """rclpy has no equivalent; keep the rospy name for except clauses."""

    _node = None
    # rclpy timers are cancelled when garbage collected; the original code
    # drops the return value of rospy.Timer(...), so keep references here.
    _timers = []

    def init_node(name, anonymous=False):
        """Create the shim's node. ``anonymous`` has no ROS2 counterpart."""
        global _node
        if _node is not None:
            return _node
        if not rclpy.ok():
            rclpy.init()
        _node = rclpy.create_node(name)
        return _node

    def _ensure_node():
        if _node is None:
            init_node("sru_nav_node")
        return _node

    def _resolve_topic(topic):
        """'~foo' (rospy private) -> '/<node>/foo' (ROS2 wants '~/foo')."""
        if topic.startswith("~"):
            return "/" + _ensure_node().get_name() + "/" + topic.lstrip("~/")
        return topic

    class Duration:
        def __init__(self, sec=0.0):
            self._sec = float(sec)

        def to_sec(self):
            return self._sec

    class Time:
        """Namespace object mirroring ``rospy.Time`` static helpers."""

        @staticmethod
        def from_sec(sec):
            sec = float(sec)
            whole = int(sec)
            return _TimeMsg(sec=whole, nanosec=int(round((sec - whole) * 1e9)))

        @staticmethod
        def now():
            return _ensure_node().get_clock().now().to_msg()

    class Publisher:
        def __init__(self, topic, msg_type, queue_size=10, latch=False):
            qos = QoSProfile(depth=max(1, int(queue_size)))
            if latch:
                qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self._pub = _ensure_node().create_publisher(
                msg_type, _resolve_topic(topic), qos)

        def publish(self, msg):
            self._pub.publish(msg)

        def get_num_connections(self):
            return self._pub.get_subscription_count()

    class Subscriber:
        def __init__(self, topic, msg_type, callback, queue_size=None):
            qos = QoSProfile(depth=max(1, int(queue_size or 10)))
            self._sub = _ensure_node().create_subscription(
                msg_type, _resolve_topic(topic), callback, qos)

        def unregister(self):
            _ensure_node().destroy_subscription(self._sub)

    class TimerEvent:
        """Stand-in for ``rospy.timer.TimerEvent``.

        rclpy invokes timer callbacks with no arguments while rospy passes a
        TimerEvent; the ROS2 ``Timer`` below adapts by synthesising one from the
        node clock, so callbacks keep the ``def cb(self, event)`` signature.
        """

        def __init__(self):
            now = _ensure_node().get_clock().now().to_msg()
            self.current_real = now
            self.current_expected = now
            self.last_real = now
            self.last_expected = now

    class Timer:
        def __init__(self, period, callback):
            # rclpy expects the period in seconds (float), not a Duration object.
            def _adapted():
                callback(TimerEvent())

            timer = _ensure_node().create_timer(period.to_sec(), _adapted)
            _timers.append(timer)

        def shutdown(self):
            pass  # retained reference is released with the node

    def _param_name(name):
        return name.lstrip("~")

    def get_param(name, default=None):
        node = _ensure_node()
        clean = _param_name(name)
        if not node.has_parameter(clean):
            node.declare_parameter(clean, default if default is not None else "")
        return node.get_parameter(clean).value

    def spin():
        rclpy.spin(_ensure_node())

    def is_shutdown():
        return not rclpy.ok()

    def on_shutdown(callback):
        try:
            rclpy.get_default_context().on_shutdown(callback)
        except Exception:  # pragma: no cover - context API differences
            pass

    def sleep(sec):
        _time.sleep(float(sec))

    def loginfo(msg):
        _ensure_node().get_logger().info(str(msg))

    def logwarn(msg):
        _ensure_node().get_logger().warn(str(msg))

    def logerr(msg):
        _ensure_node().get_logger().error(str(msg))

    def logdebug(msg):
        _ensure_node().get_logger().debug(str(msg))

    # rclpy caches throttle parameters per call site (file+line). All calls go
    # through this one wrapper, so mixing periods (e.g. 1.0 and 5.0) would raise
    # "Logging filter parameters cannot be changed between calls". Throttle by
    # the caller's file+line instead; the ROS1 branch uses rospy's native one.
    _THROTTLE_STATE = {}

    def _throttle_allows(period, key):
        now = _time.monotonic()
        if now - _THROTTLE_STATE.get(key, 0.0) >= float(period):
            _THROTTLE_STATE[key] = now
            return True
        return False

    def _log_site():
        frame = sys._getframe(2)  # 0=_log_site, 1=throttle wrapper, 2=caller
        return (frame.f_code.co_filename, frame.f_lineno)

    def loginfo_throttle(period, msg):
        if _throttle_allows(period, _log_site()):
            _ensure_node().get_logger().info(str(msg))

    def logwarn_throttle(period, msg):
        if _throttle_allows(period, _log_site()):
            _ensure_node().get_logger().warn(str(msg))

    def logerr_throttle(period, msg):
        if _throttle_allows(period, _log_site()):
            _ensure_node().get_logger().error(str(msg))

    def stamp_to_sec(stamp):
        """Message header stamp -> float seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    def get_package_path(pkg_name):
        """Package share directory via the ament index; ``None`` when unknown."""
        try:
            from ament_index_python.packages import get_package_share_directory

            return get_package_share_directory(pkg_name)
        except Exception:
            return None


def active_ros_version():
    """Return 'ros1' or 'ros2' for the active branch (diagnostics/logging)."""
    return _MODE
