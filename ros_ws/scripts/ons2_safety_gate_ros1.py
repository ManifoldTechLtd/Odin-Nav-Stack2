#!/usr/bin/env python3
"""ROS 1 SafetyGate daemon: cancels mission and publishes 20 Hz zero /cmd_vel on odom/scan/TF loss."""
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

STALE_SEC = 3.0
ZERO_HZ = 20.0
GRACE_SEC = float(os.environ.get("ONS2_SAFETY_STARTUP_GRACE_SEC", "10.0"))


class SafetyGateRos1:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.t0 = time.time()
        self.odom_t = None
        self.odom_nan = False
        self.scan_t = None
        self.scan_pts = 0
        self.active = False
        self.trips = 0
        self._stop_evt = threading.Event()

        self.tf_buf = Buffer()
        TransformListener(self.tf_buf)
        self.cmd_pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
        self.stop_pub = rospy.Publisher(
            rospy.get_param("/mission_manager/stop_topic", rospy.get_param("~stop_topic", "/neupan/stop")),
            Empty, queue_size=1,
        )
        self.enable_pub = rospy.Publisher(
            rospy.get_param("/mission_manager/enable_topic", rospy.get_param("~enable_topic", "/sru/cmd_vel_enable")),
            Bool, queue_size=1, latch=True,
        )
        self.cancel_srv = rospy.ServiceProxy("/waypoint_manager/cancel", Trigger)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odin1/odometry"), Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self._scan_cb, queue_size=1)

    def _odom_cb(self, msg):
        self.odom_t = time.time()
        p = msg.pose.pose.position
        self.odom_nan = not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z))

    def _scan_cb(self, msg):
        self.scan_t = time.time()
        self.scan_pts = sum(1 for r in msg.ranges if 0.05 < r < 30.0)

    def check_danger(self):
        now = time.time()
        grace = (now - self.t0) < GRACE_SEC
        reasons = []
        if self.odom_t is None:
            if not grace:
                reasons.append("odom_no_data")
        elif (now - self.odom_t) > STALE_SEC or self.odom_nan:
            reasons.append("odom_nan" if self.odom_nan else "odom_stale")

        if self.scan_t is None:
            if not grace:
                reasons.append("scan_no_data")
        elif (now - self.scan_t) > STALE_SEC or self.scan_pts < 5:
            reasons.append("scan_stale" if (now - self.scan_t) > STALE_SEC else "scan_few_points")

        try:
            tr = self.tf_buf.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0))
            if tr.header.stamp.to_sec() > 0 and abs(rospy.Time.now().to_sec() - tr.header.stamp.to_sec()) > STALE_SEC:
                reasons.append("tf_stale")
        except TransformException:
            if not grace:
                reasons.append("tf_lost")
        return reasons

    def trip(self, reasons):
        if self.active:
            return
        self.active = True
        self.trips += 1
        rospy.logerr("[SafetyGate] Stream loss (%s) -> cancel mission + 20Hz zero-vel brake (#%d)", ",".join(reasons), self.trips)
        self.stop_pub.publish(Empty())
        self.enable_pub.publish(Bool(data=False))
        try:
            self.cancel_srv()
        except rospy.ROSException:
            pass
        self._stop_evt.clear()
        threading.Thread(target=self._zero_loop, daemon=True).start()

    def release(self):
        if self.active:
            self.active = False
            self._stop_evt.set()
            rospy.logwarn("[SafetyGate] Streams recovered; releasing brake (trips=%d)", self.trips)

    def _zero_loop(self):
        zero = Twist()
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            self.cmd_pub.publish(zero)
            time.sleep(1.0 / ZERO_HZ)

    def spin(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            danger = self.check_danger()
            self.trip(danger) if danger else self.release()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ons2_safety_gate", anonymous=True)
    SafetyGateRos1().spin()
