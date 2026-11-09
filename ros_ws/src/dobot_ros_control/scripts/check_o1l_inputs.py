#!/usr/bin/env python3
"""Sensor-only startup gate. --reset-pose resets Odin, never the robot FSM."""
import argparse
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from o1l_input_guard import NavigationInputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reset-pose", action="store_true")
    parser.add_argument("--prefix", default="odin1")
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    rospy.init_node("o1l_input_gate", anonymous=True, disable_signals=True)
    if args.reset_pose:
        from odin_ros_driver_rev1.srv import ResetInitialPose
        service = "/{}/device0/reset_initial_pose".format(args.prefix)
        rospy.wait_for_service(service, timeout=5)
        result = []

        def reset():
            try:
                result.append(rospy.ServiceProxy(service, ResetInitialPose)())
            except Exception as exc:
                result.append(exc)

        worker = threading.Thread(target=reset, daemon=True)
        worker.start()
        worker.join(8)
        if not result or not getattr(result[0], "success", False):
            print("o1l-reset FAILED: {}".format(result), flush=True)
            return 1
        print("o1l-reset: sensor pose reset acknowledged; navigation not started", flush=True)
        # No subscriptions yet: discard pre-reset queued observations.
        time.sleep(2)
        return 0

    guard = NavigationInputs()
    odom_sub = rospy.Subscriber("/{}/odometry".format(args.prefix),
                                Odometry, guard.update_odom, queue_size=1)
    scan_sub = rospy.Subscriber("/scan_360", LaserScan, guard.update_scan, queue_size=1)
    deadline = time.monotonic() + args.timeout
    good_since = None
    reason = "WAITING"
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        reason = guard.status(rospy.Time.now().to_sec())
        if reason == "OK":
            if good_since is None:
                good_since = time.monotonic()
            if time.monotonic() - good_since >= 3:
                print("o1l-input-ready: plausible odometry + fresh nonempty scan for 3s", flush=True)
                return 0
        else:
            good_since = None
        time.sleep(0.1)
    print("o1l-input FAILED: {}".format(reason), flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
