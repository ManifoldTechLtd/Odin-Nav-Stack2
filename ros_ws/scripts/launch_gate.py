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

"""Readiness gate helper for navigation stack startup scripts.

Usage:
  launch_gate.py --wait-topic /odin1/odometry --timeout 60
  launch_gate.py --wait-tf odom map --timeout 120
  launch_gate.py --wait-tf map base_link --timeout 30 --soft

Exit codes: 0=condition met, 1=timeout, 2=invalid arguments.
"""
import argparse
import sys
import time

import rospy


def wait_topic(topic, msg_type, timeout, soft):
    """Wait until a topic publishes a fresh message (within 3s) before timeout."""
    last = {"t": None}

    def cb(_msg):
        last["t"] = time.time()

    rospy.Subscriber(topic, msg_type, cb, queue_size=1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if last["t"] is not None and time.time() - last["t"] <= 3.0:
            print("topic-ready: {} has fresh data".format(topic), flush=True)
            return 0
        time.sleep(0.5)
    print("topic-timeout: {} no data within {}s".format(topic, timeout),
          file=sys.stderr, flush=True)
    return 0 if soft else 1


def wait_tf(parent, child, timeout, soft):
    """Wait until TF transform parent->child becomes available."""
    import tf2_ros
    from tf2_ros import (ConnectivityException, ExtrapolationException,
                         LookupException)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            buf.lookup_transform(parent, child, rospy.Time(0))
            print("tf-ready: {} -> {}".format(parent, child), flush=True)
            return 0
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            pass
        time.sleep(0.5)
    print("tf-timeout: {} -> {} did not appear within {}s".format(parent, child, timeout),
          file=sys.stderr, flush=True)
    return 0 if soft else 1


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--wait-topic", metavar="TOPIC",
                   help="Wait for fresh messages on topic")
    g.add_argument("--wait-tf", nargs=2, metavar=("PARENT", "CHILD"),
                   help="Wait for TF transform parent->child")
    ap.add_argument("--timeout", type=float, required=True)
    ap.add_argument("--soft", action="store_true",
                    help="Return 0 on timeout with warning only")
    args = ap.parse_args()

    rospy.init_node("launch_gate", anonymous=True, disable_signals=True)

    if args.wait_topic:
        from nav_msgs.msg import Odometry
        return wait_topic(args.wait_topic, Odometry, args.timeout, args.soft)
    parent, child = args.wait_tf
    return wait_tf(parent, child, args.timeout, args.soft)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
