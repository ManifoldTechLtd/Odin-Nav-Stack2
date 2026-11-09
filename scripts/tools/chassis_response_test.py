#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

"""Chassis step-response calibration tool.

Modes:
  - direct: 50Hz step commands directly to chassis bridge across (levels x repeats).
  - ab:     Single-level A/B comparison between direct 50Hz step and rate-limited NeuPAN-style step.
  - neupan: Observation-only mode (does not publish /cmd_vel) while NeuPAN drives the robot.

Outputs (--output-dir/<run_id>/):
  meta.json, phases.csv, cmd_intent.csv, cmd_sent.csv, odom_std.csv, odom_hf.csv,
  imu.csv, bridge_trace.jsonl
"""

import argparse
import json
import os
import sys
import time

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
except ImportError:
    print("[FATAL] ROS1 environment (rospy) is required. Run: "
          "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash", file=sys.stderr)
    sys.exit(1)


class StreamBuffer(object):
    """Dual-timestamp stream buffer (header_stamp + host receive time)."""

    __slots__ = ("name", "header", "recv", "v1", "v2", "v3", "fields")

    def __init__(self, name, fields):
        self.name = name
        self.header = []
        self.recv = []
        self.v1 = []
        self.v2 = []
        self.v3 = []
        self.fields = fields

    def add(self, header_t, recv_t, v1=None, v2=None, v3=None):
        self.header.append(float(header_t))
        self.recv.append(float(recv_t))
        self.v1.append(None if v1 is None else float(v1))
        self.v2.append(None if v2 is None else float(v2))
        self.v3.append(None if v3 is None else float(v3))

    def save_csv(self, out_dir):
        path = os.path.join(out_dir, self.name + ".csv")
        with open(path, "w") as f:
            f.write("header_stamp,recv_stamp,%s,%s,%s\n" % tuple(self.fields))
            for i in range(len(self.header)):
                f.write("%.6f,%.6f,%s,%s,%s\n" % (
                    self.header[i], self.recv[i],
                    _fmt(self.v1[i]), _fmt(self.v2[i]), _fmt(self.v3[i])))
        return path


def _fmt(v):
    return "" if v is None else "%.6f" % v


class ChassisStepTester(object):

    def __init__(self, args):
        self.args = args
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(args.output_dir, self.run_id)
        os.makedirs(self.out_dir, exist_ok=True)

        rospy.init_node("chassis_response_test", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        self.cmd_stamped_pub = rospy.Publisher(
            args.cmd_stamped_topic, TwistStamped, queue_size=1)
        self.phase_pub = rospy.Publisher(args.phase_topic, String, queue_size=1)

        q = 1
        self.odom_std = StreamBuffer("odom_std", ("vx_mps", "wz_rads", "none"))
        self.odom_hf = StreamBuffer("odom_hf", ("vx_mps", "wz_rads", "none"))
        self.imu = StreamBuffer("imu", ("gyro_z_rads", "acc_x", "acc_y"))
        self.cmd_sent = StreamBuffer("cmd_sent", ("vx_mps", "wz_rads", "none"))
        self.bridge_trace = []
        self.phases = []
        self.cmd_intent = StreamBuffer("cmd_intent", ("vx_mps", "wz_rads", "none"))

        rospy.Subscriber(args.feedback_standard, Odometry,
                         self._odom_std_cb, queue_size=q)
        rospy.Subscriber(args.feedback_highfreq, Odometry,
                         self._odom_hf_cb, queue_size=q)
        rospy.Subscriber(args.imu_topic, Imu, self._imu_cb, queue_size=q)
        rospy.Subscriber(args.cmd_sent_topic, TwistStamped,
                         self._cmd_sent_cb, queue_size=q)
        rospy.Subscriber(args.bridge_trace_topic, String,
                         self._bridge_trace_cb, queue_size=q)

        self.rate = rospy.Rate(args.publish_rate)
        self._t_start_wall = time.time()

    def _odom_std_cb(self, msg):
        self.odom_std.add(_h(msg), time.time(),
                          msg.twist.twist.linear.x, msg.twist.twist.angular.z, None)

    def _odom_hf_cb(self, msg):
        self.odom_hf.add(_h(msg), time.time(),
                         msg.twist.twist.linear.x, msg.twist.twist.angular.z, None)

    def _imu_cb(self, msg):
        self.imu.add(_h(msg), time.time(),
                     msg.angular_velocity.z,
                     msg.linear_acceleration.x, msg.linear_acceleration.y)

    def _cmd_sent_cb(self, msg):
        self.cmd_sent.add(_h(msg), time.time(),
                          msg.twist.linear.x, msg.twist.angular.z, None)

    def _bridge_trace_cb(self, msg):
        try:
            self.bridge_trace.append((time.time(), json.loads(msg.data)))
        except Exception:
            pass

    def _mark_phase(self, phase, level="", repeat="", mode="direct"):
        stamp = rospy.get_rostime().to_sec()
        row = (stamp, time.time(), self.args.stage, self.args.axis,
               level, repeat, phase, mode)
        self.phases.append(row)
        payload = {"run_id": self.run_id, "stage": self.args.stage,
                   "axis": self.args.axis, "level": level,
                   "repeat": repeat, "phase": phase, "mode": mode,
                   "stamp": stamp, "wall": time.time()}
        self.phase_pub.publish(String(data=json.dumps(payload)))
        rospy.loginfo("[phase] %s" % json.dumps(payload))

    def _axis_cmd(self, val):
        """Map step value to (vx, wz) according to selected axis."""
        if self.args.axis == "angular_z":
            return 0.0, val
        return val, 0.0

    def _publish_intent(self, vx, wz):
        vx = max(-self.args.max_linear, min(self.args.max_linear, vx))
        wz = max(-self.args.max_angular, min(self.args.max_angular, wz))
        stamp = rospy.get_rostime()
        ts = TwistStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = "chassis_response_test"
        ts.twist.linear.x = vx
        ts.twist.angular.z = wz
        self.cmd_stamped_pub.publish(ts)
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self.cmd_vel_pub.publish(tw)
        self.cmd_intent.add(stamp.to_sec(), time.time(), vx, wz, None)

    def run_direct(self):
        levels = self.args.levels
        rospy.loginfo("[A] levels=%s repeats=%d hold=%.1fs zero=%.1fs @%dHz" % (
            levels, self.args.repeats, self.args.hold_sec,
            self.args.zero_sec, self.args.publish_rate))
        deadline = time.time() + self.args.total_timeout
        for lvl in levels:
            if time.time() > deadline:
                rospy.logwarn("[A] total timeout reached, finishing early")
                break
            for rep in range(1, self.args.repeats + 1):
                if time.time() > deadline:
                    rospy.logwarn("[A] total timeout reached, finishing early")
                    return
                for phase, dur, val in (("pre_zero", self.args.zero_sec, 0.0),
                                        ("step", self.args.hold_sec, lvl),
                                        ("post_zero", self.args.zero_sec, 0.0)):
                    self._mark_phase(phase, lvl, rep)
                    vx, wz = self._axis_cmd(val)
                    t_end = time.time() + dur
                    while time.time() < t_end and not rospy.is_shutdown():
                        if time.time() > deadline:
                            break
                        self._publish_intent(vx, wz)
                        self.rate.sleep()
        rospy.loginfo("[A] profile complete")

    def run_ab(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stage=ab requires an interactive terminal")
        raw = raw_input_compat(
            ">> Enter %s level for this run (e.g. 0.2; negative for reverse): " % self.args.axis).strip()
        lvl = float(raw)
        lim = self.args.max_angular if self.args.axis == "angular_z" \
            else self.args.max_linear
        if abs(lvl) > lim:
            raise RuntimeError("Level %.2f exceeds limit %.2f" % (lvl, lim))
        rospy.loginfo("[AB] level=%.2f direct:%dHz neupan:%.1fHz+max_acce%.2f "
                      "hold=%.1fs zero=%.1fs" % (
                          lvl, self.args.publish_rate, self.args.neupan_rate,
                          self.args.neupan_max_acce, self.args.hold_sec,
                          self.args.zero_sec))
        deadline = time.time() + self.args.total_timeout

        # Segment 1: mode=direct (50Hz direct step)
        for phase, dur, val in (("pre_zero", self.args.zero_sec, 0.0),
                                ("step", self.args.hold_sec, lvl),
                                ("post_zero", self.args.zero_sec, 0.0)):
            self._mark_phase(phase, lvl, "1", mode="direct")
            vx, wz = self._axis_cmd(val)
            t_end = time.time() + dur
            while time.time() < t_end and not rospy.is_shutdown():
                if time.time() > deadline:
                    break
                self._publish_intent(vx, wz)
                self.rate.sleep()

        # Segment 2: mode=neupan (--neupan-rate Hz with symmetric max_acce rate limiter)
        nr = rospy.Rate(self.args.neupan_rate)
        dt = 1.0 / self.args.neupan_rate
        dv = self.args.neupan_max_acce * dt
        last_v = 0.0
        last_w = 0.0
        for phase, dur, val in (("pre_zero", self.args.zero_sec, 0.0),
                                ("step", self.args.hold_sec, lvl),
                                ("post_zero", self.args.zero_sec, 0.0)):
            self._mark_phase(phase, lvl, "1", mode="neupan")
            tgt_v, tgt_w = self._axis_cmd(val)
            t_end = time.time() + dur
            while time.time() < t_end and not rospy.is_shutdown():
                if time.time() > deadline:
                    break
                last_v = max(last_v - dv, min(tgt_v, last_v + dv))
                last_w = max(last_w - dv, min(tgt_w, last_w + dv))
                self._publish_intent(last_v, last_w)
                nr.sleep()
        rospy.loginfo("[AB] complete (level=%.2f)", lvl)

    def run_neupan(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stage=neupan requires an interactive terminal")
        rospy.loginfo("[B] Observation mode: does not publish /cmd_vel.")
        deadline = time.time() + self.args.total_timeout
        for lvl in self.args.levels:
            if time.time() > deadline:
                break
            raw_input_compat(
                ">> Set NeuPAN ref_speed to %.2f, start straight-line motion, "
                "and press Enter to mark level_start..." % lvl)
            self._mark_phase("level_start", lvl, "")
            t_end = time.time() + self.args.hold_sec
            while time.time() < t_end and not rospy.is_shutdown():
                if time.time() > deadline:
                    break
                self.rate.sleep()
            self._mark_phase("level_end", lvl, "")
            if lvl != self.args.levels[-1]:
                raw_input_compat(
                    ">> Stop the robot and press Enter for the next level (%s)..."
                    % self.args.levels[self.args.levels.index(lvl) + 1])
        rospy.loginfo("[B] Observation complete")

    def _periodic_save(self, force=False):
        if not force and time.time() - getattr(self, "_last_save", 0) < 30.0:
            return
        self._last_save = time.time()
        for buf in (self.cmd_intent, self.cmd_sent, self.odom_std,
                    self.odom_hf, self.imu):
            buf.save_csv(self.out_dir)
        _save_phases(self.phases, self.out_dir)
        _save_bridge_trace(self.bridge_trace, self.out_dir)

    def save_all(self):
        self._periodic_save(force=True)
        meta = {
            "run_id": self.run_id,
            "stage": self.args.stage,
            "axis": self.args.axis,
            "levels": self.args.levels,
            "repeats": self.args.repeats,
            "hold_sec": self.args.hold_sec,
            "zero_sec": self.args.zero_sec,
            "publish_rate": self.args.publish_rate,
            "max_linear": self.args.max_linear,
            "max_angular": self.args.max_angular,
            "total_timeout": self.args.total_timeout,
            "note": self.args.note,
        }
        with open(os.path.join(self.out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        rospy.loginfo("[save] All data written to %s", self.out_dir)


def _h(msg):
    """Extract message header timestamp in seconds (0.0 if missing)."""
    t = float(msg.header.stamp.to_sec()) if msg.header.stamp.to_sec() > 0 else 0.0
    return t


def _save_phases(phases, out_dir):
    path = os.path.join(out_dir, "phases.csv")
    with open(path, "w") as f:
        f.write("header_stamp,recv_stamp,stage,axis,level,repeat,phase,mode\n")
        for r in phases:
            f.write("%.6f,%.6f,%s,%s,%s,%s,%s,%s\n" % r)


def _save_bridge_trace(rows, out_dir):
    path = os.path.join(out_dir, "bridge_trace.jsonl")
    with open(path, "w") as f:
        for recv, d in rows:
            d = dict(d)
            d["recv"] = recv
            f.write(json.dumps(d) + "\n")


def raw_input_compat(prompt):
    try:
        return raw_input(prompt)
    except NameError:
        return input(prompt)


def main():
    ap = argparse.ArgumentParser(description="Chassis step-response calibration")
    ap.add_argument("--stage", choices=["direct", "ab", "neupan"], default="direct",
                    help="direct=auto profile to bridge; ab=single-level A/B comparison; "
                         "neupan=observation mode")
    ap.add_argument("--axis", choices=["linear_x", "angular_z"], default="linear_x")
    ap.add_argument("--levels", default="0.2,0.4,0.6",
                    help="Comma-separated step levels")
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--hold-sec", type=float, default=10.0,
                    help="Step hold duration in seconds")
    ap.add_argument("--zero-sec", type=float, default=5.0)
    ap.add_argument("--publish-rate", type=int, default=50)
    ap.add_argument("--max-linear", type=float, default=0.6)
    ap.add_argument("--max-angular", type=float, default=0.6)
    ap.add_argument("--total-timeout", type=float, default=900.0)
    ap.add_argument("--neupan-rate", type=float, default=17.0,
                    help="Publish frequency (Hz) for neupan segment in ab mode")
    ap.add_argument("--neupan-max-acce", type=float, default=0.5,
                    help="Symmetric acceleration limit for neupan segment in ab mode")
    ap.add_argument("--note", default="", help="Free-form note saved to meta.json")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--cmd-stamped-topic", default="/chassis_response_test/cmd_stamped")
    ap.add_argument("--phase-topic", default="/chassis_response_test/phase")
    ap.add_argument("--feedback-standard", default="/odin1/odometry")
    ap.add_argument("--feedback-highfreq", default="/odin1/odometry_highfreq")
    ap.add_argument("--imu-topic", default="/odin1/imu")
    ap.add_argument("--cmd-sent-topic", default="/sw01_cmd_vel_bridge/cmd_sent")
    ap.add_argument("--bridge-trace-topic", default="/sw01_cmd_vel_bridge/control_trace")
    ap.add_argument("--output-dir", default=os.path.expanduser(
        "~/sw01_bag_recordings/chassis_response"))
    args = ap.parse_args()

    args.levels = [float(x) for x in str(args.levels).split(",") if x.strip()]
    for lvl in args.levels:
        if args.axis == "linear_x" and abs(lvl) > args.max_linear:
            print("[FATAL] level %.2f exceeds max-linear %.2f" % (lvl, args.max_linear),
                  file=sys.stderr)
            sys.exit(2)
        if args.axis == "angular_z" and abs(lvl) > args.max_angular:
            print("[FATAL] level %.2f exceeds max-angular %.2f" % (lvl, args.max_angular),
                  file=sys.stderr)
            sys.exit(2)

    tester = ChassisStepTester(args)
    time.sleep(0.5)
    rospy.loginfo("[T-0] run_id=%s stage=%s axis=%s levels=%s output=%s",
                  tester.run_id, args.stage, args.axis, args.levels, tester.out_dir)
    if args.stage in ("direct", "ab"):
        rospy.logwarn("[SAFETY] This mode actively drives the robot. Ensure the test area is clear.")
    else:
        rospy.logwarn("[SAFETY] Stage B does not publish cmd_vel; robot is controlled by NeuPAN.")

    try:
        if args.stage == "direct":
            tester.run_direct()
        elif args.stage == "ab":
            tester.run_ab()
        else:
            tester.run_neupan()
    except Exception as exc:
        rospy.logerr("[T-0] Aborted with exception: %s" % exc)
    finally:
        if args.stage == "direct":
            for _ in range(10):
                tester._publish_intent(0.0, 0.0)
                time.sleep(0.02)
        tester.save_all()

    print("\n>> Saved step-response data to: %s" % tester.out_dir)


if __name__ == "__main__":
    main()
