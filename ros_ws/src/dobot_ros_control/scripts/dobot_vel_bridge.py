#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dobot_vel_bridge.py — Dobot Lumo chassis velocity bridge (Python JSON-RPC client).

  - Dispatches {"jsonrpc":"2.0","method":"SetVel","params":{vx,vy,vyaw,duration},"id":1}
    via rpc_python.
  - Deadband thresholds: linear 0.43 m/s, angular 0.40 rad/s (with boost policy).
  - Mode policy: only transitions between standing (300) and walk (304) when
    allow_walk_switch=true; never enters damping (1) or prepare (100) states.
"""

import os
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Resolve package source path so `scripts/rpc_python/` is importable when run via catkin devel relay.
_HERE = os.path.dirname(os.path.abspath(__file__))
try:
    import rospkg
    _PKG = rospkg.RosPack().get_path('dobot_ros_control')
except Exception:  # noqa: BLE001
    _PKG = os.path.dirname(_HERE)
for _p in (os.path.join(_PKG, 'scripts', 'rpc_python'),
           os.path.join(_PKG, 'scripts'),
           os.path.join(_HERE, 'rpc_python'),
           _HERE):
    if _p and _p not in sys.path:
        sys.path.insert(0, _p)
from python_rpc_client import Sim2RealRpcClient  # noqa: E402
from o1l_input_guard import NavigationInputs  # noqa: E402

FSM_DAMPING = 1
FSM_PREPARE = 100
FSM_STANDING = 300
FSM_WALK = 304


class DobotVelBridge(object):
    def __init__(self):
        self.ip = rospy.get_param('~robot_ip', '192.168.5.2')
        self.port = rospy.get_param('~robot_port', 51234)
        self.rate = rospy.get_param('~rate', 20.0)
        self.timeout = rospy.get_param('~rpc_timeout', 1.0)
        self.dry_run = rospy.get_param('~dry_run', True)

        self.deadband_lin = rospy.get_param('~deadband_lin', 0.43)
        self.deadband_ang = rospy.get_param('~deadband_ang', 0.40)
        self.policy = rospy.get_param('~below_deadband_policy', 'boost')

        self.max_lin = rospy.get_param('~max_linear_vel', 1.0)
        self.max_lin_back = rospy.get_param('~max_linear_backward', 0.5)
        self.max_ang = rospy.get_param('~max_angular_vel', 1.0)

        self.cmd_timeout = rospy.get_param('~cmd_vel_timeout', 0.5)
        self.decay_window = rospy.get_param('~cmd_vel_decay_window', 0.2)

        self.allow_walk_switch = rospy.get_param('~allow_walk_switch', False)
        self.switch_guard = rospy.get_param('~fsm_switch_guard_ms', 800) / 1000.0
        self.fsm_watch_hz = rospy.get_param('~fsm_watch_hz', 2.0)
        self.duration = rospy.get_param('~setvel_duration_s', 1.0)

        self.rpc = Sim2RealRpcClient(self.ip, self.port, timeout=self.timeout)
        self.state_topic = rospy.get_param('~state_topic', '/base/state')

        self._cmd = Twist()
        self._cmd_stamp = 0.0
        self._has_cmd = False
        self._lock = threading.Lock()

        self._fsm = -1
        self._fsm_stamp = 0.0
        self._last_switch = 0.0
        self._running = False
        self._err = 'E000OK'
        self._input_guard = NavigationInputs()

        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=10)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self._on_cmd, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odin1/odometry', Odometry,
                                        self._input_guard.update_odom, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan_360', LaserScan,
                                        self._input_guard.update_scan, queue_size=1)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo('[dobot-py] robot=%s:%d rate=%.0fHz deadband(lin=%.2f ang=%.2f) policy=%s duration=%.2fs',
                      self.ip, self.port, self.rate, self.deadband_lin, self.deadband_ang,
                      self.policy, self.duration)

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = msg
            self._cmd_stamp = time.time()
            self._has_cmd = True

    def _shutdown(self):
        if self.dry_run:
            return
        rospy.logwarn('[dobot-py] shutting down -> send zero velocity')
        try:
            self.rpc.set_velocity(0.0, 0.0, 0.0, self.duration)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[dobot-py] zero on shutdown failed: %s', exc)

    def _apply_deadband(self, v, band, label):
        if v == 0.0:
            return 0.0
        if abs(v) >= band:
            return v
        if self.policy == 'boost':
            boosted = band if v > 0 else -band
            rospy.loginfo_throttle(5.0, '[dobot-py] %s %.3f below deadband %.3f -> boost to %.3f',
                                   label, v, band, boosted)
            return boosted
        return 0.0

    def _shape(self, vx, vyaw):
        vx = max(-self.max_lin_back, min(self.max_lin, vx))
        vyaw = max(-self.max_ang, min(self.max_ang, vyaw))
        vx = self._apply_deadband(vx, self.deadband_lin, 'vx')
        vyaw = self._apply_deadband(vyaw, self.deadband_ang, 'vyaw')
        return vx, vyaw

    def _watch_fsm(self):
        try:
            fsm = self.rpc.get_fsm_id()
            if isinstance(fsm, dict):
                fsm = fsm.get('fsm_id', -1)
            self._fsm = int(fsm)
            self._fsm_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(10.0, '[dobot-py] GetFsmId failed: %s', exc)
            return

        if self._fsm == FSM_WALK:
            if not self._running:
                rospy.loginfo('[dobot-py] FSM=304(WALK) -> RUNNING')
                self._running = True
                self._err = 'E000OK'
        else:
            if self._running:
                rospy.logwarn('[dobot-py] FSM=%d (not walk) -> PAUSE, mode left untouched', self._fsm)
                self._running = False
                self._err = 'E002NOT_WALK'

    def _try_switch_to_walk(self):
        """Switch from STANDING (300) to WALK (304) on fresh cmd_vel when allow_walk_switch=true."""
        if self.dry_run or not (self.allow_walk_switch and self._fsm == FSM_STANDING):
            return False
        if time.time() - self._last_switch < self.switch_guard:
            return False
        self._last_switch = time.time()
        try:
            self.rpc.set_fsm_id(FSM_WALK)
            rospy.loginfo('[dobot-py] FSM=300 + fresh cmd_vel -> switch to WALK(304)')
            time.sleep(0.05)
            self._watch_fsm()
            return True
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[dobot-py] 300->304 failed: %s', exc)
            return False

    def spin(self):
        rate = rospy.Rate(self.rate)
        next_watch = 0.0
        while not rospy.is_shutdown():
            nav_reason = self._input_guard.status(rospy.Time.now().to_sec())
            if self.dry_run:
                self.pub_state.publish(String(
                    data='DRY_RUN|%s|py|fsm=-1|vx=0.000|vyaw=0.000' % (
                        'E000OK' if nav_reason == 'OK' else 'E004NAV:' + nav_reason)))
                rate.sleep()
                continue
            now = time.time()

            if now >= next_watch:
                next_watch = now + 1.0 / self.fsm_watch_hz
                self._watch_fsm()

            with self._lock:
                has = self._has_cmd
                age = now - self._cmd_stamp
                vx = self._cmd.linear.x
                vyaw = self._cmd.angular.z

            fresh = (has and 0 <= age < self.cmd_timeout
                     and math.isfinite(vx) and math.isfinite(vyaw)
                     and nav_reason == 'OK')
            if nav_reason != 'OK':
                self._err = 'E004NAV:' + nav_reason
            elif self._err.startswith('E004NAV:'):
                self._err = 'E000OK'
            if (not self._running) and fresh and (abs(vx) > 1e-6 or abs(vyaw) > 1e-6):
                self._try_switch_to_walk()

            if self._running:
                if fresh:
                    remain = self.cmd_timeout - age
                    k = 1.0
                    if self.decay_window > 0 and remain < self.decay_window:
                        k = max(0.0, remain / self.decay_window)
                    out_vx, out_vyaw = self._shape(vx * k, vyaw * k)
                else:
                    out_vx, out_vyaw = 0.0, 0.0
                    if has and nav_reason == 'OK':
                        self._err = 'E003CMD_TIMEOUT'
                try:
                    self.rpc.set_velocity(out_vx, 0.0, out_vyaw, self.duration)
                except Exception as exc:  # noqa: BLE001
                    rospy.logwarn_throttle(2.0, '[dobot-py] SetVel failed: %s', exc)
                    self._err = 'E001RPC_UNREACHABLE'
            else:
                out_vx, out_vyaw = 0.0, 0.0

            self.pub_state.publish(String(data='%s|%s|py|fsm=%d|vx=%.3f|vyaw=%.3f' % (
                'RUNNING' if self._running else 'PAUSED', self._err, self._fsm, out_vx, out_vyaw)))
            rate.sleep()


def main():
    rospy.init_node('dobot_vel_bridge')
    node = DobotVelBridge()
    node.spin()


if __name__ == '__main__':
    main()
