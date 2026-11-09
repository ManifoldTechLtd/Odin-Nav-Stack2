#!/usr/bin/env python3
"""ROS 2 Jazzy node health monitor and SafetyGate for the ONS2 navigation stack."""
from dataclasses import asdict, dataclass
import json
import os
import threading
import time

import rclpy
from atomic_move.srv import AlignTo
from geometry_msgs.msg import Twist
from nav_interfaces.msg import MissionEvent, MissionStatus
from nav_interfaces.srv import PlanPath
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

DANGER_KEYS = frozenset({"odin", "scan", "relocalization"})
SAFETY_ZERO_HZ = 20.0
STARTUP_GRACE_SEC = float(os.environ.get("ONS2_SAFETY_STARTUP_GRACE_SEC", "10.0"))


@dataclass
class NodeResult:
    ok: bool
    detail: str
    cause: str = ""
    action: str = ""

    def to_dict(self):
        return asdict(self)


class TopicWatch:
    def __init__(self, node, topic, msg_type, stale_after, cause_stale="", cause_never="", action_stale="", action_never=""):
        self.stale_after = stale_after
        self.cause_stale = cause_stale
        self.cause_never = cause_never
        self.action_stale = action_stale
        self.action_never = action_never
        self.last_seen = None
        node.create_subscription(msg_type, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, _msg):
        self.last_seen = time.time()

    def check(self) -> NodeResult:
        if self.last_seen is None:
            return NodeResult(False, "NEVER SEEN — no messages received yet", self.cause_never, self.action_never)
        age = time.time() - self.last_seen
        if age > self.stale_after:
            return NodeResult(False, f"STALE — no message for {age:.1f}s (threshold {self.stale_after}s)", self.cause_stale, self.action_stale)
        return NodeResult(True, f"OK — {age:.1f}s ago")


class ServiceWatch:
    def __init__(self, node, service, srv_type=Trigger, ping_timeout=0.5, cause_unavail="", action_unavail=""):
        self.service = service
        self.ping_timeout = ping_timeout
        self.cause_unavail = cause_unavail
        self.action_unavail = action_unavail
        self.client = node.create_client(srv_type, service)

    def check(self) -> NodeResult:
        if self.client.wait_for_service(timeout_sec=self.ping_timeout):
            return NodeResult(True, "OK — service available")
        return NodeResult(False, f"UNAVAILABLE — service {self.service} not responding", self.cause_unavail, self.action_unavail)


class TfWatch:
    def __init__(self, node, map_frame="map", base_frame="base_link", stale_after=3.0):
        self.node = node
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.stale_after = stale_after
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def check(self) -> NodeResult:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:
            return NodeResult(
                False,
                f"TF {self.map_frame}->{self.base_frame} lookup failed: {type(exc).__name__}",
                "Odin driver exited or relocalization has not completed",
                "Check tmux window 'odin' and verify relocalization status",
            )
        stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
        if stamp_ns > 0:
            age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if age > self.stale_after:
                return NodeResult(
                    False,
                    f"TF stale for {age:.1f}s (threshold {self.stale_after}s)",
                    "Odin driver stalled or /tf stream interrupted",
                    "Check tmux window 'odin' and `ros2 topic hz /tf`",
                )
            return NodeResult(True, f"OK — TF fresh ({age:.1f}s)")
        return NodeResult(True, "OK — TF available")


class SafetyGate:
    """Immediate mission cancel + 20 Hz zero-velocity brake on perception/TF stream loss."""

    def __init__(self, node, cmd_vel_topic="/cmd_vel", stop_topic="/neupan/stop", enable_topic="/sru/cmd_vel_enable"):
        self.node = node
        self._pub = node.create_publisher(Twist, cmd_vel_topic, 1)
        self._stop_pub = node.create_publisher(Empty, stop_topic, 1)
        self._enable_pub = node.create_publisher(Bool, enable_topic, 1)
        self._cancel_cli = node.create_client(Trigger, "/waypoint_manager/cancel")
        self.active = False
        self._trip_count = 0
        self._stop_evt = threading.Event()

    def trip(self, keys):
        if self.active:
            return
        self.active = True
        self._trip_count += 1
        self.node.get_logger().error(
            f"[SafetyGate] Stream loss on [{','.join(sorted(keys))}] -> canceling mission and engaging 20Hz brake (#{self._trip_count})"
        )
        self._stop_pub.publish(Empty())
        self._enable_pub.publish(Bool(data=False))
        if self._cancel_cli.service_is_ready():
            self._cancel_cli.call_async(Trigger.Request())
        self._stop_evt.clear()
        threading.Thread(target=self._loop, daemon=True, name="safety-zero-vel").start()

    def release(self):
        if self.active:
            self.active = False
            self._stop_evt.set()
            self.node.get_logger().warn(f"[SafetyGate] Streams recovered; releasing brake (trips={self._trip_count})")

    def _loop(self):
        zero = Twist()
        while not self._stop_evt.is_set() and rclpy.ok():
            self._pub.publish(zero)
            time.sleep(1.0 / SAFETY_ZERO_HZ)


class NeupanStallWatch:
    ACTIVE_STATES = ("NAVIGATING", "RETURNING_HOME")

    def __init__(self, node, stall_timeout, epsilon=0.05, cmd_timeout=0.5, cmd_vel_topic="/cmd_vel"):
        self.stall_timeout = stall_timeout
        self.epsilon = epsilon
        self.cmd_timeout = cmd_timeout
        self.state = None
        self.last_distance = None
        self.last_progress_at = None
        self.last_cmd_at = None
        self.control_mode = None
        self.last_trace_at = None
        self.wait_reason = None
        self.wait_stamp = 0.0
        node.create_subscription(MissionStatus, "/waypoint_manager/status", self._cb, qos_profile_sensor_data)
        node.create_subscription(Twist, cmd_vel_topic, self._cmd_cb, qos_profile_sensor_data)
        node.create_subscription(String, "/neupan/control_trace", self._trace_cb, qos_profile_sensor_data)
        node.create_subscription(MissionEvent, "/waypoint_manager/mission_feedback", self._event_cb, qos_profile_sensor_data)

    def _cmd_cb(self, _msg):
        self.last_cmd_at = time.time()

    def _event_cb(self, msg):
        if msg.event == "PATH_SAFETY_WAIT":
            self.wait_reason = msg.message
            self.wait_stamp = msg.timestamp or time.time()

    def _trace_cb(self, msg):
        try:
            trace = json.loads(msg.data)
            mode = str(trace.get("mode") or "")
            stamp = float(trace.get("stamp", time.time()))
        except (ValueError, TypeError, AttributeError):
            return
        if not mode:
            return
        self.control_mode = mode
        self.last_trace_at = time.time()
        if mode in ("MPC", "ESCAPE") and stamp >= self.wait_stamp:
            self.wait_reason = None

    def _cb(self, msg):
        self.state = msg.state
        if self.state not in self.ACTIVE_STATES:
            self.last_distance = None
            self.last_progress_at = None
            self.wait_reason = None
            return
        now = time.time()
        if self.last_distance is None or abs(msg.distance_to_target_m - self.last_distance) > self.epsilon:
            self.last_distance = msg.distance_to_target_m
            self.last_progress_at = now

    def check(self) -> NodeResult:
        if self.state is None:
            return NodeResult(False, "NEVER SEEN — no mission status received yet", "mission_manager is not running", "Check tmux window 'mission'")
        if self.state not in self.ACTIVE_STATES:
            return NodeResult(True, f"idle — mission state: {self.state}")
        now = time.time()
        cmd_age = None if self.last_cmd_at is None else now - self.last_cmd_at
        missing_cmd = cmd_age is None or cmd_age > self.cmd_timeout
        waiting = (
            self.last_trace_at is not None
            and now - self.last_trace_at < 1.0
            and self.control_mode in ("IDLE", "WAIT_PATH", "WAIT_TF", "RECOVERY_WAIT")
        )
        if self.wait_reason and (missing_cmd or waiting):
            return NodeResult(
                False,
                f"WAIT — {self.wait_reason} (state={self.state})",
                "mission_manager is in safety wait state",
                "Inspect /scan, /scan_360, /tf, and obstacle clearance along active path",
            )
        if missing_cmd:
            detail = "WAIT — no /cmd_vel received yet" if cmd_age is None else f"CMD_STALE — no /cmd_vel for {cmd_age:.2f}s (threshold {self.cmd_timeout:.2f}s)"
            return NodeResult(False, detail, "Active mission has no recent velocity output", "Check local planner logs and /neupan/control_trace")
        if waiting:
            return NodeResult(False, f"WAIT — control mode {self.control_mode} (state={self.state})", "Local planner is waiting for a valid path or TF", "Check mission path status, perception, and TF")
        if self.last_progress_at is None:
            return NodeResult(True, f"Waiting for initial progress feedback (state={self.state})")
        age = now - self.last_progress_at
        if age > self.stall_timeout:
            return NodeResult(False, f"STALLED — no distance progress for {age:.1f}s (state={self.state})", "Velocity commands are published but the robot is not advancing", "Check chassis bridge state, obstacle clearance, and localization")
        return NodeResult(True, f"OK — advancing ({age:.1f}s ago, state={self.state})")


def system_note(results: dict) -> str:
    failed = [k for k, v in results.items() if not v.ok]
    never = [k for k, v in results.items() if "NEVER SEEN" in v.detail]
    if len(never) >= 3:
        return "Multiple nodes have never published — verify the stack tmux session is running"
    if len(failed) >= 4:
        return "Multiple nodes are offline — check tmux window 'odin' first"
    if len(failed) == 1 and failed[0] in results:
        return f"Single component anomaly ({failed[0]}): {results[failed[0]].action}"
    return ""


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("node_health_monitor")
        self.start_time = time.time()

        odom_topic = self.declare_parameter("odom_topic", "/odin1/odometry").value
        scan_topic = self.declare_parameter("scan_topic", "/scan").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        stop_topic = self.declare_parameter("stop_topic", "/neupan/stop").value
        enable_topic = self.declare_parameter("enable_topic", "/sru/cmd_vel_enable").value
        plan_service = self.declare_parameter("plan_service", "/map_planner/plan").value
        align_svc = self.declare_parameter("align_service", "/atomic_move/align").value
        stall_sec = self.declare_parameter("nav_stall_timeout_sec", 15.0).value
        cmd_timeout = self.declare_parameter("cmd_vel_timeout_sec", 0.5).value
        period_sec = self.declare_parameter("print_period_sec", 2.0).value
        map_frame = self.declare_parameter("tf_map_frame", "map").value
        base_frame = self.declare_parameter("tf_base_frame", "base_link").value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(String, "/waypoint_manager/system_health", latched)

        self.watches = {
            "odin": TopicWatch(self, odom_topic, Odometry, 3.0, "Odin driver exited or USB disconnected", "Odin driver not started", "Check tmux window 'odin' and USB connection", "Start stack via scripts/ons2_sw01.sh"),
            "scan": TopicWatch(self, scan_topic, LaserScan, 3.0, "pc_to_scan or Odin cloud stream interrupted", "pc_to_scan not started", "Check tmux window 'sw01' and /odin1/cloud_slam", "Check tmux window 'sw01'"),
            "relocalization": TfWatch(self, map_frame=map_frame, base_frame=base_frame, stale_after=3.0),
            "mission_manager": TopicWatch(self, "/waypoint_manager/status", MissionStatus, 2.0, "mission_manager stalled or exited", "mission_manager not started yet", "Check tmux window 'mission'", "Check tmux window 'mission'"),
            "map_planner": ServiceWatch(self, plan_service, PlanPath, 0.5, "Global planner not running or map failed to load", "Check tmux window 'sw01' and map_server"),
            "atomic_move": ServiceWatch(self, align_svc, AlignTo, 0.5, "atomic_move node not running", "Check tmux window 'atomic'"),
            "neupan": NeupanStallWatch(self, stall_timeout=stall_sec, cmd_timeout=cmd_timeout, cmd_vel_topic=cmd_vel_topic),
        }
        self.safety_gate = SafetyGate(self, cmd_vel_topic=cmd_vel_topic, stop_topic=stop_topic, enable_topic=enable_topic)
        self.timer = self.create_timer(max(0.2, float(period_sec)), self._tick)

    def _tick(self):
        results = {key: watch.check() for key, watch in self.watches.items()}
        all_ok = all(r.ok for r in results.values())
        note = system_note(results)

        lines = ["==== ONS2 ROS 2 Node Health & SafetyGate Monitor ===="]
        for key, res in results.items():
            lines.append(f"  {'[OK]  ' if res.ok else '[FAIL]'} {key:<16s}: {res.detail}")
            if not res.ok and res.action:
                lines.append(f"         Action: {res.action}")
        if note:
            lines.append(f"  [NOTE] {note}")

        in_grace = (time.time() - self.start_time) < STARTUP_GRACE_SEC
        danger = {k for k, r in results.items() if not r.ok and k in DANGER_KEYS and not (in_grace and "NEVER SEEN" in r.detail)}
        self.safety_gate.trip(danger) if danger else self.safety_gate.release()
        if self.safety_gate.active:
            lines.append(f"  [SAFETY GATE ACTIVE] 20Hz zero-velocity brake engaged (danger={','.join(sorted(danger))})")
        self.get_logger().info("\n".join(lines))

        payload = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "all_ok": all_ok,
            "nodes": {k: v.to_dict() for k, v in results.items()},
            "safety": {"active": self.safety_gate.active, "trip_count": self.safety_gate._trip_count, "danger_keys": sorted(DANGER_KEYS)},
        }
        if note:
            payload["system_note"] = note
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.health_pub.publish(out)


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
