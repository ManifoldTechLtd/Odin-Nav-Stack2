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

"""Actual nodes on synthetic inputs; private command topics and no robot SDK."""
import math
import os
import subprocess
import threading
import time
import uuid
from pathlib import Path as FilePath

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_interfaces.action import NavPath
from nav_interfaces.msg import MissionEvent, Waypoint
from nav_interfaces.srv import PlanPath, PlanSequence, StartMission
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def wait_until(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for navigation safety behavior")


@pytest.fixture(scope="module", autouse=True)
def isolated_ros():
    assert os.environ.get("ROS_DOMAIN_ID") == "184", "Use private test domain 184"
    assert os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"
    rclpy.init()
    yield
    rclpy.shutdown()


class NavigationRig:
    def __init__(self, map_args=()):
        self.node = rclpy.create_node("path_safety_test_" + uuid.uuid4().hex[:8])
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.running = True
        self.fresh = True
        self.pose_fresh = True
        self.fake_enabled = True
        self.robot = [0.0, 0.0, 0.0]
        self.paths = []
        self.stops = []
        self.events = []
        self.latest_handle = None
        self.processes = []
        self.logs = []
        self.obstacle = None
        self.plan_delay = 0.0
        self.plan_failure = False
        self.plan_requests = []
        self.plan_inflight = 0
        self.max_plan_inflight = 0
        self.sequence_calls = 0
        self.delay_seen = threading.Event()
        self.reject_next = False
        self.rejected_goals = []
        self.complete_current = False
        self.cancel_delay = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, "/map", qos)
        self.fake_pub = self.node.create_publisher(OccupancyGrid, "/sliding_map", qos)
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan_360", 5)
        self.stop_sub = self.node.create_subscription(
            Empty, "/neupan/stop", lambda _: self.stops.append(time.monotonic()), 10)
        self.event_sub = self.node.create_subscription(
            MissionEvent, "/waypoint_manager/mission_feedback",
            lambda msg: self.events.append(msg), 10)
        self.static_tf = StaticTransformBroadcaster(self.node)
        self.dynamic_tf = TransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        transform.header.stamp = self.node.get_clock().now().to_msg()
        self.static_tf.sendTransform(transform)
        self.action = ActionServer(
            self.node, NavPath, "/neupan/nav_path", self.execute_path,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            goal_callback=self.accept_goal,
            callback_group=ReentrantCallbackGroup())
        self.pump_thread = threading.Thread(target=self.pump, daemon=True)
        self.pump_thread.start()
        self.start_node("map_planner", "map_planner_node", [
            "-p", "inflation_radius:=0.2",
            "-p", "obstacle_cost_safe_distance:=0.4",
            "-p", "line_of_sight_safe_distance:=0.1",
            "-p", "smoothing_iterations:=20",
            "-p", "sliding_map_max_age_sec:=0.7",
            "-p", "inflated_map_publish_period_sec:=0.2",
            "-r", "/map_planner/plan_sequence:=/safety_test/actual_sequence"] + list(map_args))
        self.plan_client = self.node.create_client(PlanPath, "/map_planner/plan")
        assert self.plan_client.wait_for_service(timeout_sec=10.0)
        self.sequence_client = self.node.create_client(PlanSequence, "/safety_test/actual_sequence")
        def sequence_proxy(request, _):
            self.sequence_calls += 1
            return self.call(self.sequence_client, request, timeout=40.0)
        self.sequence_proxy = self.node.create_service(
            PlanSequence, "/map_planner/plan_sequence", sequence_proxy,
            callback_group=ReentrantCallbackGroup())
        wait_until(lambda: self.plan(3.0).success)

    def start_node(self, package, executable, args):
        binary = os.path.join(get_package_prefix(package), "lib", package, executable)
        log = open("/tmp/" + self.node.get_name() + "_" + executable + ".log", "w+")
        self.logs.append(log)
        self.processes.append(subprocess.Popen([binary, "--ros-args"] + args,
                                              stdout=log, stderr=subprocess.STDOUT))

    def grid(self, dynamic=False):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.info.resolution = 0.1
        msg.info.width = 400
        msg.info.height = 200
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -10.0
        msg.info.origin.orientation.w = 1.0
        grid = np.zeros((200, 400), dtype=np.int8)
        if dynamic and self.obstacle:
            x0, x1, y0, y1 = self.obstacle
            grid[int((y0 + 10) / 0.1):int((y1 + 10) / 0.1) + 1,
                 int((x0 + 5) / 0.1):int((x1 + 5) / 0.1) + 1] = 100
        msg.data = grid.ravel().tolist()
        return msg

    def pump(self):
        while self.running:
            transform = TransformStamped()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.transform.translation.x = self.robot[0]
            transform.transform.translation.y = self.robot[1]
            transform.transform.rotation.z = math.sin(self.robot[2] / 2)
            transform.transform.rotation.w = math.cos(self.robot[2] / 2)
            if self.pose_fresh:
                self.dynamic_tf.sendTransform(transform)
            self.map_pub.publish(self.grid())
            if self.fresh:
                if self.fake_enabled:
                    self.fake_pub.publish(self.grid(dynamic=True))
                scan = LaserScan()
                scan.header.stamp = self.node.get_clock().now().to_msg()
                scan.header.frame_id = "base_link"
                scan.angle_min = -math.pi
                scan.angle_increment = math.pi / 180
                scan.range_min = 0.05
                scan.range_max = 4.5
                scan.ranges = [math.inf] * 360
                self.scan_pub.publish(scan)
            time.sleep(0.08)

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        wait_until(future.done, timeout)
        assert future.exception() is None
        return future.result()

    def plan(self, x, y=0.0):
        request = PlanPath.Request()
        request.goal.header.frame_id = "map"
        request.goal.pose.position.x = float(x)
        request.goal.pose.position.y = float(y)
        request.goal.pose.orientation.w = 1.0
        request.shadow_only = True
        return self.call(self.plan_client, request)

    def start_mission(self, goal=25.0, delayed_plan=False, wait_for_path=True,
                      initial_timeout=None, extra_args=()):
        args = [
            "-p", "navigation_mode:=replan_periodic",
            "-p", "periodic_replan_sec:=0.8",
            "-p", "enable_alignment:=false",
            "-p", "nav_stall_timeout_sec:=30.0",
            "-p", "plan_max_retries:=0"]
        if initial_timeout is not None:
            args += ["-p", f"initial_plan_timeout_sec:={float(initial_timeout)}"]
        if delayed_plan:
            def proxy(request, _):
                self.plan_requests.append((time.monotonic(), request.goal.pose.position.x))
                self.plan_inflight += 1
                self.max_plan_inflight = max(self.max_plan_inflight, self.plan_inflight)
                try:
                    if self.plan_failure:
                        result = PlanPath.Response()
                        result.message = "NO_PATH_FOUND"
                        return result
                    result = self.call(self.plan_client, request)
                    if self.plan_delay:
                        self.delay_seen.set()
                        time.sleep(self.plan_delay)
                    return result
                finally:
                    self.plan_inflight -= 1
            self.proxy = self.node.create_service(
                PlanPath, "/safety_test/plan", proxy,
                callback_group=ReentrantCallbackGroup())
            args += ["-p", "plan_service:=/safety_test/plan"]
        self.start_node("mission_manager", "mission_manager_node", args + list(extra_args))
        result = self.submit_mission(goal)
        if wait_for_path:
            wait_until(lambda: self.paths)
        return result

    def submit_mission(self, goal):
        client = self.node.create_client(StartMission, "/waypoint_manager/start_mission")
        assert client.wait_for_service(timeout_sec=10.0)
        request = StartMission.Request()
        goals = goal if isinstance(goal, (list, tuple)) else [goal]
        request.waypoints = []
        for gx in goals:
            waypoint = Waypoint()
            waypoint.x = float(gx)
            waypoint.y = 0.0
            request.waypoints.append(waypoint)
        begin = time.monotonic()
        result = self.call(client, request)
        self.start_response_seconds = time.monotonic() - begin
        assert result.success, result.message
        return result

    def accept_goal(self, goal):
        if self.reject_next:
            self.reject_next = False
            self.rejected_goals.append(goal.path)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_path(self, handle):
        self.latest_handle = handle
        self.paths.append(handle.request.path)
        while (self.running and handle == self.latest_handle
               and not handle.is_cancel_requested and not self.complete_current):
            feedback = NavPath.Feedback()
            goal = handle.request.path.poses[-1].pose.position
            feedback.distance_remaining = math.hypot(
                goal.x - self.robot[0], goal.y - self.robot[1])
            handle.publish_feedback(feedback)
            time.sleep(0.04)
        if self.complete_current and handle == self.latest_handle:
            self.complete_current = False
            handle.succeed()
            result = NavPath.Result()
            result.status = 0
            return result
        if handle.is_cancel_requested and self.cancel_delay > 0.0:
            time.sleep(self.cancel_delay)
        if handle.status == GoalStatus.STATUS_EXECUTING:
            handle.abort()
        elif handle.is_cancel_requested:
            handle.canceled()
        result = NavPath.Result()
        result.status = 1
        return result

    def close(self):
        self.running = False
        self.pump_thread.join(timeout=3)
        self.executor.shutdown(timeout_sec=3)
        self.spin_thread.join(timeout=3)
        self.action.destroy()
        for process in reversed(self.processes):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.node.destroy_node()
        for log in self.logs:
            log.close()


@pytest.fixture
def rig(request):
    instance = NavigationRig.__new__(NavigationRig)
    try:
        instance.__init__(map_args=getattr(request, "param", ()))
        yield instance
    finally:
        if hasattr(instance, "processes"):
            instance.close()


def path_xy(path):
    return np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])


def test_stale_sliding_map_is_not_static_only_success(rig):
    rig.fake_enabled = False
    time.sleep(1.0)
    result = rig.plan(4.0)
    assert not result.success, "Stale dynamic map was silently ignored"
    assert "DYNAMIC_MAP" in result.message


def test_navigation_tf_consumers_use_fresh_dynamic_samples_and_retained_static_tf(rig):
    from rclpy.qos import ReliabilityPolicy

    rig.start_mission()
    dynamic = wait_until(lambda: (
        items if len(items := rig.node.get_subscriptions_info_by_topic("/tf")) >= 2 else None))
    assert all(item.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
               for item in dynamic)
    static = rig.node.get_subscriptions_info_by_topic("/tf_static")
    assert len(static) >= 2
    assert all(item.qos_profile.reliability == ReliabilityPolicy.RELIABLE
               and item.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
               for item in static)


def test_overlap_replans_around_existing_obstacle(rig):
    rig.start_mission()
    rig.obstacle = (8.1, 9.2, -0.4, 0.4)
    time.sleep(0.25)
    rig.robot = [7.2, -0.34, 0.0]
    path = wait_until(lambda: next(
        (p for p in reversed(rig.paths) if p.poses[-1].pose.position.x > 14), None))
    xy = path_xy(path)
    crossing = xy[(xy[:, 0] >= 8.0) & (xy[:, 0] <= 9.4)]
    assert len(crossing)
    assert np.all((crossing[:, 1] < -0.5) | (crossing[:, 1] > 0.6)), (
        "Overlap installed the cached route through the existing obstacle", crossing)


@pytest.mark.parametrize("lateral_m", [
    pytest.param(0.33977478375698117, id="bag_880s"),
    pytest.param(0.20107762446270877, id="bag_1208s"),
])
def test_overlap_has_no_nearest_point_right_angle(rig, lateral_m):
    rig.start_mission()
    rig.robot = [7.2, -lateral_m, 0.0]
    path = wait_until(lambda: next(
        (p for p in reversed(rig.paths) if p.poses[-1].pose.position.x > 14), None))
    delta = np.diff(path_xy(path)[:3], axis=0)
    cross = delta[0, 0] * delta[1, 1] - delta[0, 1] * delta[1, 0]
    kink = abs(math.atan2(cross, delta[0] @ delta[1]))
    assert math.degrees(kink) < 45, "Synthetic sideways connector remains"


def test_occupied_intermediate_anchor_has_forward_alternative(rig):
    rig.start_mission()
    anchor = rig.paths[0].poses[-1].pose.position.x
    rig.obstacle = (anchor - 0.15, anchor + 0.15, -0.2, 0.2)
    path = wait_until(lambda: next(
        (p for p in reversed(rig.paths) if p.poses[-1].pose.position.x > anchor + 0.5),
        None), timeout=8.0)
    assert path.poses[-1].pose.position.x < 17.0, "Anchor search must be bounded"


def test_sensor_loss_stops_then_recovers_without_skipping(rig):
    rig.start_mission()
    initial_stops = len(rig.stops)
    rig.fresh = False
    wait_until(lambda: len(rig.stops) > initial_stops, timeout=4.0)
    count = len(rig.paths)
    rig.fresh = True
    wait_until(lambda: len(rig.paths) > count, timeout=5.0)
    assert rig.paths[-1].poses[-1].pose.position.x < 12.0


def test_stale_robot_pose_stops_even_when_scans_continue(rig):
    rig.start_mission()
    stopped = len(rig.stops)
    rig.pose_fresh = False
    wait_until(lambda: len(rig.stops) > stopped, timeout=4.0)
    assert rig.fresh
    paths = len(rig.paths)
    rig.pose_fresh = True
    wait_until(lambda: len(rig.paths) > paths, timeout=5.0)


def test_atomic_move_rejects_a_stale_pose_before_publishing_motion(rig):
    from atomic_move.srv import Move
    from geometry_msgs.msg import Twist

    rig.pose_fresh = False
    commands = []
    rig.node.create_subscription(Twist, "/safety_test/atomic_cmd_vel", commands.append, 10)
    stop = rig.node.create_publisher(Empty, "/safety_test/atomic_stop", 10)
    rig.start_node("atomic_move", "atomic_move_node", [
        "-r", "__node:=atomic_tf_test",
        "-r", "/cmd_vel:=/safety_test/atomic_cmd_vel",
        "-r", "/neupan/stop:=/safety_test/atomic_stop",
        "-p", "control_frame:=map"])
    client = rig.node.create_client(Move, "/atomic_tf_test/move")
    assert client.wait_for_service(timeout_sec=5.0)
    wait_until(lambda: len(rig.node.get_subscriptions_info_by_topic("/tf")) >= 2)
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.header.stamp = (rig.node.get_clock().now() -
                             rclpy.duration.Duration(seconds=2.0)).to_msg()
    transform.transform.rotation.w = 1.0
    for _ in range(3):
        rig.dynamic_tf.sendTransform(transform)
        time.sleep(0.05)
    pending = client.call_async(Move.Request(distance=0.5))
    try:
        wait_until(lambda: pending.done() or any(abs(m.linear.x) > 0 for m in commands))
        assert not any(abs(m.linear.x) > 0 for m in commands), "Motion used a stale robot pose"
        assert pending.done()
        assert not pending.result().success
        assert "TRANSFORM_UNAVAILABLE" in pending.result().message
    finally:
        stop.publish(Empty())
        wait_until(pending.done, timeout=3.0)


def test_cold_service_reply_loss_does_not_interrupt_established_navigation(rig):
    """DDS may lose a new endpoint's first reply during asymmetric discovery.

    Inject that loss at the response transport boundary after navigation starts.
    Keep real mission/planner binaries, deadlines, maps, scans and action feedback.
    """
    from nav_interfaces.srv import ValidatePath

    backend = rig.node.create_client(ValidatePath, "/map_planner/validate_path")
    assert backend.wait_for_service(timeout_sec=2.0)
    validation = rig.node.create_service(
        ValidatePath, "/safety_test/validate_path",
        lambda request, _: rig.call(backend, request),
        callback_group=ReentrantCallbackGroup())
    rig.start_mission(delayed_plan=True, extra_args=[
        "-r", "/map_planner/validate_path:=/safety_test/validate_path"])
    fault = {"all_replies": False}
    dropped = []

    def inject_loss(service):
        original_send = service.send_response

        def send(response, header):
            sequence = header.request_id.sequence_number
            if fault["all_replies"] or sequence == 1:
                dropped.append((service.service_name, sequence))
                return
            original_send(response, header)

        service.send_response = send

    inject_loss(validation)
    inject_loss(rig.proxy)
    paths, stops = len(rig.paths), len(rig.stops)
    wait_until(lambda: len(rig.paths) >= paths + 2 or len(rig.stops) > stops, timeout=7.0)
    assert len(rig.stops) == stops, f"Cold service endpoints interrupted navigation: {dropped}"
    assert len(rig.paths) >= paths + 2, "Periodic replanning lost its response session"

    # A genuine outage must still stop, and restoring replies must permit retry.
    fault["all_replies"] = True
    wait_until(lambda: len(rig.stops) > stops, timeout=4.0)
    paths = len(rig.paths)
    fault["all_replies"] = False
    wait_until(lambda: len(rig.paths) > paths, timeout=7.0)
    assert any(event.event == "PATH_SAFETY_WAIT" for event in rig.events)


def validate(rig, points, check_start=True):
    from geometry_msgs.msg import PoseStamped
    from nav_interfaces.srv import ValidatePath

    client = rig.node.create_client(ValidatePath, "/map_planner/validate_path")
    assert client.wait_for_service(timeout_sec=2.0)
    request = ValidatePath.Request()
    request.check_start_connection = check_start
    request.path = Path()
    request.path.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        request.path.poses.append(pose)
    return rig.call(client, request)


def test_final_validation_preserves_polyline_instead_of_prepending_robot(rig):
    rig.robot = [0.08, -0.03, 0.0]
    time.sleep(0.2)
    result = validate(rig, [(0, 0), (0.1, 0), (0.2, 0), (1, 0)])
    assert result.success, result.message
    xy = path_xy(result.path)
    assert np.all(xy[:, 1] == 0.0)
    assert not np.array_equal(xy[0], rig.robot[:2])


def test_final_validation_checks_entire_sparse_segments(rig):
    assert validate(rig, [(0, 0), (4, 0)]).success
    rig.obstacle = (1.8, 2.2, -0.1, 0.1)
    time.sleep(0.25)
    result = validate(rig, [(0, 0), (4, 0)])
    assert not result.success
    assert "COLLISION" in result.message


def test_astar_near_obstacle_corner_returns_a_valid_detour(rig):
    rig.obstacle = (8.1, 9.2, -0.4, 0.4)
    rig.robot = [7.2, -0.34, 0.0]
    time.sleep(0.25)
    result = rig.plan(17.0)
    assert result.success, result.message
    checked = validate(rig, path_xy(result.path))
    assert checked.success, checked.message


@pytest.mark.parametrize("rig", [
    ("-p", "path_min_clear_m:=0.2", "-p", "obstacle_cost_weight:=0.0",
     "-p", "line_of_sight_safe_distance:=0.0", "-p", "smoothing_iterations:=0"),
    ("-p", "path_min_clear_m:=0.2", "-p", "obstacle_cost_weight:=0.0",
     "-p", "line_of_sight_safe_distance:=0.0", "-p", "smoothing_iterations:=40"),
], indirect=True, ids=["astar_and_shortcut", "smoothed"])
def test_planner_uses_the_same_required_clearance_as_install_validation(rig):
    # Soft preferences can be disabled; hard installation clearance must still
    # produce a usable detour rather than repeated success/rejection cycles.
    rig.obstacle = (1.2, 1.6, -0.3, 0.3)
    wait_until(lambda: not validate(rig, [(0.0, 0.0), (3.0, 0.0)]).success)
    result = rig.plan(3.0)
    assert result.success, result.message
    checked = validate(rig, path_xy(result.path))
    assert checked.success, "Planner returned an uninstallable path: " + checked.message


@pytest.mark.parametrize("rig", [
    ("-p", "path_min_clear_m:=0.2", "-p", "obstacle_cost_weight:=0.0",
     "-p", "goal_relocate_enable:=false"),
], indirect=True)
def test_goal_below_required_clearance_is_not_reported_as_a_successful_plan(rig):
    rig.obstacle = (3.21, 3.4, -0.1, 0.1)
    wait_until(lambda: not validate(rig, [(0.0, 0.0), (3.05, 0.0)]).success)
    result = rig.plan(3.05)
    assert not result.success, "A goal below required clearance cannot be installed"
    assert result.message == "GOAL_OCCUPIED"
    assert not result.path.poses


def test_final_waypoint_is_not_relocated_when_occupied(rig):
    rig.start_mission(goal=4.0)
    stops = len(rig.stops)
    rig.obstacle = (3.8, 4.2, -0.2, 0.2)
    wait_until(lambda: len(rig.stops) > stops, timeout=4.0)
    time.sleep(1.6)
    assert all(p.poses[-1].pose.position.x < 4.2 for p in rig.paths)
    assert not any(e.event == "WAYPOINT_SKIPPED" for e in rig.events)


def test_pause_during_safety_wait_prevents_auto_resume(rig):
    rig.start_mission()
    stops = len(rig.stops)
    rig.fresh = False
    wait_until(lambda: len(rig.stops) > stops, timeout=4.0)
    client = rig.node.create_client(Trigger, "/waypoint_manager/pause")
    assert client.wait_for_service(timeout_sec=2.0)
    assert rig.call(client, Trigger.Request()).success
    count = len(rig.paths)
    rig.fresh = True
    time.sleep(1.8)
    assert len(rig.paths) == count


def test_late_plan_cannot_install_after_cancel(rig):
    rig.start_mission(delayed_plan=True)
    rig.plan_delay = 1.0
    assert rig.delay_seen.wait(timeout=3.0)
    client = rig.node.create_client(Trigger, "/waypoint_manager/cancel")
    assert client.wait_for_service(timeout_sec=2.0)
    assert rig.call(client, Trigger.Request()).success
    count = len(rig.paths)
    time.sleep(1.5)
    assert len(rig.paths) == count


def test_valid_old_path_survives_ordinary_planning_failure(rig):
    rig.start_mission(delayed_plan=True)
    stops = len(rig.stops)
    events = len(rig.events)
    rig.plan_failure = True
    time.sleep(1.8)
    assert len(rig.stops) == stops
    assert not any(e.event == "PATH_SAFETY_WAIT" for e in rig.events[events:])


def test_rejected_segment_does_not_commit_the_new_anchor(rig):
    rig.start_mission()
    old_anchor = rig.paths[0].poses[-1].pose.position.x
    count = len(rig.paths)
    rig.reject_next = True
    rig.robot = [7.2, -0.34, 0.0]
    wait_until(lambda: rig.rejected_goals)
    assert rig.rejected_goals[0].poses[-1].pose.position.x > old_anchor + 5
    wait_until(lambda: len(rig.paths) > count)
    assert abs(rig.paths[count].poses[-1].pose.position.x - old_anchor) < 0.2


def test_slow_initial_plan_does_not_starve_scan_freshness(rig):
    rig.plan_delay = 1.2
    rig.start_mission(delayed_plan=True)
    assert not any(e.event == "PATH_SAFETY_WAIT" for e in rig.events), (
        "Healthy scans were queued behind a synchronous planning call")


def test_initial_plan_over_two_seconds_is_nonblocking_and_installed_once(rig):
    rig.plan_delay = 2.4
    rig.start_mission(delayed_plan=True, wait_for_path=False)
    assert rig.start_response_seconds < 0.6, "Initial planning blocks mission callbacks"
    wait_until(lambda: rig.paths, timeout=8.0)
    assert len(rig.plan_requests) == 1, "A valid slow initial plan was retried"
    assert rig.max_plan_inflight == 1
    assert rig.sequence_calls == 0, "Visualization duplicated the full initial plan"
    assert not any(e.event == "PATH_SAFETY_WAIT" for e in rig.events)


def test_cancel_and_new_target_while_initial_plan_is_pending(rig):
    rig.plan_delay = 2.4
    rig.start_mission(delayed_plan=True, wait_for_path=False)
    assert rig.start_response_seconds < 0.6
    assert rig.delay_seen.wait(timeout=2.0)
    cancel = rig.node.create_client(Trigger, "/waypoint_manager/cancel")
    assert cancel.wait_for_service(timeout_sec=2.0)
    begin = time.monotonic()
    assert rig.call(cancel, Trigger.Request()).success
    assert time.monotonic() - begin < 0.6, "Cancel waits for the old planner response"
    rig.plan_delay = 0.0
    rig.submit_mission(4.0)
    assert rig.start_response_seconds < 0.6
    wait_until(lambda: rig.paths, timeout=8.0)
    assert rig.max_plan_inflight == 1, "New target queued overlapping full-route work"
    assert all(p.poses[-1].pose.position.x < 4.2 for p in rig.paths)
    assert rig.sequence_calls == 0


def test_initial_response_timeout_is_explicit_and_not_automatically_requeued(rig):
    rig.plan_delay = 1.0
    rig.start_mission(delayed_plan=True, wait_for_path=False, initial_timeout=0.4)
    wait_until(lambda: any(e.event == "FAILED" and "PLAN_RESPONSE_TIMEOUT" in e.message
                           for e in rig.events), timeout=4.0)
    time.sleep(1.2)
    assert len(rig.plan_requests) == 1
    assert not rig.paths


def test_sliding_map_reception_continues_during_long_planning_batch(rig):
    client = rig.node.create_client(PlanSequence, "/map_planner/plan_sequence")
    assert client.wait_for_service(timeout_sec=3.0)
    request = PlanSequence.Request()
    request.start.header.frame_id = "map"
    request.start.pose.orientation.w = 1.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 1.0
    goal.pose.orientation.w = 1.0
    request.goals = [goal] * 4096
    result = rig.call(client, request, timeout=40.0)
    assert result.success, "Healthy fake-map input starved behind planning: " + result.message


@pytest.mark.parametrize("package,requested,expected", [
    ("map_planner", "", "Release"), ("mission_manager", "", "Release"),
    ("map_planner", "Debug", "Debug"), ("mission_manager", "Debug", "Debug"),
])
def test_default_navigation_build_is_release(package, requested, expected, tmp_path):
    source = FilePath(__file__).resolve().parents[2] / package
    build = tmp_path / package
    result = subprocess.run([
        "cmake", "-S", str(source), "-B", str(build),
        "-DCMAKE_BUILD_TYPE=" + requested, "-DBUILD_TESTING=OFF",
        "-DPython3_EXECUTABLE=/usr/bin/python3",
    ], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (build / "CMakeCache.txt").read_text()
    assert "CMAKE_BUILD_TYPE:STRING=" + expected in cache, "Navigation build type was not respected"


def test_initial_route_plan_stops_previous_navigation_immediately(rig):
    rig.start_mission(goal=[3.0, 9.0], delayed_plan=True)
    assert len(rig.paths) == 1
    stops_before = len(rig.stops)
    rig.plan_delay = 1.2
    rig.delay_seen.clear()
    rig.complete_current = True

    # While waypoint 2 is waiting in startInitialRoutePlan() (path_safety_waiting_ == true),
    # mission_manager must immediately emit stopNavigation() before the delayed plan finishes.
    assert rig.delay_seen.wait(timeout=3.0)
    wait_until(lambda: len(rig.stops) > stops_before, timeout=0.6)
    assert len(rig.paths) == 1, "Stop arrived only after the new plan was already installed"
    wait_until(lambda: len(rig.paths) >= 2, timeout=5.0)


def test_stop_reasserts_cancel_until_stubborn_action_terminates_and_latches(rig):
    rig.cancel_delay = 0.45
    rig.start_mission()
    stops_before = len(rig.stops)
    cancel = rig.node.create_client(Trigger, "/waypoint_manager/cancel")
    assert cancel.wait_for_service(timeout_sec=2.0)
    assert rig.call(cancel, Trigger.Request()).success

    # Because the action goal stays live for ~0.45s while canceling, mission_manager
    # must re-assert /neupan/stop across process() ticks rather than firing only once.
    wait_until(lambda: len(rig.stops) - stops_before >= 2, timeout=2.0)
    paths_after_cancel = len(rig.paths)

    replan_pub = rig.node.create_publisher(Empty, "/neupan/replan", 10)
    for _ in range(3):
        replan_pub.publish(Empty())
        time.sleep(0.1)
    time.sleep(0.4)
    assert len(rig.paths) == paths_after_cancel, "Latched stop allowed a replan trigger to send a path"

    rig.cancel_delay = 0.0
    rig.submit_mission(4.0)
    wait_until(lambda: len(rig.paths) > paths_after_cancel, timeout=5.0)


def test_atomic_move_rejects_concurrent_request_without_clearing_stop(rig):
    from atomic_move.srv import Move, Turn
    from geometry_msgs.msg import Twist

    commands = []
    rig.node.create_subscription(Twist, "/safety_test/atomic_conc_cmd_vel", commands.append, 50)
    stop = rig.node.create_publisher(Empty, "/safety_test/atomic_conc_stop", 10)
    rig.start_node("atomic_move", "atomic_move_node", [
        "-r", "__node:=atomic_conc_test",
        "-r", "/cmd_vel:=/safety_test/atomic_conc_cmd_vel",
        "-r", "/neupan/stop:=/safety_test/atomic_conc_stop",
        "-p", "control_frame:=map"])
    move_cli = rig.node.create_client(Move, "/atomic_conc_test/move")
    turn_cli = rig.node.create_client(Turn, "/atomic_conc_test/turn")
    assert move_cli.wait_for_service(timeout_sec=5.0)
    assert turn_cli.wait_for_service(timeout_sec=5.0)
    wait_until(lambda: len(rig.node.get_subscriptions_info_by_topic("/tf")) >= 2)
    wait_until(lambda: rig.node.get_subscriptions_info_by_topic("/safety_test/atomic_conc_stop"))

    move_fut = move_cli.call_async(Move.Request(distance=1.5))
    try:
        wait_until(lambda: any(m.linear.x > 0.05 for m in commands), timeout=3.0)

        turn_fut = turn_cli.call_async(Turn.Request(angle=90.0))
        wait_until(turn_fut.done, timeout=1.0)
        assert not turn_fut.result().success
        assert "BUSY" in turn_fut.result().message
        assert not any(abs(m.angular.z) > 1e-6 for m in commands)

        stop.publish(Empty())
        wait_until(move_fut.done, timeout=1.5)
        assert not move_fut.result().success
        assert "CANCELLED" in move_fut.result().message
        assert commands and abs(commands[-1].linear.x) == 0.0 and abs(commands[-1].angular.z) == 0.0
    finally:
        stop.publish(Empty())
        wait_until(move_fut.done, timeout=3.0)


def test_atomic_move_stop_wins_race_against_simultaneous_new_request(rig):
    from atomic_move.srv import AlignTo
    from geometry_msgs.msg import Twist

    commands = []
    rig.node.create_subscription(Twist, "/safety_test/atomic_race_cmd_vel", commands.append, 50)
    stop = rig.node.create_publisher(Empty, "/safety_test/atomic_race_stop", 10)
    rig.start_node("atomic_move", "atomic_move_node", [
        "-r", "__node:=atomic_race_test",
        "-r", "/cmd_vel:=/safety_test/atomic_race_cmd_vel",
        "-r", "/neupan/stop:=/safety_test/atomic_race_stop",
        "-p", "control_frame:=map",
        "-p", "stop_hold_sec:=0.08"])
    align_cli = rig.node.create_client(AlignTo, "/atomic_race_test/align")
    assert align_cli.wait_for_service(timeout_sec=5.0)
    wait_until(lambda: len(rig.node.get_subscriptions_info_by_topic("/tf")) >= 2)
    wait_until(lambda: rig.node.get_subscriptions_info_by_topic("/safety_test/atomic_race_stop"))

    stop.publish(Empty())
    time.sleep(0.01)
    race_fut = align_cli.call_async(AlignTo.Request(x=1.5, y=0.0, yaw=0.0))
    wait_until(race_fut.done, timeout=1.5)
    assert not race_fut.result().success
    assert "CANCELLED" in race_fut.result().message
    assert not any(abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6 for m in commands), (
        "New request arriving immediately after /neupan/stop overwrote stop and drove the robot")

    time.sleep(0.12)
    ok_fut = align_cli.call_async(AlignTo.Request(x=0.0, y=0.0, yaw=0.0))
    wait_until(ok_fut.done, timeout=2.0)
    assert ok_fut.result().success


@pytest.mark.parametrize("rig", [
    ("-p", "path_min_clear_m:=0.10",
     "-p", "sliding_map_start_exempt_radius_m:=0.48"),
], indirect=True)
def test_parallel_passerby_in_blindspot_does_not_trap_start_occupied(rig):
    # Parallel walker / large avoidance turn leaves a blind-spot /sliding_map cell at y=0.20m
    # alongside base_link (inside 0.20 + 0.10 = 0.30m dynamic inflation). Must not trap start!
    rig.obstacle = (-0.1, 0.1, 0.18, 0.25)
    time.sleep(0.25)
    result = rig.plan(4.0)
    assert result.success, f"Parallel passerby trapped robot in dynamic inflation: {result.message}"
    checked = validate(rig, path_xy(result.path))
    assert checked.success, checked.message


@pytest.mark.parametrize("rig", [
    ("-p", "inflation_radius:=0.35", "-p", "path_min_clear_m:=0.10",
     "-p", "obstacle_cost_safe_distance:=0.80", "-p", "obstacle_cost_weight:=8.0",
     "-p", "dynamic_obstacle_cost_safe_distance:=0.35",
     "-p", "dynamic_obstacle_cost_weight:=3.5",
     "-p", "start_heading_penalty_weight:=4.0",
     "-p", "start_heading_penalty_radius_m:=1.5",
     "-p", "path_protect_initial_m:=0.25"),
], indirect=True)
def test_close_dynamic_obstacle_detour_is_smooth_and_recovers_from_safety_wait(rig):
    # 1) Sudden close obstacle ~1.35m ahead: initial 1m detour heading must stay smooth (<45 deg).
    rig.obstacle = (1.30, 1.50, -0.15, 0.15)
    time.sleep(0.25)
    detour = rig.plan(6.0)
    assert detour.success, detour.message
    assert "REQUIRES_ALIGNMENT" not in detour.message, f"Steep sideways plunge: {detour.message}"

    # 2) Even if robot stopped at -100 deg yaw during PATH_SAFETY_WAIT, install_gate must not deadlock.
    rig.obstacle = None
    time.sleep(0.25)
    rig.start_mission(goal=6.0, extra_args=["-p", "install_gate_enable:=true"])
    stops_before = len(rig.stops)
    paths_before = len(rig.paths)
    rig.robot = [0.0, 0.0, math.radians(-100.0)]
    rig.obstacle = (1.30, 1.50, -0.15, 0.15)
    wait_until(lambda: len(rig.stops) > stops_before, timeout=4.0)
    wait_until(lambda: len(rig.paths) > paths_before, timeout=4.0)


@pytest.mark.parametrize("rig", [
    ("-p", "inflation_radius:=0.30", "-p", "path_min_clear_m:=0.10",
     "-p", "goal_relocate_clearance_m:=0.55",
     "-p", "goal_relocate_max_radius_m:=3.5"),
], indirect=True)
def test_blocked_goal_relocates_to_approach_side_scaled_by_obstacle_inflation_size(rig):
    # 1) Small obstacle covering goal=(4.0, 0.0): relocates outside inflation + 0.55m DUNE margin (~0.85m before 3.85).
    rig.obstacle = (3.85, 4.15, -0.15, 0.15)
    time.sleep(0.25)
    res_small = rig.plan(4.0, 0.0)
    assert res_small.success, f"Small blocked goal failed to relocate: {res_small.message}"
    assert "GOAL_RELOCATED" in res_small.message
    checked_small = validate(rig, path_xy(res_small.path))
    assert checked_small.success, checked_small.message
    end_small = res_small.path.poses[-1].pose.position
    assert 2.75 < end_small.x < 3.16, f"Expected approach-side relocation with 0.55m DUNE margin, got x={end_small.x}"
    assert abs(end_small.y) < 0.25

    # 2) Goal placed JUST outside inflation (x=3.45, 0.40m from obstacle 3.85) but inside DUNE repulsion zone:
    # must ALSO push out to comfortable >=0.85m clearance so robot doesn't step in place!
    res_near = rig.plan(3.45, 0.0)
    assert res_near.success and "GOAL_RELOCATED" in res_near.message
    assert res_near.path.poses[-1].pose.position.x < 3.16

    # 3) Large 1.6m x 1.6m obstacle centered on goal=(4.0, 0.0):
    # connected-inflation shell expands automatically and picks the approach-side front face with 0.55m extra margin.
    rig.obstacle = (3.20, 4.80, -0.80, 0.80)
    time.sleep(0.25)
    res_large = rig.plan(4.0, 0.0)
    assert res_large.success, f"Large blocked goal failed to relocate: {res_large.message}"
    assert "GOAL_RELOCATED" in res_large.message
    checked_large = validate(rig, path_xy(res_large.path))
    assert checked_large.success, checked_large.message
    end_large = res_large.path.poses[-1].pose.position
    assert 2.10 < end_large.x < 2.50, f"Expected approach-side front of large obstacle, got x={end_large.x}"
    assert abs(end_large.y) < 0.25

    # 4) Long obstacle (2.60..4.50 x -0.60..0.60): still relocates cleanly to approach side (x < 2.0) with full margin.
    rig.obstacle = (2.60, 4.50, -0.60, 0.60)
    time.sleep(0.25)
    res_table = rig.plan(4.0, 0.0)
    assert res_table.success and "GOAL_RELOCATED" in res_table.message
    assert validate(rig, path_xy(res_table.path)).success
    end_table = res_table.path.poses[-1].pose.position
    assert 1.50 < end_table.x < 1.95 and abs(end_table.y) < 0.30


@pytest.mark.parametrize("rig", [
    ("-p", "inflation_radius:=0.25", "-p", "path_min_clear_m:=0.10"),
], indirect=True)
def test_mission_arrives_and_aligns_yaw_in_place_at_relocated_blocked_waypoint(rig):
    from atomic_move.srv import AlignTo
    from nav_interfaces.msg import MissionStatus

    align_calls = []
    statuses = []

    def on_align(req, resp):
        align_calls.append((req.x, req.y, req.yaw))
        resp.success = True
        resp.message = "OK"
        return resp

    rig.node.create_service(AlignTo, "/atomic_move/align", on_align,
                            callback_group=ReentrantCallbackGroup())
    rig.node.create_subscription(MissionStatus, "/waypoint_manager/status",
                                 lambda m: statuses.append(m.state), 10)

    # Park obstacle right on Waypoint 0 (4.0, 0.0); Waypoint 1 (6.5, 0.0) is clear.
    rig.obstacle = (3.80, 4.20, -0.20, 0.20)
    time.sleep(0.25)

    rig.start_node("mission_manager", "mission_manager_node", [
        "-p", "navigation_mode:=replan_periodic",
        "-p", "periodic_replan_sec:=0.8",
        "-p", "enable_alignment:=true",
        "-p", "handover_settle_sec:=0.05",
        "-p", "nav_stall_timeout_sec:=30.0",
        "-p", "plan_max_retries:=0",
    ])
    start_cli = rig.node.create_client(StartMission, "/waypoint_manager/start_mission")
    assert start_cli.wait_for_service(timeout_sec=10.0)
    req = StartMission.Request()
    wp0 = Waypoint(x=4.0, y=0.0, yaw=math.pi / 2, dwell_sec=0.1, name="blocked_wp")
    wp1 = Waypoint(x=6.5, y=0.0, yaw=0.0, dwell_sec=0.0, name="clear_wp")
    req.waypoints = [wp0, wp1]
    assert rig.call(start_cli, req).success

    wait_until(lambda: len(rig.paths) >= 1, timeout=5.0)
    p0_end = rig.paths[-1].poses[-1].pose.position
    assert p0_end.x < 3.60, f"Path 0 did not relocate outside obstacle: end.x={p0_end.x}"

    # Simulate robot arriving at the relocated goal of Waypoint 0
    rig.robot = [p0_end.x, p0_end.y, 0.0]
    rig.complete_current = True
    wait_until(lambda: len(align_calls) >= 1, timeout=5.0)

    # Alignment MUST target the relocated (x, y) in place — NOT (4.0, 0.0) inside the obstacle!
    ax0, ay0, ayaw0 = align_calls[0]
    assert ax0 == pytest.approx(p0_end.x, abs=0.05), f"Aligned into obstacle x={ax0} instead of relocated {p0_end.x}"
    assert ay0 == pytest.approx(p0_end.y, abs=0.05)
    assert ayaw0 == pytest.approx(math.pi / 2, abs=0.02)
    assert not any(e.event == "WAYPOINT_SKIPPED" for e in rig.events)

    # Mission must advance to Waypoint 1 (6.5, 0.0) and complete!
    wait_until(lambda: any(abs(p.poses[-1].pose.position.x - 6.5) < 0.15 for p in rig.paths), timeout=5.0)
    rig.robot = [6.5, 0.0, 0.0]
    rig.complete_current = True
    wait_until(lambda: "COMPLETED" in statuses, timeout=5.0)


