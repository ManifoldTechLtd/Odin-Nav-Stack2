"""ROS 2 adapter: wires the map, pose, mission, safety and camera topics/services to MonitorStateHub."""

import signal
import subprocess
import threading
import time
from array import array

from web_console.core import yaw_from_quaternion

MAP_SERVER_GET_MAP = "/map_server/map"
TRIGGER_SERVICES = (
    "/waypoint_manager/pause", "/waypoint_manager/resume", "/waypoint_manager/cancel",
    "/waypoint_manager/skip", "/waypoint_manager/return_home", "/waypoint_manager/reset",
    "/waypoint/goal/skip", "/waypoint/goal/reset",
)
START_MISSION_SERVICE = "/waypoint_manager/start_mission"


def _non_loopback_ipv4s():
    try:
        res = subprocess.run(["ip", "-4", "-o", "addr", "show", "up"],
                             capture_output=True, text=True, timeout=2)
        ips = set()
        for line in (res.stdout or "").splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[1] != "lo":
                ips.add(parts[3].split("/")[0])
        return frozenset(ips)
    except Exception:
        return frozenset()


def run(args, hub):
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav_msgs.srv import GetMap
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import Trigger

    try:
        from nav_interfaces.msg import MissionStatus, Waypoint
        from nav_interfaces.srv import StartMission
    except ImportError:
        MissionStatus = Waypoint = StartMission = None

    class WebConsoleNode(Node):
        def __init__(self, context):
            super().__init__("web_console", context=context)
            self._cb = ReentrantCallbackGroup()
            self._img_sub = None
            self._img_lock = threading.Lock()
            self._map_req_inflight = False
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)

            self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, latched)
            self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos_profile_sensor_data)
            self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd, qos_profile_sensor_data)
            self.create_subscription(String, "/waypoint_manager/system_health", self._on_health,
                                     qos_profile_sensor_data)
            if MissionStatus is not None:
                self.create_subscription(MissionStatus, "/waypoint_manager/status", self._on_status,
                                         qos_profile_sensor_data)

            self._cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
            self._neupan_stop_pub = self.create_publisher(Empty, "/neupan/stop", 10)
            self._estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

            self._triggers = {n: self.create_client(Trigger, n, callback_group=self._cb) for n in TRIGGER_SERVICES}
            self._map_client = self.create_client(GetMap, MAP_SERVER_GET_MAP, callback_group=self._cb)
            self._start_client = (self.create_client(StartMission, START_MISSION_SERVICE, callback_group=self._cb)
                                  if StartMission is not None else None)
            self.create_timer(1.0 / args.pose_hz, self._on_pose_tick)
            self.create_timer(2.0, self._poll_map_service, callback_group=self._cb)
            self.create_timer(0.04, lambda: hub.nav.tick())

        def _on_map(self, msg):
            info = msg.info
            if info.width > 0 and info.height > 0 and len(msg.data) >= info.width * info.height:
                hub.console.set_map(info.width, info.height, info.resolution,
                                    info.origin.position.x, info.origin.position.y, array("b", msg.data))

        def _poll_map_service(self):
            if hub.console.snapshot()["map"] is not None or self._map_req_inflight:
                return
            if not self._map_client.service_is_ready():
                return
            self._map_req_inflight = True
            fut = self._map_client.call_async(GetMap.Request())

            def _done(f):
                self._map_req_inflight = False
                try:
                    res = f.result()
                    if res is not None:
                        self._on_map(res.map)
                except Exception:
                    pass

            fut.add_done_callback(_done)

        def _on_pose_tick(self):
            try:
                tf = self.tf_buffer.lookup_transform(args.map_frame, args.base_frame, Time())
            except tf2_ros.TransformException:
                hub.console.clear_pose()
                return
            t, r = tf.transform.translation, tf.transform.rotation
            hub.console.set_pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

        def _on_odom(self, msg):
            p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
            hub.on_odom(p.x, p.y, v.x, v.y, w.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

        def _on_cmd(self, msg):
            hub.on_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

        def _on_health(self, msg):
            hub.update_system_health_json(msg.data)

        def _on_status(self, msg):
            tgt = msg.current_target
            hub.on_mission_status({
                "state": msg.state,
                "mission_id": msg.mission_id,
                "current_waypoint_idx": int(msg.current_waypoint_idx),
                "total_waypoints": int(msg.total_waypoints),
                "distance_to_target_m": float(msg.distance_to_target_m),
                "progress_percent": float(msg.progress_percent),
                "dwell_remaining_sec": float(msg.dwell_remaining_sec),
                "target": {"x": tgt.position.x, "y": tgt.position.y,
                           "yaw": yaw_from_quaternion(tgt.orientation.x, tgt.orientation.y,
                                                      tgt.orientation.z, tgt.orientation.w)},
            })

        def _on_image(self, msg):
            hub.camera.on_compressed_frame(bytes(msg.data))

        def publish_cmd_vel(self, vx, vy, vyaw):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(vyaw)
            self._cmd_pub.publish(msg)

        def publish_neupan_stop(self):
            self._neupan_stop_pub.publish(Empty())

        def publish_estop(self, active):
            self._estop_pub.publish(Bool(data=bool(active)))

        def subscribe_image(self):
            with self._img_lock:
                if self._img_sub is None:
                    self._img_sub = self.create_subscription(CompressedImage, args.image_topic, self._on_image,
                                                             qos_profile_sensor_data)

        def unsubscribe_image(self):
            with self._img_lock:
                if self._img_sub is not None:
                    self.destroy_subscription(self._img_sub)
                    self._img_sub = None

        def _call(self, name, client, request, timeout=5.0):
            if not client.wait_for_service(timeout_sec=2.0):
                return None, "服务 {} 当前未就绪（导航栈可能未启动）".format(name)
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if future.done():
                    return future.result(), None
                time.sleep(0.02)
            return None, "调用服务 {} 超时".format(name)

        def list_topics(self):
            return {name: list(types) for name, types in self.get_topic_names_and_types()}

        def call_trigger(self, name):
            client = self._triggers.get(name)
            if client is None:
                return False, "未知服务 {}".format(name)
            res, err = self._call(name, client, Trigger.Request(), timeout=2.0)
            if err:
                return False, err
            return bool(res.success), str(res.message)

        def start_mission(self, points, loop, temporary):
            if self._start_client is None:
                return False, "nav_interfaces 未安装，无法下发任务", ""
            req = StartMission.Request()
            req.waypoints = []
            for p in points:
                wp = Waypoint()
                wp.x, wp.y, wp.yaw = p["x"], p["y"], p["yaw"]
                wp.dwell_sec, wp.name = p["dwell_sec"], p["name"]
                req.waypoints.append(wp)
            req.loop, req.temporary = bool(loop), bool(temporary)
            res, err = self._call(START_MISSION_SERVICE, self._start_client, req, timeout=15.0)
            if err:
                return False, err, ""
            return bool(res.success), str(res.message), str(res.mission_id)

    stop_all = False

    def _on_sigterm(_signum, _frame):
        nonlocal stop_all
        stop_all = True

    try:
        signal.signal(signal.SIGTERM, _on_sigterm)
    except ValueError:
        pass
    while not stop_all:
        ctx = Context()
        rclpy.init(context=ctx)
        boot_ips = _non_loopback_ipv4s()
        node = WebConsoleNode(ctx)
        hub.bind(call_trigger=node.call_trigger, publish_cmd_vel=node.publish_cmd_vel,
                 publish_neupan_stop=node.publish_neupan_stop, publish_estop=node.publish_estop,
                 subscribe_image=node.subscribe_image, unsubscribe_image=node.unsubscribe_image,
                 start_mission=node.start_mission, list_topics=node.list_topics)
        if hub.camera.enabled:
            node.subscribe_image()
        executor = MultiThreadedExecutor(num_threads=4, context=ctx)
        executor.add_node(node)
        rebind = threading.Event()

        def _watch_dds():
            no_pose_since = None
            while ctx.ok() and not rebind.is_set():
                time.sleep(3.0)
                cur_ips = _non_loopback_ipv4s()
                if not boot_ips and cur_ips:
                    rebind.set()
                    break
                st_active = hub.stack.status().get("session_active", False)
                has_pose = hub.console.snapshot()["pose"] is not None
                if st_active and not has_pose:
                    if no_pose_since is None:
                        no_pose_since = time.monotonic()
                    elif time.monotonic() - no_pose_since > 12.0:
                        rebind.set()
                        break
                else:
                    no_pose_since = None

        threading.Thread(target=_watch_dds, daemon=True).start()
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            while ctx.ok() and not rebind.is_set() and not stop_all:
                time.sleep(0.2)
            if not rebind.is_set():
                stop_all = True
        except KeyboardInterrupt:
            stop_all = True
        finally:
            rebind.set()
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            if ctx.ok():
                try:
                    rclpy.shutdown(context=ctx)
                except Exception:
                    pass
            spin_thread.join(timeout=2.0)
