"""ROS 1 adapter: wires the map, pose, mission, safety and camera topics/services to MonitorStateHub."""

import threading
from array import array

from web_console.core import yaw_from_quaternion

CHASSIS_ENABLE_SERVICE = "/sw01_cmd_vel_bridge/enable"
CHASSIS_MOTION_TOPIC = "/sw01_cmd_vel_bridge/motion_enabled"
START_MISSION_SERVICE = "/waypoint_manager/start_mission"


def run(args, hub):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import SetBool, Trigger

    try:
        from nav_interfaces.msg import MissionStatus, Waypoint
        from nav_interfaces.srv import StartMission
    except ImportError:
        MissionStatus = Waypoint = StartMission = None

    rospy.init_node("web_console", anonymous=False)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)
    img_lock = threading.Lock()
    img_sub = {"handle": None}

    def on_map(msg):
        info = msg.info
        hub.console.set_map(info.width, info.height, info.resolution,
                            info.origin.position.x, info.origin.position.y, array("b", msg.data))

    def on_odom(msg):
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        hub.on_odom(p.x, p.y, v.x, v.y, w.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_status(msg):
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

    def on_pose_tick(_event):
        try:
            tf = tf_buffer.lookup_transform(args.map_frame, args.base_frame, rospy.Time(0))
        except tf2_ros.TransformException:
            hub.console.clear_pose()
            return
        t, r = tf.transform.translation, tf.transform.rotation
        hub.console.set_pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    cmd_pub = rospy.Publisher(args.cmd_vel_topic, Twist, queue_size=10)
    neupan_stop_pub = rospy.Publisher("/neupan/stop", Empty, queue_size=10)
    estop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)

    rospy.Subscriber(args.map_topic, OccupancyGrid, on_map, queue_size=1)
    rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=10)
    rospy.Subscriber(args.cmd_vel_topic, Twist, lambda m: hub.on_cmd_vel(m.linear.x, m.linear.y, m.angular.z),
                     queue_size=10)
    rospy.Subscriber("/waypoint_manager/system_health", String,
                     lambda m: hub.update_system_health_json(m.data), queue_size=1)
    rospy.Subscriber(CHASSIS_MOTION_TOPIC, Bool, lambda m: hub.mission.update_chassis(m.data), queue_size=1)
    if MissionStatus is not None:
        rospy.Subscriber("/waypoint_manager/status", MissionStatus, on_status, queue_size=10)
    rospy.Timer(rospy.Duration(1.0 / args.pose_hz), on_pose_tick)
    rospy.Timer(rospy.Duration(0.04), lambda _e: hub.nav.tick())

    def publish_cmd_vel(vx, vy, vyaw):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(vyaw)
        cmd_pub.publish(msg)

    def call_trigger(name):
        try:
            rospy.wait_for_service(name, timeout=2.0)
            res = rospy.ServiceProxy(name, Trigger)()
            return bool(res.success), str(res.message)
        except rospy.ROSException:
            return False, "服务 {} 当前未就绪（导航栈可能未启动）".format(name)
        except rospy.ServiceException as exc:
            return False, str(exc)

    def start_mission(points, loop, temporary):
        if StartMission is None:
            return False, "nav_interfaces 未安装，无法下发任务", ""
        waypoints = [Waypoint(x=p["x"], y=p["y"], yaw=p["yaw"], dwell_sec=p["dwell_sec"], name=p["name"])
                     for p in points]
        try:
            rospy.wait_for_service(START_MISSION_SERVICE, timeout=5.0)
            res = rospy.ServiceProxy(START_MISSION_SERVICE, StartMission)(
                waypoints=waypoints, loop=bool(loop), temporary=bool(temporary))
            return bool(res.success), str(res.message), str(res.mission_id)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, str(exc), ""

    def set_chassis(enabled):
        try:
            rospy.wait_for_service(CHASSIS_ENABLE_SERVICE, timeout=2.0)
            res = rospy.ServiceProxy(CHASSIS_ENABLE_SERVICE, SetBool)(data=bool(enabled))
            return bool(res.success), str(res.message)
        except (rospy.ROSException, rospy.ServiceException):
            return False, "当前平台没有底盘使能接口"

    def subscribe_image():
        with img_lock:
            if img_sub["handle"] is None:
                img_sub["handle"] = rospy.Subscriber(
                    args.image_topic, CompressedImage,
                    lambda m: hub.camera.on_compressed_frame(bytes(m.data)), queue_size=1)

    def unsubscribe_image():
        with img_lock:
            if img_sub["handle"] is not None:
                img_sub["handle"].unregister()
                img_sub["handle"] = None

    hub.bind(
        call_trigger=call_trigger,
        publish_cmd_vel=publish_cmd_vel,
        publish_neupan_stop=lambda: neupan_stop_pub.publish(Empty()),
        publish_estop=lambda active: estop_pub.publish(Bool(data=bool(active))),
        subscribe_image=subscribe_image,
        unsubscribe_image=unsubscribe_image,
        start_mission=start_mission,
        set_chassis=set_chassis,
    )
    rospy.spin()
