"""ROS 1 adapter: feeds the map and the map->base pose into ConsoleState."""

from array import array

from web_console.core import yaw_from_quaternion


def run(args, state):
    import rospy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)

    def on_map(msg):
        info = msg.info
        state.set_map(info.width, info.height, info.resolution,
                      info.origin.position.x, info.origin.position.y, array("b", msg.data))

    def on_pose_tick(_event):
        try:
            tf = tf_buffer.lookup_transform(args.map_frame, args.base_frame, rospy.Time(0))
        except tf2_ros.TransformException:
            state.clear_pose()
            return
        t, r = tf.transform.translation, tf.transform.rotation
        state.set_pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    rospy.init_node("web_console", anonymous=False)
    rospy.Subscriber(args.map_topic, OccupancyGrid, on_map, queue_size=1)
    rospy.Timer(rospy.Duration(1.0 / args.pose_hz), on_pose_tick)
    rospy.spin()
