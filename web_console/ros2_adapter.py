"""ROS 2 adapter: feeds the map and the map->base pose into ConsoleState."""

from array import array

from web_console.core import yaw_from_quaternion


def run(args, state):
    import rclpy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time

    class WebConsoleNode(Node):
        def __init__(self):
            super().__init__("web_console")
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, latched)
            self.create_timer(1.0 / args.pose_hz, self._on_pose_tick)

        def _on_map(self, msg):
            info = msg.info
            state.set_map(info.width, info.height, info.resolution,
                          info.origin.position.x, info.origin.position.y, array("b", msg.data))

        def _on_pose_tick(self):
            try:
                tf = self.tf_buffer.lookup_transform(args.map_frame, args.base_frame, Time())
            except tf2_ros.TransformException:
                state.clear_pose()
                return
            t, r = tf.transform.translation, tf.transform.rotation
            state.set_pose(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    rclpy.init()
    node = WebConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
