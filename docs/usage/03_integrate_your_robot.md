# 03 · Integrate Your Own Robot

This guide describes the interface the stack expects from a robot, and the minimum steps to connect a new one. The only robot-specific code you must write is a **chassis bridge** that turns `/cmd_vel` into your robot's motion commands.

Read [02 · Parameter Tuning](02_parameter_tuning.md) after the first run.

## 1. The interface contract

### What the stack needs from your robot

| Direction | Name | Type | Provided by | Notes |
|---|---|---|---|---|
| In | TF `map → odom → odin1_base_link` | tf | Odin1 driver | Published after relocalization |
| In | TF `odin1_base_link → base_link` | static tf | **You** | Measure the sensor mount |
| In | `/odin1/odometry` | `nav_msgs/Odometry` | Odin1 driver | The stack uses Odin odometry for control; your chassis odometry is not required |
| In | `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | Odin1 driver | Converted to `/scan` by `pointcloud_to_laserscan` |
| In | `/map` | `nav_msgs/OccupancyGrid` | `map_server` | Static grid map (PGM + YAML) |
| Out | `/cmd_vel` | `geometry_msgs/Twist` | Stack (NeuPAN or `atomic_move`) | **Your bridge subscribes** |

Frames: `base_link` is the centre of the footprint on the ground. The stack assumes `+X` forward, `+Y` left, `+Z` up.

### What the stack exposes

| Name | Type | Purpose |
|---|---|---|
| `/waypoint_manager/start_mission` | `nav_interfaces/srv/StartMission` | Start a mission (ROS1: `rosservice`, ROS2: `ros2 service`) |
| `/waypoint_manager/{pause,resume,return_home,cancel,reset}` | `std_srvs/Trigger` | Mission control |
| `/waypoint_manager/status` | `nav_interfaces/msg/MissionStatus` | `state`, `mission_id`, `current_waypoint_idx`, `total_waypoints`, `current_target`, `dwell_remaining_sec`, `progress_percent`, `distance_to_target_m`, `linear_vel`, `angular_vel` |
| `/waypoint_manager/mission_feedback` | `nav_interfaces/msg/MissionEvent` | Event stream |
| `/atomic_move/*` (ROS1) · `/atomic_move_node/*` (ROS2) | `Move`, `Turn`, `AlignTo` | Final alignment and manual checks |

A waypoint is `x`, `y`, `yaw` (rad), `dwell_sec`, and `name`, in the `map` frame.

## 2. Step 1 · Measure the robot

1. **Footprint:** measure the bounding box `L × W` in metres (circumscribed radius `R_circ = hypot(L, W) / 2`).
2. **Sensor mount:** measure the offset from the `base_link` centre to the Odin1 sensor (x forward, z up).
3. **Limits:** note the maximum forward speed, reverse speed, yaw rate, and acceleration your SDK allows.
4. **Clearance:** measure the narrowest passage you need to pass.

## 3. Step 2 · Create the robot profile (`robots/<my_robot>/`)

Copy the template profile and a reference NeuPAN pair:

```bash
cp -r robots/_template robots/my_robot
mkdir -p robots/my_robot/neupan
cp robots/sw01/neupan/planner.yaml robots/my_robot/neupan/planner.yaml
cp robots/sw01/neupan/config.yaml  robots/my_robot/neupan/config.yaml
```

Edit `robots/my_robot/robot.yaml` (see [docs/architecture/ROBOT_PROFILE.md](../architecture/ROBOT_PROFILE.md) for the full schema):

1. `geometry.footprint`: set `mode: polygon` (`length`, `width`, `margin`) or `mode: radial` (`radius`).
2. `geometry.sensor_mount`: set `parent: odin1_base_link`, `child: base_link`, and the measured `x`, `y`, `z`, `yaw` offset.
3. `navigation`: set `inflation_radius` (≤ half the narrowest passage), `obstacle_cost_safe_distance`, `obstacle_cost_weight`, `line_of_sight_safe_distance`, and `min_height` / `max_height`.
4. `bridge`: set your bridge package and launch file under `bridge.ros1` and/or `bridge.ros2`.

Edit `robots/my_robot/neupan/planner.yaml`:

```yaml
robot:
  length: 0.90        # from step 1
  width: 0.50
  max_speed: [0.8, 0.8]
  max_acce: [0.5, 0.5]
ref_speed: 0.4
```

Keep `planner_config_file: "planner.yaml"` and `dune_use_analytic: true` in `robots/my_robot/neupan/config.yaml`. Analytic DUNE adapts to any rectangular footprint without retraining.

## 4. Step 3 · Write the chassis bridge

The bridge must:

1. Subscribe to `/cmd_vel`.
2. **Stop if `/cmd_vel` is stale** (0.25–0.5 s).
3. **Clamp** `linear.x`, `angular.z` to your robot's limits.
4. **Send zero velocity** and stop the SDK's motion when the command is zero.

These two examples are complete and can be used as-is. Replace `send_to_robot()` with your SDK call.

### ROS2 (rclpy)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(v, lim):
    return max(-lim, min(lim, v))


class MyRobotBridge(Node):
    def __init__(self):
        super().__init__('my_robot_bridge')
        self.timeout = self.declare_parameter('cmd_timeout', 0.4).value
        self.max_vx = self.declare_parameter('max_vx', 0.5).value
        self.max_wz = self.declare_parameter('max_wz', 0.8).value
        self.cmd = Twist()
        self.last_rx = self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_rx = self.get_clock().now()

    def _tick(self):
        stale = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9 > self.timeout
        vx = 0.0 if stale else clamp(self.cmd.linear.x, self.max_vx)
        wz = 0.0 if stale else clamp(self.cmd.angular.z, self.max_wz)
        self.send_to_robot(vx, wz)

    def send_to_robot(self, vx, wz):
        # TODO: call your SDK here. Send a stop command when vx == wz == 0.
        pass


def main():
    rclpy.init()
    node = MyRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_to_robot(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### ROS1 (rospy)

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def clamp(v, lim):
    return max(-lim, min(lim, v))


class MyRobotBridge(object):
    def __init__(self):
        self.timeout = rospy.get_param('~cmd_timeout', 0.4)
        self.max_vx = rospy.get_param('~max_vx', 0.5)
        self.max_wz = rospy.get_param('~max_wz', 0.8)
        self.cmd = Twist()
        self.last_rx = rospy.Time.now()
        rospy.Subscriber('/cmd_vel', Twist, self._on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self._tick)  # 50 Hz

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_rx = rospy.Time.now()

    def _tick(self, _event):
        stale = (rospy.Time.now() - self.last_rx).to_sec() > self.timeout
        vx = 0.0 if stale else clamp(self.cmd.linear.x, self.max_vx)
        wz = 0.0 if stale else clamp(self.cmd.angular.z, self.max_wz)
        self.send_to_robot(vx, wz)

    def send_to_robot(self, vx, wz):
        # TODO: call your SDK here. Send a stop command when vx == wz == 0.
        pass


if __name__ == '__main__':
    rospy.init_node('my_robot_bridge')
    MyRobotBridge()
    rospy.spin()
```

Reference implementations in the repository: `ros_ws/src/dobot_ros_control` (ROS1, RPC), `ros_ws/src/unitree_control` (ROS1, Unitree SDK), and `ros2_ws/src/sw01_ros_control` (ROS2, C++).

## 5. Step 4 · Verify the static TF in `robot.yaml`

The offset from `odin1_base_link` to `base_link` comes from `geometry.sensor_mount` in `robots/my_robot/robot.yaml`:

- **Go2 reference (`robots/go2/robot.yaml`):** `x: -0.30, y: 0.0, z: 0.0, yaw: 0.0`
- **SW01 reference (`robots/sw01/robot.yaml`):** `x: -0.42, y: 0.0, z: 0.12, yaw: 0.0`

`ons2_bringup` automatically publishes `odin1_base_link → base_link` from `geometry.sensor_mount` and keeps the Odin-internal frames (`imu`, `odin1_lidar_link`, `odin1_camera_link`) unchanged.

## 6. Step 5 · Launch the stack

`ons2_bringup` reads `robots/<robot>/robot.yaml` via `scripts/lib/robot_profile.py` to wire Static TF, `sliding_map` footprint, `pc_to_scan`, `map_planner`, and the chassis bridge. Use `scripts/ons2_go2.sh` (ROS1) or `scripts/ons2_sw01.sh` (ROS2) as a template for `scripts/ons2_my_robot.sh`, or launch the bringup directly:

### ROS1

1. Set your chassis bridge package and launch file in `robots/my_robot/robot.yaml` under `bridge.ros1`:

   ```yaml
   bridge:
     ros1:
       package: my_robot_control
       launch: my_robot_control.launch
   ```

2. Start `roscore`, the Odin1 driver, and wait for relocalization:

   ```bash
   source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash
   roscore &
   roslaunch odin_ros_driver odin1_ros1.launch
   ```

3. Start the navigation stack with your profile parameters (`robot_profile.py` automatically injects `bridge_pkg`, `bridge_launch`, Static TF, `sliding_map`, and `map_planner` args into `go2.launch`), then `mission_manager`:

   ```bash
   BRINGUP_ARGS="$(python3 scripts/lib/robot_profile.py --ros1-bringup-args robots/my_robot/robot.yaml)"
   roslaunch ons2_bringup go2.launch \
     map_file_name:=NAME \
     navigation_mode:=replan_periodic \
     neupan_config:=$PWD/robots/my_robot/neupan/config.yaml \
     $BRINGUP_ARGS
   roslaunch mission_manager mission_manager.launch navigation_mode:=replan_periodic local_planner:=neupan
   ```

### ROS2

Use one terminal per step, with `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`, `export ROS_DOMAIN_ID=98`, and `export PYTHONPATH=$PWD/scripts/lib:$PWD/NeuPAN:$PYTHONPATH`.

1. Odin1 driver:

   ```bash
   ros2 launch $PWD/ros2_ws/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py
   ```

2. Static map (`map_server` is a lifecycle node; configure and activate it):

   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$PWD/ros2_ws/src/map_planner/maps/NAME.yaml \
     -p topic_name:=map -p frame_id:=map &
   ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
   ```

3. Full navigation stack + bridge + NeuPAN from your profile:

   ```bash
   ros2 launch ons2_bringup bringup.launch.py \
     profile:=$PWD/robots/my_robot/robot.yaml \
     navigation_mode:=replan_periodic \
     periodic_replan_sec:=2.0
   ```

4. Mission manager and `atomic_move`:

   ```bash
   ros2 launch mission_manager mission_manager.launch.py navigation_mode:=replan_periodic local_planner:=neupan
   ros2 launch atomic_move atomic_move.launch.py
   ```

## 7. Optional · Using Your Own Local Planner Instead of NeuPAN
If you want to use your own local planner instead of NeuPAN, after you have implemented your planner, you need to set the `local_planner.backend` to your planner's name in `robots/<robot>/robot.yaml`.

Both **ROS 1 (`ros_ws`)** and **ROS 2 (`ros2_ws`)** decouple the **mission interface contract** (`interface.mode: action | goal_pose`) from the **local planner backend** (`local_planner.backend`).

### How the architecture behaves when you change `local_planner.backend`

| Setting in `robots/<robot>/robot.yaml` | `ons2_bringup` (`ros_ws` & `ros2_ws`) | `mission_manager` (`ros_ws` & `ros2_ws`) |
|---|---|---|
| `backend: neupan` (default) | Launches built-in `neupan_ros` with `robots/<robot>/neupan/config.yaml` | Calls `map_planner` (global A*) and sends `nav_msgs/Path` to `/neupan/nav_path` (`mode: action`) |
| `backend: <my_planner>` with `package` + `launch` | Automatically includes `<package>/launch/<launch>` instead of NeuPAN (in both ROS 1 `go2.launch`/`dobot.launch` and ROS 2 `bringup.launch.py`) | Runs `mode: action` or `mode: goal_pose` according to `backends.<my_planner>.interface.mode` |
| `backend: <my_planner>` without `package` / `launch` | Skips launching NeuPAN; leaves local planner startup to your own terminal/container | Runs `mode: action` or `mode: goal_pose` according to `backends.<my_planner>.interface.mode` |

You can switch the active backend either permanently in `robots/<robot>/robot.yaml` or per-run on the command line:

**Example:**
```bash
bash scripts/ons2_go2.sh --local-planner=my_tracker        # ROS 1 (Go2)
bash scripts/ons2_dobot_lumo.sh --local-planner=my_tracker # ROS 1 (Dobot Lumo)
bash scripts/ons2_sw01.sh --local-planner=my_tracker       # ROS 2 (SW01)
```

---

### Option A · Path-Tracking Local Planner (`mode: action`, e.g. DWA / TEB / MPPI / Pure Pursuit)

Keep global A* (`map_planner`) and `sliding_map`, and replace only the local trajectory tracker.

#### 1. Configure `robots/<my_robot>/robot.yaml`

```yaml
local_planner:
  backend: my_tracker
  backends:
    my_tracker:
      interface:
        mode: action
      # Optional: let ons2_bringup launch your node automatically (omit if you launch it yourself)
      package: my_tracker_pkg
      launch: my_tracker.launch      # ROS 1 (.launch) or ROS 2 (.launch.py)
```

#### 2. Required ROS Interfaces for Local Planner

| Direction | Default ROS Topic / Action / TF | Overridable in `robot.yaml` (`local_planner.backends.<name>.interface`) | Message / Action Type | Description |
|---|---|---|---|---|
| **Action Server** | `/neupan/nav_path` | `nav_path_action` | ROS 1: `nav_interfaces/navPathAction`<br/>ROS 2: `nav_interfaces/action/NavPath` | Receives `goal.path` (`nav_msgs/Path`) and `goal.target_pose` (`geometry_msgs/PoseStamped`) from `mission_manager`; returns `success` on arrival. |
| **Sub (In)** | `/neupan/stop` | `stop_topic` | `std_msgs/Empty` | Immediate stop / preempt signal from `mission_manager`; zero `/cmd_vel` upon receipt. |
| **Sub (In)** | `/scan_360` or `/scan` | `scan_topic` | `sensor_msgs/LaserScan` | 2D obstacle scan (`/scan` or `/scan_360`). |
| **Sub (In)** | `/odin1/odometry` | `odom_topic` | `nav_msgs/Odometry` | Odometry and twist feedback. |
| **TF (In)** | `map -> base_link` | — | `tf2_msgs/TFMessage` | Robot pose in the `map` frame. |
| **Pub (Out)** | `/cmd_vel` | `cmd_vel_topic` (and `bridge.common.cmd_vel_topic`) | `geometry_msgs/Twist` | Velocity command sent to the chassis bridge. |

#### 3. Implement the `/neupan/nav_path` Action Server

Your node receives the global A* path `goal.path` (`nav_msgs/Path`) and `goal.target_pose` (`geometry_msgs/PoseStamped`), publishes `/cmd_vel` (`geometry_msgs/Twist`), and stops on `/neupan/stop` (`std_msgs/Empty`).

##### Minimal ROS 2 (`ros2_ws`, `rclpy`) Skeleton

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_interfaces.action import NavPath


class CustomTrackerRos2(Node):
    def __init__(self):
        super().__init__('custom_local_planner')
        self._abort = False
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Empty, '/neupan/stop', self._on_stop, 10)
        self._server = ActionServer(
            self, NavPath, '/neupan/nav_path',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def _on_stop(self, _msg):
        self._abort = True
        self._cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        self._abort = False
        path = goal_handle.request.path       # nav_msgs/Path from map_planner
        target = goal_handle.request.target_pose
        # TODO: track `path.poses` and publish Twist to `self._cmd_pub` until arrival
        self._cmd_pub.publish(Twist())
        result = NavPath.Result()
        if self._abort or goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            return result
        goal_handle.succeed()
        result.success = True
        result.final_distance = 0.0
        return result


def main():
    rclpy.init()
    rclpy.spin(CustomTrackerRos2())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

##### Minimal ROS 1 (`ros_ws`, `rospy`) Skeleton

```python
#!/usr/bin/env python3
import actionlib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_interfaces.msg import navPathAction, navPathFeedback, navPathResult


class CustomTrackerRos1(object):
    def __init__(self):
        self._abort = False
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/neupan/stop', Empty, self._on_stop, queue_size=1)
        self._server = actionlib.SimpleActionServer(
            '/neupan/nav_path', navPathAction,
            execute_cb=self._execute, auto_start=False,
        )
        self._server.start()

    def _on_stop(self, _msg):
        self._abort = True
        self._cmd_pub.publish(Twist())

    def _execute(self, goal):
        self._abort = False
        path = goal.path                      # nav_msgs/Path from map_planner
        target = goal.target_pose
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._abort or self._server.is_preempt_requested():
                self._cmd_pub.publish(Twist())
                self._server.set_preempted(navPathResult(success=False, message='aborted'))
                return
            # TODO: compute Twist from `path.poses` and publish to `self._cmd_pub`
            # Publish `navPathFeedback(distance_remaining=dist)` while tracking.
            reached = True
            if reached:
                self._cmd_pub.publish(Twist())
                self._server.set_succeeded(navPathResult(success=True, final_distance=0.0))
                return
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('custom_local_planner')
    CustomTrackerRos1()
    rospy.spin()
```

---

### Option B · End-to-End Goal-Pose Planner (`mode: goal_pose`, e.g. SRU / ViNT / RL policy)

Bypass global A* (`map_planner`) and send each waypoint directly to your policy in both `ros_ws` and `ros2_ws`:

1. In `robots/<my_robot>/robot.yaml`:
   ```yaml
   local_planner:
     backend: my_policy
     backends:
       my_policy:
         interface:
           mode: goal_pose
           goal_topic: /my_policy/goal_pose          # Default: /sru/goal_pose
           enable_topic: /my_policy/cmd_vel_enable   # Default: /sru/cmd_vel_enable
           cmd_vel_topic: /my_policy/cmd_vel         # Default: /sru/cmd_vel
         # Optional: include package + launch for automatic bringup
         package: my_policy_pkg
         launch: my_policy.launch
   ```
2. `mission_manager` automatically switches to the `goal_pose` contract and uses your configured topic names:
   - Publishes the active waypoint (`geometry_msgs/PoseStamped`) to `goal_topic` (default `/sru/goal_pose`);
   - Publishes `std_msgs/Bool` to `enable_topic` (default `/sru/cmd_vel_enable`: `true` while navigating, `false` when idle/paused/aligning);
   - Monitors `cmd_vel_topic` and `/odin1/odometry` to detect arrival, executes dwell and `atomic_move` final heading alignment, and advances to the next waypoint.

## 8. Optional · Sharing A* Parameters or Replacing the Global Planner (`map_planner`)

### 8.1 Sharing A* Parameters Across All Robots vs. Per-Robot Overrides

`ons2_bringup` loads global A* parameters in two layers (in both `ros_ws` and `ros2_ws`):

1. **Shared baseline (`map_planner/config/nav_params.yaml`)**: loaded first for every robot.
2. **Per-robot overrides (`robots/<robot>/robot.yaml` -> `navigation:`)**: only keys present in `robot.yaml` override `nav_params.yaml`.

Therefore:
- If all your robots **share the same A* parameters**, edit `ros_ws/src/map_planner/config/nav_params.yaml` (ROS 1) or `ros2_ws/src/map_planner/config/nav_params.yaml` (ROS 2) and omit those keys from `robots/<robot>/robot.yaml`.
- If a specific robot needs a different `inflation_radius` (e.g. a wider chassis), add only `inflation_radius: <value>` under `navigation:` in that robot's `robot.yaml`.

### 8.2 Replacing the Built-In A* Global Planner (`global_planner.backend`)

Just like `local_planner`, the global planner is pluggable across both **ROS 1 (`ros_ws`)** and **ROS 2 (`ros2_ws`)**:

1. In `robots/<my_robot>/robot.yaml` (or via `--global-planner=<name>` on the command line):
   ```yaml
   global_planner:
     backend: my_global_planner
     backends:
       my_global_planner:
         plan_service: /map_planner/plan
         # Optional: include package + launch for automatic bringup (omit if launched externally)
         package: my_global_planner_pkg
         launch: my_global_planner.launch   # `.launch` for ROS 1, or `.launch.py` for ROS 2
   ```
2. Your global planner node implements the global planning interface below:

#### Required ROS Interfaces for Global Planner

| Direction | Default ROS Service / Topic / TF | Overridable in `robot.yaml` (`global_planner.backends.<name>`) | Type | Description |
|---|---|---|---|---|
| **Service Server** | `/map_planner/plan` | `plan_service` | `nav_interfaces/srv/PlanPath` | Takes `req.start` and `req.goal` (`geometry_msgs/PoseStamped`); returns `res.success`, `res.message`, and `res.path` (`nav_msgs/Path` in `map` frame). |
| **Sub (In)** | `/map` | — | `nav_msgs/OccupancyGrid` | Static 2D occupancy grid map from `map_server`. |
| **Sub (In)** | `/sliding_map` | — | `nav_msgs/OccupancyGrid` | Local dynamic obstacle occupancy grid from `sliding_map_node`. |
| **TF (In)** | `map -> base_link` | — | `tf2_msgs/TFMessage` | Robot pose lookup when `req.use_start == false`. |
| **Pub (Out)** | `/initial_path` | — | `nav_msgs/Path` | Publishes the planned global path. |

#### Minimal ROS 2 (`ros2_ws`, `rclpy`) Global Planner Skeleton

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_interfaces.srv import PlanPath


class CustomGlobalPlannerRos2(Node):
    def __init__(self):
        super().__init__('custom_global_planner')
        self._path_pub = self.create_publisher(Path, '/initial_path', 1)
        self.create_service(PlanPath, '/map_planner/plan', self._on_plan)

    def _on_plan(self, req, res):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        # TODO: compute waypoints from req.start to req.goal (e.g. JPS / Theta* / RRT* / topological graph)
        path.poses = [req.start, req.goal]
        self._path_pub.publish(path)
        res.success = True
        res.message = 'OK'
        res.path = path
        return res


def main():
    rclpy.init()
    rclpy.spin(CustomGlobalPlannerRos2())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Minimal ROS 1 (`ros_ws`, `rospy`) Global Planner Skeleton

```python
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from nav_interfaces.srv import PlanPath, PlanPathResponse


class CustomGlobalPlannerRos1(object):
    def __init__(self):
        self._path_pub = rospy.Publisher('/initial_path', Path, queue_size=1, latch=True)
        rospy.Service('/map_planner/plan', PlanPath, self._on_plan)

    def _on_plan(self, req):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        # TODO: compute waypoints from req.start to req.goal
        path.poses = [req.start, req.goal]
        self._path_pub.publish(path)
        return PlanPathResponse(success=True, message='OK', path=path)


if __name__ == '__main__':
    rospy.init_node('custom_global_planner')
    CustomGlobalPlannerRos1()
    rospy.spin()
```

## 9. Step 6 · Bring-up checks

Verify each layer from the bottom up. Keep the robot on a gantry or in an open area with an operator at the emergency stop.

1. **TF:** `ros2 run tf2_ros tf2_echo map base_link` (ROS2) or `rosrun tf tf_echo map base_link` (ROS1). Rotate the robot in place: obstacles in `/scan_360` must not rotate with it.
2. **Bridge:** publish a slow command and confirm the robot moves in the expected direction:

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```

   Stop the publisher. The robot must stop within the `cmd_timeout` you set.
3. **Alignment:** call `atomic_move` `move` (0.5 m forward) and `turn` (90° left) and check the signs.
4. **One waypoint:** start a mission 2–3 m ahead in open space:

   ```bash
   ros2 service call /waypoint_manager/start_mission nav_interfaces/srv/StartMission \
     "{waypoints: [{x: 2.0, y: 0.0, yaw: 0.0, dwell_sec: 0.0, name: 'test'}], loop: false, temporary: false}"
   ```

5. **Tune:** start from [02 · Parameter Tuning](02_parameter_tuning.md), section 3.

## 10. Checklist

- [ ] Footprint, sensor mount, and speed limits measured
- [ ] NeuPAN profile pair created, `dune_use_analytic: true`
- [ ] `sliding_map` footprint and `inflation_radius` updated
- [ ] Chassis bridge handles stale commands and clamps velocity
- [ ] Static TF `odin1_base_link → base_link` matches the mount
- [ ] TF tree checked: `map → odom → odin1_base_link → base_link`
- [ ] Bridge tested with a slow command and a stop
- [ ] Single-waypoint mission succeeded in open space

## 11. Optional · Add inspection perception

Navigation answers *how to get there*; inspection answers *what is there*. The inspection
component is a **separate open-source repository** and is deliberately not merged into this
ROS workspace:

**➡️ [Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)** — Florence-2 based
structured inspection (valve state, gauge readings, indicator color, nameplate OCR), driven by a
YAML/JSON checklist.

It attaches over HTTP in two processes, so the model environment stays isolated from ROS:

```
/odin1/image/undistorted ──► capture node (:8730) ──HTTP──► perception-run (Florence-2)
                                     ▲                              │
                                     └──── POST /annotate ◄─────────┘
                                     │
                  /inspection/annotated/compressed ──► Foxglove / RViz / rosbridge
```

**Installation and the step-by-step procedure are in that repository's README** — its
*Integrating with Odin-Nav-Stack2* section covers the topic contract, annotated-image publishing,
and the ROS 1 / Jetson pitfalls. Do not duplicate that setup here.

Prerequisite on this side: your driver must actually publish images — `sendrgb: 1` and
`sendrgbundistort: 1` in the driver config — verified with
`rostopic hz /odin1/image/undistorted` before wiring perception in.
