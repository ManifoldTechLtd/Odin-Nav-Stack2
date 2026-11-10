# 03 · 接入你自己的机器人

[English](03_integrate_your_robot.md) | **中文**

本文说明导航栈对机器人的接口要求，以及接入一台新机器人的最小步骤。你唯一需要写的机器人专属代码就是一个**底盘桥**：把 `/cmd_vel` 转成你机器人的运动指令。

跑通之后请继续读 [02 · 参数调优](02_parameter_tuning.zh-CN.md)。

## 1. 接口契约

### 栈需要你的机器人提供什么

| 方向 | 名称 | 类型 | 由谁提供 | 说明 |
|---|---|---|---|---|
| 入 | TF `map → odom → odin1_base_link` | tf | Odin1 驱动 | 重定位成功后发布 |
| 入 | TF `odin1_base_link → base_link` | 静态 TF | **你** | 测量传感器安装位置 |
| 入 | `/odin1/odometry` | `nav_msgs/Odometry` | Odin1 驱动 | 栈用 Odin 里程计做控制；不要求你的底盘里程计 |
| 入 | `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | Odin1 驱动 | 由 `pointcloud_to_laserscan` 转成 `/scan` |
| 入 | `/map` | `nav_msgs/OccupancyGrid` | `map_server` | 静态栅格地图（PGM + YAML） |
| 出 | `/cmd_vel` | `geometry_msgs/Twist` | 栈（NeuPAN 或 `atomic_move`） | **由你的桥订阅** |

坐标系：`base_link` 是机器人外形在地面上的中心。栈假定 `+X` 向前、`+Y` 向左、`+Z` 向上。

### 栈对外暴露什么

| 名称 | 类型 | 用途 |
|---|---|---|
| `/waypoint_manager/start_mission` | `nav_interfaces/srv/StartMission` | 启动任务（ROS1 用 `rosservice`，ROS2 用 `ros2 service`） |
| `/waypoint_manager/{pause,resume,return_home,cancel,reset}` | `std_srvs/Trigger` | 任务控制 |
| `/waypoint_manager/status` | `nav_interfaces/msg/MissionStatus` | `state`、`mission_id`、`current_waypoint_idx`、`total_waypoints`、`current_target`、`dwell_remaining_sec`、`progress_percent`、`distance_to_target_m`、`linear_vel`、`angular_vel` |
| `/waypoint_manager/mission_feedback` | `nav_interfaces/msg/MissionEvent` | 事件流 |
| `/atomic_move/*`（ROS1）· `/atomic_move_node/*`（ROS2） | `Move`、`Turn`、`AlignTo` | 末段对齐与手动验证 |

一个航点包含 `x`、`y`、`yaw`（弧度）、`dwell_sec` 与 `name`，都在 `map` 坐标系下。

## 2. 步骤 1 · 测量机器人

1. **外形：** 量出包围盒 `L × W`（米），外接圆半径 `R_circ = hypot(L, W) / 2`。
2. **传感器安装：** 量出 `base_link` 中心到 Odin1 传感器的偏移（x 向前、z 向上）。
3. **限幅：** 记下你 SDK 允许的最大前进速度、后退速度、偏航角速度与加速度。
4. **净空：** 量出你需要通过的最窄通道。

## 3. 步骤 2 · 创建平台档案（`robots/<my_robot>/`）

复制模板档案和一套参考 NeuPAN 配置：

```bash
cp -r robots/_template robots/my_robot
mkdir -p robots/my_robot/neupan
cp robots/sw01/neupan/planner.yaml robots/my_robot/neupan/planner.yaml
cp robots/sw01/neupan/config.yaml  robots/my_robot/neupan/config.yaml
```

编辑 `robots/my_robot/robot.yaml`（完整 schema 见 [docs/architecture/ROBOT_PROFILE.md](../architecture/ROBOT_PROFILE.md)）：

1. `geometry.footprint`：设 `mode: polygon`（`length`、`width`、`margin`）或 `mode: radial`（`radius`）。
2. `geometry.sensor_mount`：设 `parent: odin1_base_link`、`child: base_link`，以及实测的 `x`、`y`、`z`、`yaw` 偏移。
3. `navigation`：设 `inflation_radius`（≤ 最窄通道的一半）、`obstacle_cost_safe_distance`、`obstacle_cost_weight`、`line_of_sight_safe_distance`，以及 `min_height` / `max_height`。
4. `bridge`：把你的底盘桥包名与 launch 文件填到 `bridge.ros1` 和/或 `bridge.ros2` 下。

编辑 `robots/my_robot/neupan/planner.yaml`：

```yaml
robot:
  length: 0.90        # from step 1
  width: 0.50
  max_speed: [0.8, 0.8]
  max_acce: [0.5, 0.5]
ref_speed: 0.4
```

`robots/my_robot/neupan/config.yaml` 里保持 `planner_config_file: "planner.yaml"` 与 `dune_use_analytic: true`。解析版 DUNE 能自动适配任意矩形外形，不需要重训。

## 4. 步骤 3 · 写底盘桥

桥必须做到：

1. 订阅 `/cmd_vel`。
2. **`/cmd_vel` 断流就停**（0.25–0.5 s）。
3. **限幅** `linear.x`、`angular.z` 到机器人允许的范围。
4. 指令为零时**下发零速度**并停止 SDK 的运动。

下面两个例子是完整的，可以直接用。把你自己的 SDK 调用替换进 `send_to_robot()` 即可。

### ROS2（rclpy）

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

### ROS1（rospy）

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

仓库里的参考实现：`ros_ws/src/dobot_ros_control`（ROS1，RPC）、`ros_ws/src/unitree_control`（ROS1，Unitree SDK）、`ros2_ws/src/sw01_ros_control`（ROS2，C++）。

## 5. 步骤 4 · 验证 `robot.yaml` 里的静态 TF

从 `odin1_base_link` 到 `base_link` 的偏移来自 `robots/my_robot/robot.yaml` 的 `geometry.sensor_mount`：

- **Go2 参考（`robots/go2/robot.yaml`）：** `x: -0.30, y: 0.0, z: 0.0, yaw: 0.0`
- **SW01 参考（`robots/sw01/robot.yaml`）：** `x: -0.42, y: 0.0, z: 0.12, yaw: 0.0`

`ons2_bringup` 会根据 `geometry.sensor_mount` 自动发布 `odin1_base_link → base_link`，并保持 Odin 内部的坐标系（`imu`、`odin1_lidar_link`、`odin1_camera_link`）不变。

## 6. 步骤 5 · 启动栈

`ons2_bringup` 通过 `scripts/lib/robot_profile.py` 读 `robots/<robot>/robot.yaml`，据此串联静态 TF、`sliding_map` 外形、`pc_to_scan`、`map_planner` 与底盘桥。可以拿 `scripts/ons2_go2.sh`（ROS1）或 `scripts/ons2_sw01.sh`（ROS2）当模板写 `scripts/ons2_my_robot.sh`，也可以直接起 bringup：

### ROS1

1. 在 `robots/my_robot/robot.yaml` 的 `bridge.ros1` 下填你的底盘桥包与 launch：

   ```yaml
   bridge:
     ros1:
       package: my_robot_control
       launch: my_robot_control.launch
   ```

2. 起 `roscore`、Odin1 驱动，等重定位完成：

   ```bash
   source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash
   roscore &
   roslaunch odin_ros_driver odin1_ros1.launch
   ```

3. 用你的档案参数启动导航栈（`robot_profile.py` 会自动把 `bridge_pkg`、`bridge_launch`、静态 TF、`sliding_map`、`map_planner` 参数注入 `go2.launch`），再起 `mission_manager`：

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

每一步一个终端，都先 `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`、`export ROS_DOMAIN_ID=98`、`export PYTHONPATH=$PWD/scripts/lib:$PWD/NeuPAN:$PYTHONPATH`。

1. Odin1 驱动：

   ```bash
   ros2 launch $PWD/ros2_ws/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py
   ```

2. 静态地图（`map_server` 是 lifecycle 节点，要 configure 再 activate）：

   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$PWD/ros2_ws/src/map_planner/maps/NAME.yaml \
     -p topic_name:=map -p frame_id:=map &
   ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
   ```

3. 整套导航栈 + 底盘桥 + NeuPAN（按你的档案）：

   ```bash
   ros2 launch ons2_bringup bringup.launch.py \
     profile:=$PWD/robots/my_robot/robot.yaml \
     navigation_mode:=replan_periodic \
     periodic_replan_sec:=2.0
   ```

4. Mission manager 与 `atomic_move`：

   ```bash
   ros2 launch mission_manager mission_manager.launch.py navigation_mode:=replan_periodic local_planner:=neupan
   ros2 launch atomic_move atomic_move.launch.py
   ```

## 7. 可选 · 用自己的局部规划器替代 NeuPAN

如果你要用自己的局部规划器替代 NeuPAN，实现完之后，在 `robots/<robot>/robot.yaml` 里把 `local_planner.backend` 设成你规划器的名字。

**ROS 1（`ros_ws`）** 与 **ROS 2（`ros2_ws`）** 都把**任务接口契约**（`interface.mode: action | goal_pose`）与**局部规划器后端**（`local_planner.backend`）解耦了。

### 改 `local_planner.backend` 后架构怎么表现

| `robots/<robot>/robot.yaml` 里的设置 | `ons2_bringup`（`ros_ws` 与 `ros2_ws`） | `mission_manager`（`ros_ws` 与 `ros2_ws`） |
|---|---|---|
| `backend: neupan`（默认） | 用 `robots/<robot>/neupan/config.yaml` 起内置 `neupan_ros` | 调 `map_planner`（全局 A\*），把 `nav_msgs/Path` 发到 `/neupan/nav_path`（`mode: action`） |
| `backend: <my_planner>` 且有 `package` + `launch` | 自动 include `<package>/launch/<launch>` 替代 NeuPAN（ROS 1 的 `go2.launch`/`dobot.launch` 与 ROS 2 的 `bringup.launch.py` 都支持） | 按 `backends.<my_planner>.interface.mode` 走 `mode: action` 或 `mode: goal_pose` |
| `backend: <my_planner>` 但没有 `package` / `launch` | 跳过 NeuPAN 启动，局部规划器的启动交给你自己的终端/容器 | 同上，按 `interface.mode` 决定 |

后端可以永久改在 `robots/<robot>/robot.yaml`，也可以单次运行时在命令行切换：

**示例：**

```bash
bash scripts/ons2_go2.sh --local-planner=my_tracker        # ROS 1 (Go2)
bash scripts/ons2_dobot_lumo.sh --local-planner=my_tracker # ROS 1 (Dobot Lumo)
bash scripts/ons2_sw01.sh --local-planner=my_tracker       # ROS 2 (SW01)
```

---

### 方式 A · 路径跟踪型局部规划器（`mode: action`，如 DWA / TEB / MPPI / Pure Pursuit）

保留全局 A\*（`map_planner`）与 `sliding_map`，只替换局部轨迹跟踪器。

#### 1. 配置 `robots/<my_robot>/robot.yaml`

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

#### 2. 局部规划器需要的 ROS 接口

| 方向 | 默认话题 / Action / TF | 可在 `robot.yaml`（`local_planner.backends.<name>.interface`）覆盖 | 消息 / Action 类型 | 说明 |
|---|---|---|---|---|
| **Action Server** | `/neupan/nav_path` | `nav_path_action` | ROS 1：`nav_interfaces/navPathAction`<br/>ROS 2：`nav_interfaces/action/NavPath` | 从 `mission_manager` 收 `goal.path`（`nav_msgs/Path`）与 `goal.target_pose`（`geometry_msgs/PoseStamped`）；到达后返回 `success` |
| **订阅（入）** | `/neupan/stop` | `stop_topic` | `std_msgs/Empty` | `mission_manager` 的立即停止/抢占信号；收到即把 `/cmd_vel` 置零 |
| **订阅（入）** | `/scan_360` 或 `/scan` | `scan_topic` | `sensor_msgs/LaserScan` | 2D 障碍扫描 |
| **订阅（入）** | `/odin1/odometry` | `odom_topic` | `nav_msgs/Odometry` | 里程计与速度反馈 |
| **TF（入）** | `map -> base_link` | — | `tf2_msgs/TFMessage` | 机器人在 `map` 系的位姿 |
| **发布（出）** | `/cmd_vel` | `cmd_vel_topic`（以及 `bridge.common.cmd_vel_topic`） | `geometry_msgs/Twist` | 发给底盘桥的速度指令 |

#### 3. 实现 `/neupan/nav_path` Action Server

你的节点接收全局 A\* 路径 `goal.path`（`nav_msgs/Path`）与 `goal.target_pose`（`geometry_msgs/PoseStamped`），发布 `/cmd_vel`（`geometry_msgs/Twist`），并在收到 `/neupan/stop`（`std_msgs/Empty`）时停止。

##### 最小 ROS 2（`ros2_ws`，`rclpy`）骨架

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

##### 最小 ROS 1（`ros_ws`，`rospy`）骨架

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

### 方式 B · 端到端目标位姿型规划器（`mode: goal_pose`，如 SRU / ViNT / RL 策略）

绕过全局 A\*（`map_planner`），在 `ros_ws` 与 `ros2_ws` 里都直接把航点交给你的策略：

1. 在 `robots/<my_robot>/robot.yaml` 里：

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

2. `mission_manager` 会自动切到 `goal_pose` 契约并使用你配置的话题名：
   - 把当前航点（`geometry_msgs/PoseStamped`）发到 `goal_topic`（默认 `/sru/goal_pose`）；
   - 把 `std_msgs/Bool` 发到 `enable_topic`（默认 `/sru/cmd_vel_enable`：导航中为 `true`，空闲/暂停/对齐时为 `false`）；
   - 监视 `cmd_vel_topic` 与 `/odin1/odometry` 判断是否到达，执行停留与 `atomic_move` 的末段航向对齐，然后推进到下一个航点。

## 8. 可选 · 共享 A\* 参数或替换全局规划器（`map_planner`）

### 8.1 所有机器人共享 A\* 参数 vs 单机覆盖

`ons2_bringup` 分两层加载全局 A\* 参数（`ros_ws` 与 `ros2_ws` 都是）：

1. **共享基线（`map_planner/config/nav_params.yaml`）**：每台机器人都会先加载。
2. **单机覆盖（`robots/<robot>/robot.yaml` 的 `navigation:`）**：只有 `robot.yaml` 里出现的键才会覆盖 `nav_params.yaml`。

因此：

- 如果所有机器人**共用同一套 A\* 参数**，直接改 `ros_ws/src/map_planner/config/nav_params.yaml`（ROS 1）或 `ros2_ws/src/map_planner/config/nav_params.yaml`（ROS 2），并从各个 `robots/<robot>/robot.yaml` 里删掉这些键。
- 如果某台机器人需要不同的 `inflation_radius`（比如车更宽），只在那台机器的 `robot.yaml` 的 `navigation:` 下加 `inflation_radius: <值>` 即可。

### 8.2 替换内置 A\* 全局规划器（`global_planner.backend`）

和 `local_planner` 一样，全局规划器在 **ROS 1（`ros_ws`）** 与 **ROS 2（`ros2_ws`）** 上都是可插拔的：

1. 在 `robots/<my_robot>/robot.yaml` 里（或命令行加 `--global-planner=<name>`）：

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

2. 你的全局规划器节点实现下面这套全局规划接口：

| 方向 | 默认服务 / 话题 / TF | 可在 `robot.yaml`（`global_planner.backends.<name>`）覆盖 | 类型 | 说明 |
|---|---|---|---|---|
| **Service Server** | `/map_planner/plan` | `plan_service` | `nav_interfaces/srv/PlanPath` | 接收 `req.start` 与 `req.goal`（`geometry_msgs/PoseStamped`）；返回 `res.success`、`res.message` 与 `res.path`（`map` 系的 `nav_msgs/Path`） |
| **订阅（入）** | `/map` | — | `nav_msgs/OccupancyGrid` | `map_server` 提供的静态 2D 占据栅格 |
| **订阅（入）** | `/sliding_map` | — | `nav_msgs/OccupancyGrid` | `sliding_map_node` 提供的局部动态障碍栅格 |
| **TF（入）** | `map -> base_link` | — | `tf2_msgs/TFMessage` | 当 `req.use_start == false` 时查询机器人位姿 |
| **发布（出）** | `/initial_path` | — | `nav_msgs/Path` | 发布规划出的全局路径 |

#### 最小 ROS 2（`ros2_ws`，`rclpy`）全局规划器骨架

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

#### 最小 ROS 1（`ros_ws`，`rospy`）全局规划器骨架

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

## 9. 步骤 6 · 上线检查

自下而上逐层验证。机器人务必架在台架上，或放在空旷场地且有人守在急停旁边。

1. **TF：** `ros2 run tf2_ros tf2_echo map base_link`（ROS2）或 `rosrun tf tf_echo map base_link`（ROS1）。原地转动机器人：`/scan_360` 里的障碍物**不应该**跟着转。
2. **底盘桥：** 下发一个慢速指令，确认机器人往预期方向动：

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```

   停掉发布者。机器人必须在你设定的 `cmd_timeout` 内停下。

3. **对齐：** 调 `atomic_move` 的 `move`（前进 0.5 m）与 `turn`（左转 90°），检查方向符号。
4. **单个航点：** 在空旷处起步，发一个前方 2–3 m 的航点：

   ```bash
   ros2 service call /waypoint_manager/start_mission nav_interfaces/srv/StartMission \
     "{waypoints: [{x: 2.0, y: 0.0, yaw: 0.0, dwell_sec: 0.0, name: 'test'}], loop: false, temporary: false}"
   ```

5. **调参：** 从 [02 · 参数调优](02_parameter_tuning.zh-CN.md) 的第 3 节开始。

## 10. 检查清单

- [ ] 外形、传感器安装位置与速度限幅已实测
- [ ] NeuPAN 配置对已创建，`dune_use_analytic: true`
- [ ] `sliding_map` 外形与 `inflation_radius` 已更新
- [ ] 底盘桥能处理断流并做速度限幅
- [ ] 静态 TF `odin1_base_link → base_link` 与实测安装一致
- [ ] TF 树已核验：`map → odom → odin1_base_link → base_link`
- [ ] 桥已用慢速指令与停止指令验证过
- [ ] 空旷场地单航点任务成功

## 11. 可选 · 接入巡检感知

导航回答的是*怎么去*；巡检回答的是*那里有什么*。巡检感知是一个**独立的开源仓库**，刻意不合并进这个 ROS 工作区：

**➡️ [Odin-perception](https://github.com/ManifoldTechLtd/Odin-perception)** —— 基于 Florence-2 的结构化巡检（阀门状态、仪表读数、指示灯颜色、铭牌 OCR），由 YAML/JSON 清单驱动。

它通过 HTTP 以两个进程对接，模型环境因此与 ROS 保持隔离：

```
/odin1/image/undistorted ──► capture node (:8730) ──HTTP──► perception-run (Florence-2)
                                     ▲                              │
                                     └──── POST /annotate ◄─────────┘
                                     │
                  /inspection/annotated/compressed ──► Foxglove / RViz / rosbridge
```

**安装与接入的逐步操作在该仓库的 README 里** —— 其中的 *Integrating with Odin-Nav-Stack2* 章节说明了话题契约、标注图发布，以及 ROS 1 / Jetson 上的坑。不要在这里重复那份配置。

导航侧的前置条件：你的驱动必须真的在推图 —— 驱动配置里 `sendrgb: 1` 与 `sendrgbundistort: 1`，并用 `rostopic hz /odin1/image/undistorted` 确认有数据之后，再接感知。
