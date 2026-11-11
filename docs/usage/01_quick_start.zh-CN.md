# 01 · 现有机器人快速开始

[English](01_quick_start.md) | **中文**

在已支持的机器人上启动导航栈：**Unitree Go2**（ROS1）、**ASTRALL SW01**（ROS2 或 ROS1）、**Dobot Lumo**（ROS1）。

换成自己的机器人看 [03 · 接入你自己的机器人](03_integrate_your_robot.md)；改速度、外形或跟踪行为看 [02 · 参数调优](02_parameter_tuning.md)。

## 0. 前置条件

- 工作区已按你使用的运行时编译完成（见 [README](../../README.zh-CN.md#安装与编译)）。
- Odin1 / Odin1Lite 传感器已连接，且 udev 规则已安装。
- 准备好**栅格地图**（`NAME.yaml` + `NAME.pgm`），走重定位导航的话还要配套的 **Odin 重定位地图**（`NAME.bin`）。地图用 `--map=NAME` 选择。

| 机器人 | 运行时 | 栅格地图目录 | 重定位地图目录 | 平台档案 |
|---|---|---|---|---|
| Unitree Go2 | ROS1 | `ros_ws/src/map_planner/maps/` | `customer_file/` | `robots/go2/` |
| ASTRALL SW01 | ROS2 | `ros2_ws/src/map_planner/maps/` | `customer_file/` | `robots/sw01/` |
| Dobot Lumo | ROS1 | `ros_ws/src/map_planner/maps/` | `customer_file/` | `robots/dobot_lumo/` |

如果不想走重定位（SW01 ROS2 或任意机器人），加 `--skip-reloc`。

## 1. Unitree Go2（ROS1）

Go2 用 ROS1 栈（`ons2_bringup/launch/go2.launch`）和底盘桥 `unitree_control`，后者需要先装 **unitree_sdk2**。

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_go2.sh --map=NAME
```

- 就绪闸门在主终端打印；长驻进程跑在 `ons2_go2` 这个 tmux 会话里（`roscore / odin / go2 / mission / waypoint / rosbridge / foxglove / health / control`）。
- 进入会话：`tmux attach -t ons2_go2`。
- 机器人的几何参数、全局规划参数与 NeuPAN 配置从 `robots/go2/robot.yaml` 和 `robots/go2/neupan/` 加载。

停止：

```bash
bash scripts/ons2_go2_stop.sh
```

## 2. ASTRALL SW01（ROS2 Jazzy）

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_sw01.sh --map=NAME
```

- 默认导航模式 `replan_periodic`（可用 `--navigation-mode=escape|replan_periodic|neupan_only` 覆盖）。
- 就绪闸门在主终端打印；长驻节点跑在 `ons2_sw01` 这个 tmux 会话里（`odin / map_server / sw01 / mission / atomic / waypoint / rosbridge / foxglove / health / control`）。
- 进入会话：`tmux attach -t ons2_sw01`。
- 脚本里设了 `ROS_DOMAIN_ID=98`。你手动开的终端要自己 export 同样的值。
- 机器人的几何参数、全局规划参数、桥接话题与 NeuPAN 配置从 `robots/sw01/robot.yaml` 和 `robots/sw01/neupan/` 加载。

停止：

```bash
bash scripts/ons2_sw01_stop.sh
```

无图导航（没有先验地图，例如新场地在建立重定位地图之前）：

```bash
bash scripts/ons2_sw01_mapless.sh
```

- Odin 以 SLAM 模式运行（`custom_map_mode: 1`）。导航地图为空白的 `empty`（100 m × 100 m，以启动位姿为中心），用静态恒等变换 `map → odom` 代替重定位，因此跳过闸门 2。
- 空白图上的全局路径只是直线，尚未扫描到的障碍不在全局路径上。请分小步下发目标，并随时准备用遥控器接管。
- 该模式不接受 `--map`。`--navigation-mode`、`--periodic-replan-sec`、`--neupan-config`、`--mock-odin` 的用法与 `ons2_sw01.sh` 相同。
- 目标在 `map` 坐标系下下发，`map` 与 `odom` 重合，原点为启动位姿。停止：`bash scripts/ons2_sw01_stop.sh`。

`ons2_go2.sh`、`ons2_sw01.sh`、`ons2_dobot_lumo.sh` 共通的常用参数：

| 选项 | 作用 |
|---|---|
| `--map=NAME` | 同时切换导航用 `.yaml/.pgm` 与重定位用 `customer_file/NAME.bin`（默认 `map_opt`） |
| `--navigation-mode=MODE` | `replan_periodic`（默认）、`escape` 或 `neupan_only` |
| `--periodic-replan-sec=SEC` | 周期性全局重规划间隔（秒，默认 `2.0`） |
| `--neupan-config=PATH` | 覆盖 `robots/<robot>/neupan/config.yaml` |
| `--skip-reloc` | 跳过重定位闸门（**仅限台架测试**） |
| `--mock-odin` | 用 mock 节点替代 Odin 驱动，便于无硬件验证整栈 |

## 3. Dobot Lumo（ROS1）

Dobot Lumo 用 ROS1 栈（`ons2_bringup/launch/dobot.launch`），配 Odin1Lite 以太网驱动和 Python JSON-RPC 速度桥 `dobot_vel_bridge.py`。

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_dobot_lumo.sh --map=NAME
```

- 就绪闸门（roscore、O1L 设备发现、`load_map`、`odom -> map` TF、导航输入门）在主终端打印；长驻节点跑在 `ons2_dobot_lumo` 会话里（`roscore / odin / load_map / dobot / neupan / mission / waypoint / rosbridge / foxglove / health / control`）。
- 进入会话：`tmux attach -t ons2_dobot_lumo`。
- **安全默认值：** `dobot.launch` 默认 `dry_run=true` 和 `allow_walk_switch=false`（只读模式，**不会**向机器人下发 `SetVel`）。在有监督的硬件测试和 Dobot 几何标定完成之前，请保持这两个默认值。

停止：

```bash
bash scripts/ons2_dobot_lumo_stop.sh
```

## 4. 发送任务与交互控制台

栈跑起来、且 `/waypoint_manager/status` 报告 `IDLE` 之后，就可以通过交互终端（`test_nav_control_rospy.py` / `test_nav_control_ros2.py`）或命令行下发任务。

### ROS1

#### 方式 A：交互终端（`ros_ws/scripts/test_nav_control_rospy.py`）

用 `ons2_go2.sh` 或 `ons2_dobot_lumo.sh` 启动时，交互控制台已经跑在 `control` 窗口里：

```bash
tmux attach -t ons2_go2 \; select-window -t control          # Go2
tmux attach -t ons2_dobot_lumo \; select-window -t control   # Dobot Lumo
```

也可以在任何独立终端里打开（去掉 `--allow-motion` 就是只读监控）：

```bash
source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash
/usr/bin/python3 ros_ws/scripts/test_nav_control_rospy.py --allow-motion
```

#### 方式 B：命令行助手（`scripts/tools/mission_cli.sh`）

```bash
bash scripts/tools/mission_cli.sh status
bash scripts/tools/mission_cli.sh start 2.0 0.0 0.0 3.0 point_A    # x y yaw dwell_sec name（每次一个航点）
bash scripts/tools/mission_cli.sh pause | resume | cancel | return_home | reset
```

会让机器人动的命令（`pause`、`resume`、`cancel`、`return_home`、`reset`）需要显式设置 `GO2_MOTION_CONFIRM=I_UNDERSTAND_GO2_MAY_MOVE`。

### ROS2

#### 方式 A：交互终端（`ros2_ws/scripts/test_nav_control_ros2.py`）

进 `ons2_sw01.sh` 起的 `control` 窗口，或在独立终端里启动（不加 `--allow-motion` 为只读）：

```bash
# 进入正在跑的 tmux 控制窗口：
tmux attach -t ons2_sw01 \; select-window -t control

# 或者独立启动：
export ROS_DOMAIN_ID=98
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 ros2_ws/scripts/test_nav_control_ros2.py --allow-motion
```

#### 方式 B：直接调 ROS 2 服务

```bash
ros2 service call /waypoint_manager/start_mission nav_interfaces/srv/StartMission \
  "{waypoints: [{x: 2.0, y: 0.0, yaw: 0.0, dwell_sec: 3.0, name: 'A'},
                {x: 5.0, y: 1.2, yaw: 1.57, dwell_sec: 0.0, name: 'B'}],
    loop: false, temporary: false}"

ros2 service call /waypoint_manager/pause std_srvs/srv/Trigger "{}"
ros2 topic echo /waypoint_manager/status --once
```

## 5. Web 控制前端

除了终端控制台，ONS2 还通过 WebSocket 桥把机器人暴露出来，你可以用浏览器驱动与观察它。

### 5.1 开源前端（Foxglove）—— 已随启动脚本拉起

启动编排已经起了 `foxglove_bridge`，不需要额外安装。

1. 浏览器或桌面版打开 [Foxglove Studio](https://foxglove.dev/)。
2. 连接 **Foxglove WebSocket** → `ws://<机器人IP>:8765`（桥监听 `0.0.0.0`，有线与 WiFi 地址都能连）。
3. 添加面板：**3D**（点云 + TF）、**Image**（`/odin1/image/undistorted`）、**Plot**，以及 `/waypoint_manager/status` 的 **Raw Message**。
4. 从 Foxglove 发点：把 *2D Pose* 的发布话题改成 `/waypoint_pick`（专用采点通道）——**不要**用 `/move_base_simple/goal`，否则每拖一下机器人都会立刻动。

### 5.2 3DGS 导航控制台 —— 可选，联系获取

面向生产巡检，我们另外提供一套**独立维护的控制终端**，带 3D Gaussian Splatting 场景视图：可视化更丰富、任务编辑、可直接做车队看板。**它同时支持 ROS 1 与 ROS 2**，所以上面的开源监控还覆盖不到的平台（Go2、Dobot Lumo）也能用它。它不在本次开源范围内 —— **请联系我们获取**。

> 📹 **两个前端的演示视频**都在
> [ONS2 项目主页](https://manifoldtechltd.github.io/Odin-Nav-Stack2-Webpage/)的
> **"Navigation in 3DGS Reconstructed Scenes"** 章节里。
>
> 如果你在纠结先上哪个前端：从 Foxglove 开始（免费、零配置），需要生产级任务管理时再上 3DGS 控制台。

## 6. 信任机器人之前的检查

1. **TF**：`map → odom → odin1_base_link → base_link` 必须连上（见 [README 的 FAQ](../../README.zh-CN.md#faq)）。
2. **符号约定**：`+linear.x` 向前、`+angular.z` 逆时针。调 `atomic_move` 的 `move` 与 `turn` 服务（前进 0.5 m、左转 90°）验证方向。
3. **先跑短任务**：开阔地发一个前方 2–3 m 的航点，再试窄通道。
