# 在 Isaac Sim 中运行 ONS2（MT-Real2Sim）

本文是"仿真器运行 ONS2"的唯一使用文档来源。仿真器仓库为
[MT-Real2Sim-Tutorial](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial)，集成资产位于
其 `integrations/ons2/` 目录。

> 状态：ROS 2（Jazzy）已端到端验证（3 航点、5 航点任务均 `COMPLETED`）。
> 仿真器的 ROS 1（Noetic）桥已支持 odin 帧布局；Noetic 侧启动脚本与 NeuPAN Python 3.10
> sidecar 尚未验证。

## 1. 真机与仿真的关系

仿真支持是增量、默认关闭的，不会改变真机上的运行方式。

| | 真机 | 仿真（Isaac Sim） |
|---|---|---|
| 入口 | `scripts/ons2_sw01.sh`（ROS 2）、`scripts/ons2_go2.sh` / `scripts/ons2_dobot_lumo.sh`（ROS 1） | 仿真 ROS 2 容器内的 `scripts/ons2_sim_ros2.sh` |
| 平台档案 | `robots/<robot>/robot.yaml` | `robots/go2_sim/robot.yaml`，复用 `robots/go2/neupan/` |
| 时钟 | 系统时钟，`use_sim_time=false` | 所有节点 `use_sim_time:=true`，`/clock` 由桥发布 |
| 传感器 / TF | Odin1 驱动发布 `/odin1/*` 与 TF | `sim_bridge_ros2` 发布 `/odin1/*` + `/clock`、订阅 `/cmd_vel`；TF 链同为 `map→odom→imu→odin1_base_link→base_link` |
| 地图 | 现场地图三件套，`--map=NAME` | 仿真器 `integrations/ons2/maps/` 下的 `office_0923_sim.pgm` / `.yaml`，经 `--map` 传入 |
| 底盘 | SW01 `[c]` 使能闸门 | 不使用；`/cmd_vel` 直接驱动仿真机器人 |

共享的 ROS 2 launch 只新增了默认 `false` 的 `use_sim_time` 参数，控制台脚本只新增了可选的
`--use-sim-time` 开关；仿真档案与 `scripts/ons2_sim_ros2.sh` 都是独立文件。规划器、控制器与
导航调参代码零改动，只要不使用仿真开关，真机行为与之前完全一致。

### 1.1 仿真要改什么 / 真机要改什么

| 对象 | 要改的内容 |
|---|---|
| 仿真 | 用 `scripts/ons2_sim_ros2.sh` 并指定仿真地图（`--map`），保持所有节点 `use_sim_time:=true` 且 `/clock` 正常；航点用仿真器的 `send_waypoints.sh` 或 Foxglove `/waypoint_pick` 控制台下发（§6、§6.1）。 |
| 真机 | 什么都不用改。继续用 `scripts/ons2_*.sh` 与 `robots/<robot>/...`，不要传 `use_sim_time:=true`（默认为 `false`）。 |

启动见 §5，排障对照表见 §8。

## 2. 运行拓扑

```
Isaac Sim (TCP:9090, msgpack)
├─ ROS 2 Jazzy 容器：sim_bridge_ros2 → /odin1/* + /clock，订阅 /cmd_vel
│   ONS2 ROS 2：go2_sim profile、map_server、sliding_map、map_planner、NeuPAN、mission_manager
└─ ROS 1 Noetic 容器：sim_bridge（frame_layout=odin），ONS2 ROS 1（未验证）
```

TF 链与真机 Go2 一致：`map→odom→imu→odin1_base_link→base_link`。话题/TF 契约见仿真器
仓库 `docs/simulation-interface.md`。

## 3. 前置条件

* 仿真器 `v2` 分支（或集成分支）与 Isaac 镜像 `r2s2r-isaac:5.1.0`。
* 场景目录（包含 `office_0923.usdz`，不随仓库分发，通过 `DATASET_DIR` 指定）。
* Docker + NVIDIA container toolkit。
* ONS2 仓库与本仓库同级（默认 `../Odin-Nav-Stack2`，可用 `--ons2-root` 覆盖）。

## 4. 一次性构建（ROS 2）

```bash
cd MT-Real2Sim-Tutorial/docker
docker compose build ros2

ONS2_ROOT=/abs/path/to/Odin-Nav-Stack2 docker compose up -d ros2
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ons2/ros2_ws &&
  colcon build --symlink-install --packages-select \
    nav_interfaces atomic_move map_planner sliding_map neupan_ros mission_manager \
    ons2_bringup sim_bridge_ros2'
```

NeuPAN 以源码方式运行（`PYTHONPATH=/ons2/NeuPAN`）；镜像内已装好其依赖（torch CPU、
cvxpylayers、ecos、gctl、matplotlib、numpy 1.26、scipy 1.12）。

**NeuPAN 配置布局。** 每个机器人目录 `robots/<robot>/neupan/` 下各有 `config.yaml` +
`planner.yaml`。官方三套：`robots/go2`（Unitree Go2）、`robots/sw01`（璇玑 SW01，解析式
DUNE）、`robots/_template`（用户自定义副本）。仿真 profile `robots/go2_sim` **复用**
Go2 的同一份文件（`local_planner.backends.neupan.config: ../go2/neupan/config.yaml`），
不要再在 `go2_sim` 下复制第四套。

## 5. 启动

```bash
cd MT-Real2Sim-Tutorial
DATASET_DIR=/abs/path/to/dataset \
  ./start.sh --robot go2 --runtime ros2 --stack ons2 --map office_0923_sim
```

日志：

* 桥：`docker exec isaac_nav_ros2 tail -f /tmp/bridge.log`
* 导航：`docker exec isaac_nav_ros2 tail -f /tmp/ons2_sim.log`

手动方式（Isaac 已在运行时）：

```bash
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash &&
  ros2 launch sim_bridge_ros2 bridge.launch.py \
    bridge_config:=/ons2/ros2_ws/src/sim_bridge_ros2/config/go2_sim.yaml'
docker exec isaac_nav_ros2 bash -lc \
  'bash /ons2/scripts/ons2_sim_ros2.sh --map /integrations/ons2/maps/office_0923_sim.yaml'
```

`scripts/ons2_sim_ros2.sh` 是唯一传 `use_sim_time:=true` 的入口：它依次拉起
`map_server`、`go2_sim` bringup、`atomic_move`、`mission_manager`、waypoint 桥，以及
（装有 tmux 时）控制台。若节点无输出或机器人不动，先查 §8，不要先改参数。

## 6. 下发航点

接口与真机完全一致：`waypoint_goal_bridge` → `/waypoint_manager/start_mission`。

```bash
cd MT-Real2Sim-Tutorial
bash integrations/ons2/waypoints/send_waypoints.sh --runtime ros2 --scene office_0923_sim
```

监控：

```bash
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash &&
  ros2 topic echo /waypoint_manager/status'
```

已验证预设：从出生区出发的 3 航点任务（`office_0923_sim.yaml`）与 5 航点返程
（`office_0923_return5.yaml`）。

## 6.1 交互控制台（与真机同一脚本）

仿真运行官方 ROS 2 控制台 `ros2_ws/scripts/test_nav_control_ros2.py`（真机
`scripts/ons2_sw01.sh` 启动的同一脚本），放在 tmux 会话里：

```bash
docker exec -it isaac_nav_ros2 tmux attach -t ons2_sim
```

* 在 Foxglove 用 **Pose** 工具发到 `/waypoint_pick`，frame 选 `map`；单点/多点采点、
  暂停/恢复/取消/返航等状态转移都在控制台里完成。
* 控制台以 `--allow-motion` 启动；仿真额外传 `--use-sim-time`，让 TF 查询使用
  `/clock`（除该参数外与 public main 脚本一致）。
* `[c]` 底盘使能是 SW01 专用，仿真不需要；运动直接经 `/cmd_vel` 下发。

public `main` 官方启动脚本：ROS1 Go2 `scripts/ons2_go2.sh` →
`ros_ws/scripts/test_nav_control_rospy.py`；ROS2 `scripts/ons2_sw01.sh` →
`ros2_ws/scripts/test_nav_control_ros2.py`。旧 `test_nav_control.py` 已被这两个
运行时专用脚本取代。

## 7. 地图说明

* `office_0923_sim.pgm` 由内部 v3 部分地图派生（原件保留为 `office_0923_v3partial.pgm`）；
  unknown 填为 free，墙体（0）与门（128）仍视为障碍。
* 规划器把所有非 free 格视为不可通行，且会**重定位**余量不足（约 <0.55 m）的目标
  （`goal_relocate_max_radius_m=3.5 m`）。选航点时保证余量 ≥1.1 m，并用
  `/map_planner/plan` 预验证。
* 2D 地图必须与 USDZ 场景同一坐标系，不能拿真机地图直接套用仿真场景。

## 8. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` 下 source ROS；脚本已在 source 期间关闭 `set -u`。 |
| NeuPAN 报 `planner.yaml not found` | 每个机器人的 `neupan/` 需同时有 `config.yaml` + `planner.yaml`；`go2_sim` 通过 profile 的 `config:` 复用 `go2` 的同一份。 |
| `msgpack` / `numpy._core` 报错 | 镜像需包含 `msgpack`、numpy 1.26.4、scipy 1.12.0（见 Dockerfile.ros2）。 |
| 目标被重定位 | 提高航点余量（≥1.1 m）或调整 `goal_relocate_*` 参数。 |
| 导航不前进 | 所有节点需 `use_sim_time:=true`，且桥必须发布 `/clock`。 |
| GUI 模式不动 | 点击 Isaac 的 **Play**；office 场景配置默认 headless。 |
| 没有 `/odin1/*` | TCP 只允许一个客户端；保持单个桥连接，Isaac 重启后重启桥。 |

## 9. ROS 1 运行时

仿真器 ROS 1 桥已支持 `frame_layout: odin` 与相同话题。运行 Noetic 版 ONS2 还需：
NeuPAN Python 3.10 sidecar 容器（Noetic 为 Python 3.8）、Noetic 地图路径参数与
`ons2_go2_sim.sh` 启动脚本。该路径列为后续工作，尚未验证。
