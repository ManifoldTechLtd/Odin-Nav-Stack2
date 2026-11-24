# 在 Isaac Sim 中运行 ONS2（MT-Real2Sim）

本文是"仿真器运行 ONS2"的唯一使用文档来源。仿真器仓库为
[MT-Real2Sim-Tutorial](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial)，集成资产位于
其 `integrations/ons2/` 目录。

> 状态：ROS 2（Jazzy）已端到端验证（3 航点、5 航点任务均 `COMPLETED`）。
> 仿真器的 ROS 1（Noetic）桥已支持 odin 帧布局；Noetic 侧启动脚本与 NeuPAN Python 3.10
> sidecar 尚未验证。

## 1. 运行拓扑

```
Isaac Sim (TCP:9090, msgpack)
├─ ROS 2 Jazzy 容器：sim_bridge_ros2 → /odin1/* + /clock，订阅 /cmd_vel
│   ONS2 ROS 2：go2_sim profile、map_server、sliding_map、map_planner、NeuPAN、mission_manager
└─ ROS 1 Noetic 容器：sim_bridge（frame_layout=odin），ONS2 ROS 1（未验证）
```

TF 链与真机 Go2 一致：`map→odom→imu→odin1_base_link→base_link`。话题/TF 契约见仿真器
仓库 `docs/simulation-interface.md`。

## 2. 前置条件

* 仿真器 `v2` 分支（或集成分支）与 Isaac 镜像 `r2s2r-isaac:5.1.0`。
* 场景目录（包含 `office_0923.usdz`，不随仓库分发，通过 `DATASET_DIR` 指定）。
* Docker + NVIDIA container toolkit。
* ONS2 仓库与本仓库同级（默认 `../Odin-Nav-Stack2`，可用 `--ons2-root` 覆盖）。

## 3. 一次性构建（ROS 2）

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

## 4. 启动

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

## 5. 下发航点

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

## 6. 地图说明

* `office_0923_sim.pgm` 由内部 v3 部分地图派生（原件保留为 `office_0923_v3partial.pgm`）；
  unknown 填为 free，墙体（0）与门（128）仍视为障碍。
* 规划器把所有非 free 格视为不可通行，且会**重定位**余量不足（约 <0.55 m）的目标
  （`goal_relocate_max_radius_m=3.5 m`）。选航点时保证余量 ≥1.1 m，并用
  `/map_planner/plan` 预验证。
* 2D 地图必须与 USDZ 场景同一坐标系，不能拿真机地图直接套用仿真场景。

## 7. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` 下 source ROS；脚本已在 source 期间关闭 `set -u`。 |
| NeuPAN 报 `planner.yaml not found` | 每个机器人的 `neupan/` 需同时有 `config.yaml` + `planner.yaml`；`go2_sim` 通过 profile 的 `config:` 复用 `go2` 的同一份。 |
| `msgpack` / `numpy._core` 报错 | 镜像需包含 `msgpack`、numpy 1.26.4、scipy 1.12.0（见 Dockerfile.ros2）。 |
| 目标被重定位 | 提高航点余量（≥1.1 m）或调整 `goal_relocate_*` 参数。 |
| 导航不前进 | 所有节点需 `use_sim_time:=true`，且桥必须发布 `/clock`。 |
| GUI 模式不动 | 点击 Isaac 的 **Play**；office 场景配置默认 headless。 |
| 没有 `/odin1/*` | TCP 只允许一个客户端；保持单个桥连接，Isaac 重启后重启桥。 |

## 8. ROS 1 运行时

仿真器 ROS 1 桥已支持 `frame_layout: odin` 与相同话题。运行 Noetic 版 ONS2 还需：
NeuPAN Python 3.10 sidecar 容器（Noetic 为 Python 3.8）、Noetic 地图路径参数与
`ons2_go2_sim.sh` 启动脚本。该路径列为后续工作，尚未验证。
