# ros2_ws — ROS 2 Jazzy Workspace

This workspace contains the **ROS 2 Jazzy (Ubuntu 24.04)** navigation, perception, and chassis bridge packages for ONS2.

- Topic, service, and action names match the ROS 1 (`ros_ws/`) stack 1:1.
- Robot-specific geometry, sensor mount TF, footprint, global-planner parameters, and NeuPAN config paths are loaded from [`../robots/<robot>/robot.yaml`](../robots/README.md) via `ons2_bringup`.
- Upstream NeuPAN (`../NeuPAN/`) runs directly under Python 3.12 via `neupan_ros/ros2_shim.py`.

---

## Package Overview

| Package | Description |
|---|---|
| `nav_interfaces` | Custom ROS 2 messages (`Waypoint`, `MissionStatus`, `MissionEvent`, `OperatorCommand`), services (`StartMission`, `PlanPath`, `PlanSequence`, `AtomicMove`, `AtomicTurn`, `AtomicAlign`), and actions (`nav`, `navPath`). |
| `ons2_bringup` | Robot-profile-driven launch orchestration (`bringup.launch.py`) reading `robots/<robot>/robot.yaml`. |
| `sliding_map` | Ego-centric 360-degree virtual laser scan accumulator (`sliding_map_node`) with polygon/radial robot footprint masking and temporal cell decay. |
| `map_planner` | 2D occupancy-grid A* global planner (`map_planner_node`) and optional temporal scan filter (`scan_temporal_filter.py`). |
| `neupan_ros` | ROS 2 action server (`neupan_action_node.py`) and `ros2_shim.py` wrapping the NeuPAN MPC + DUNE local planner. |
| `mission_manager` | Multi-waypoint mission state machine (`mission_manager_node`) supporting periodic replanning, stuck recovery, route segmentation, and heading alignment. |
| `atomic_move` | Closed-loop short-range translation, in-place rotation, and goal yaw alignment services (`atomic_move_node`). |
| `sw01_ros_control` | ASTRALL SW01 chassis velocity & SDK lifecycle bridge (`sw01_cmd_vel_bridge`). |
| `pointcloud_saver` | Online voxel/statistical filtering and PCD map exporter (`pointcloud_saver_node`). |
| `odin_vlm_terminal` | Visual-language query node subscribing to Odin camera streams. |

---

## Building on Ubuntu 24.04 (ROS 2 Jazzy)

```bash
# 1. Install ROS 2 Jazzy system dependencies
sudo apt install ros-jazzy-ros-base ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-rosbridge-server ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-std-srvs ros-jazzy-message-filters \
  ros-jazzy-nav2-map-server ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime

# 2. Install NeuPAN Python dependencies
python3 -m pip install numpy scipy cvxpy cvxpylayers ecos matplotlib rich pyyaml scikit-learn loguru "gctl==1.2" torch \
  --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple

# 3. Build the workspace
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> **Odin Driver Submodule**: Initialize the sensor driver submodule from the repository root before building if needed:
> `git submodule update --init ros_ws/src/odin_ros_driver`

---

## Running the Stack

Use the unified entry script from the repository root:

```bash
# Start full SW01 ROS 2 stack in tmux (default ROS_DOMAIN_ID=98)
bash scripts/ons2_sw01.sh --map=map_opt

# Stop stack
bash scripts/ons2_sw01_stop.sh
```

See [`../docs/usage/01_quick_start.md`](../docs/usage/01_quick_start.md) and [`../docs/usage/02_parameter_tuning.md`](../docs/usage/02_parameter_tuning.md) for full usage and parameter tuning instructions.
