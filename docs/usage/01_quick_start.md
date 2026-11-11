# 01 · Quick Start for Existing Robots

**English** | [中文](01_quick_start.zh-CN.md)

Start the navigation stack on a robot that is already supported: **Unitree Go2** (ROS1), **ASTRALL SW01** (ROS2 or ROS1), and **Dobot Lumo** (ROS1).

To run this on your own robot, see [03 · Integrate Your Own Robot](03_integrate_your_robot.md). To change speed, footprint or tracking behavior, see [02 · Parameter Tuning](02_parameter_tuning.md).

## 0. Prerequisites

- The workspace is built for the runtime you use (see the [README](../../README.md#installation-and-build)).
- The Odin1 / Odin1Lite sensor is connected and the udev rule is installed.
- A **grid map** (`NAME.yaml` + `NAME.pgm`) and, for relocalized navigation, the matching **Odin relocalization map** (`NAME.bin`) are available. Maps are selected with `--map=NAME`.

| Robot | Runtime | Grid map directory | Relocalization map directory | Profile |
|---|---|---|---|---|
| Unitree Go2 | ROS1 | `ros_ws/src/map_planner/maps/` | `customer_file/` | `robots/go2/` |
| ASTRALL SW01 | ROS2 | `ros2_ws/src/map_planner/maps/` | `customer_file/` | `robots/sw01/` |
| Dobot Lumo | ROS1 | `ros_ws/src/map_planner/maps/` | `customer_file/` | `robots/dobot_lumo/` |

To run without relocalization on SW01 ROS2 (or any robot), pass `--skip-reloc`.

## 1. Unitree Go2 (ROS1)

Go2 uses the ROS1 stack (`ons2_bringup/launch/go2.launch`) and the chassis bridge `unitree_control`, which requires **unitree_sdk2** to be installed.

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_go2.sh --map=NAME
```

- Readiness gates are printed in the main terminal; long-running processes run in the `ons2_go2` tmux session (`roscore / odin / go2 / mission / waypoint / rosbridge / foxglove / health / control`).
- Attach with `tmux attach -t ons2_go2`.
- Robot-specific geometry, global planner parameters, and NeuPAN configs are loaded from `robots/go2/robot.yaml` and `robots/go2/neupan/`.

Stop the stack:

```bash
bash scripts/ons2_go2_stop.sh
```

## 2. ASTRALL SW01 (ROS2 Jazzy)

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_sw01.sh --map=NAME
```

- Default navigation mode: `replan_periodic` (override with `--navigation-mode=escape|replan_periodic|neupan_only`).
- Readiness gates are printed in the main terminal; long-running nodes run in the `ons2_sw01` tmux session (`odin / map_server / sw01 / mission / atomic / waypoint / rosbridge / foxglove / health / control`).
- Attach with `tmux attach -t ons2_sw01`.
- `ROS_DOMAIN_ID=98` is set by the script. Set it yourself in any terminal you open by hand.
- Robot-specific geometry, global planner parameters, bridge topic, and NeuPAN configs are loaded from `robots/sw01/robot.yaml` and `robots/sw01/neupan/`.

Stop:

```bash
bash scripts/ons2_sw01_stop.sh
```

Mapless navigation (no prior map, e.g. at a new site before any relocalization map exists):

```bash
bash scripts/ons2_sw01_mapless.sh
```

- Odin runs in SLAM mode (`custom_map_mode: 1`). The navigation map is the blank `empty` map (100 m × 100 m, centred on the start pose), and a static identity `map → odom` TF replaces relocalization, so Gate 2 is skipped.
- Global paths on a blank map are straight lines, and obstacles not yet scanned are not on the global path. Send goals in short steps and keep the remote control ready to take over.
- `--map` is not accepted in this mode. `--navigation-mode`, `--periodic-replan-sec`, `--neupan-config` and `--mock-odin` work as in `ons2_sw01.sh`.
- Goals are given in the `map` frame, which equals `odom` with its origin at the start pose. Stop with `bash scripts/ons2_sw01_stop.sh`.

Useful options across `ons2_go2.sh`, `ons2_sw01.sh`, and `ons2_dobot_lumo.sh`:

| Option | Effect |
|---|---|
| `--map=NAME` | Switch navigation `.yaml/.pgm` and relocalization `customer_file/NAME.bin` together (default `map_opt`) |
| `--navigation-mode=MODE` | `replan_periodic` (default), `escape`, or `neupan_only` |
| `--periodic-replan-sec=SEC` | Periodic global replanning interval in seconds (default `2.0`) |
| `--neupan-config=PATH` | Override `robots/<robot>/neupan/config.yaml` |
| `--skip-reloc` | Skip the relocalization gate (use only for bench tests) |
| `--mock-odin` | Replace the Odin driver with the mock node for hardware-free stack verification |

## 3. Dobot Lumo (ROS1)

Dobot Lumo uses the ROS1 stack (`ons2_bringup/launch/dobot.launch`) with the Odin1Lite Ethernet driver and the Python JSON-RPC velocity bridge `dobot_vel_bridge.py`.

```bash
cd ~/Odin-Nav-Stack2
bash scripts/ons2_dobot_lumo.sh --map=NAME
```

- Readiness gates (roscore, O1L device discovery, `load_map`, `odom -> map` TF, and navigation input gate) are printed in the main terminal; long-running nodes run in the `ons2_dobot_lumo` tmux session (`roscore / odin / load_map / dobot / neupan / mission / waypoint / rosbridge / foxglove / health / control`).
- Attach with `tmux attach -t ons2_dobot_lumo`.
- **Safety default:** `dobot.launch` defaults to `dry_run=true` and `allow_walk_switch=false` (read-only mode; no `SetVel` commands are sent to the robot). Keep these defaults until supervised hardware testing and Dobot geometry calibration are complete.

Stop:

```bash
bash scripts/ons2_dobot_lumo_stop.sh
```

## 4. Send Missions & Interactive Control Console

Once the stack is running and `/waypoint_manager/status` reports `IDLE`, you can dispatch missions via the interactive terminal console (`test_nav_control_rospy.py` / `test_nav_control_ros2.py`) or CLI commands.

### ROS1

#### Option A: Interactive Terminal Console (`ros_ws/scripts/test_nav_control_rospy.py`)

When launched via `ons2_go2.sh` or `ons2_dobot_lumo.sh`, the interactive console is already running in the `control` tmux window:

```bash
tmux attach -t ons2_go2 \; select-window -t control          # Go2
tmux attach -t ons2_dobot_lumo \; select-window -t control   # Dobot Lumo
```

Or open it in any standalone terminal (omit `--allow-motion` for read-only monitoring):

```bash
source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash
/usr/bin/python3 ros_ws/scripts/test_nav_control_rospy.py --allow-motion
```

#### Option B: Command-Line Helper (`scripts/tools/mission_cli.sh`)

```bash
bash scripts/tools/mission_cli.sh status
bash scripts/tools/mission_cli.sh start 2.0 0.0 0.0 3.0 point_A    # x y yaw dwell_sec name (one waypoint per call)
bash scripts/tools/mission_cli.sh pause | resume | cancel | return_home | reset
```

Commands that move the robot (`pause`, `resume`, `cancel`, `return_home`, `reset`) require `GO2_MOTION_CONFIRM=I_UNDERSTAND_GO2_MAY_MOVE`.

### ROS2

#### Option A: Interactive Terminal Console (`ros2_ws/scripts/test_nav_control_ros2.py`)

Attach to the `control` window started by `ons2_sw01.sh`, or launch it in a standalone terminal (read-only unless `--allow-motion` is passed):

```bash
# Attach to the running tmux control window:
tmux attach -t ons2_sw01 \; select-window -t control

# Or launch standalone:
export ROS_DOMAIN_ID=98
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 ros2_ws/scripts/test_nav_control_ros2.py --allow-motion
```

#### Option B: Direct ROS 2 Service Calls

```bash
ros2 service call /waypoint_manager/start_mission nav_interfaces/srv/StartMission \
  "{waypoints: [{x: 2.0, y: 0.0, yaw: 0.0, dwell_sec: 3.0, name: 'A'},
                {x: 5.0, y: 1.2, yaw: 1.57, dwell_sec: 0.0, name: 'B'}],
    loop: false, temporary: false}"

ros2 service call /waypoint_manager/pause std_srvs/srv/Trigger "{}"
ros2 topic echo /waypoint_manager/status --once
```

## 5. Web Control Frontend

Beyond the terminal console, ONS2 exposes the robot over a WebSocket bridge, so you can
drive and watch it from a browser.

### 5.1 Open-source frontend (Foxglove) — included

The launcher already starts `foxglove_bridge`; nothing extra to install.

1. Open [Foxglove Studio](https://foxglove.dev/) in a browser (or the desktop app).
2. Connect to **Foxglove WebSocket** → `ws://<robot-ip>:8765`
   (the bridge listens on `0.0.0.0`, so both the wired and WiFi addresses work).
3. Add panels: **3D** (point cloud + TF), **Image** (`/odin1/image/undistorted`),
   **Plot**, and **Raw Message** for `/waypoint_manager/status`.
4. Publish a goal from Foxglove: set the *2D Pose* publish topic to `/waypoint_pick`
   (the dedicated capture channel) — **not** `/move_base_simple/goal`, or the robot starts
   moving on every drag.

### 5.2 3DGS navigation console — optional, contact us

For production inspection we provide a **separate, actively maintained control terminal**
with a 3D Gaussian Splatting scene view: much richer visualisation, mission editing, and
fleet-ready dashboards. **It supports both ROS 1 and ROS 2**, so it also covers the platforms
where the open-source monitor above is not available yet (Go2, Dobot Lumo). It is not part of
this open-source release — **please contact us to obtain it**.

> 📹 **Video demos of both frontends** are on the
> [ONS2 project page](https://manifoldtechltd.github.io/Odin-Nav-Stack2-Webpage/), in the
> **"Navigation in 3DGS Reconstructed Scenes"** section.
>
> If you are evaluating which frontend to start with: begin with Foxglove (free, zero setup),
> and move to the 3DGS console when you need production-grade mission management.

## 6. Check Before Trusting the Robot

1. **TF**: `map → odom → odin1_base_link → base_link` must be connected (see the FAQ in the [README](../../README.md#faq)).
2. **Sign convention**: `+linear.x` forward, `+angular.z` counter-clockwise. Call the `move` and `turn` services of `atomic_move` (0.5 m forward, 90° left) and check the direction.
3. **Short mission first**: one waypoint 2–3 m ahead in open space, then the narrow passages.
