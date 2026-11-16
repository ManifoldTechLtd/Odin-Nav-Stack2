# SRU — End-to-End Visual Navigation Policy (Inference)

SRU (Spatially-Enhanced Recurrent Memory) is a **mapless, global-planner-free**
end-to-end visual navigation policy: depth image + odometry + goal in,
body-frame velocity commands out (5 Hz by default). In this repository it is a
switchable `local_planner` backend alongside the default NeuPAN.

> Training code lives in [ManifoldTechLtd/SRU-Odin](https://github.com/ManifoldTechLtd/SRU-Odin),
> not here. This directory holds the **inference core only**; ONNX models are
> **not distributed with the repository** (see §2).

## 1. Layout

```
sru/
├── README.md / requirements.txt
└── src/                          # inference core (single source, import name: sru)
    ├── constants.py              #   thresholds (arrival 0.6 m, depth staleness)
    ├── model.py                  #   ONNX Runtime inference (VAE + policy, LSTM state)
    ├── navigation_policy_node.py #   control loop, goal management, zero-velocity guards
    ├── node_main.py              #   version-agnostic entry (params, model paths)
    ├── ros_shim.py               #   rospy <-> rclpy adaptation layer
    └── utils.py / visualization.py

ros2_ws/src/sru_nav/              # ROS2 ament package (thin wrapper)
ros_ws/src/sru_nav/               # ROS1 catkin package (thin wrapper)
```

The `sru` entry at each package root is a **relative symlink** to `sru/src`, so
both distributions run one copy of the code and cannot drift. **Use `git clone`**
— zip downloads drop symlinks. All ROS-version differences are absorbed by
`ros_shim.py` (via `ROS_VERSION`); the node source contains no raw ROS API calls.

## 2. Models (not in git)

Provide two ONNX files under `<repo>/sru/models/` (the start scripts look there
first), or point elsewhere with environment variables:

| File | Default name | Override |
|---|---|---|
| VAE encoder | `vae_encoder.onnx` | `SRU_VAE_MODEL` |
| Navigation policy | `nav_policy.onnx` | `SRU_POLICY_MODEL` (ROS2 also `SRU_MODELS_DIR`) |

- Policy I/O: `obs[·,2576] + h_in/c_in[1,·,512]` → `actions[·,3] + h_out/c_out[1,·,512]`
- VAE: `[·,1,40,64]` → `mu[·,64,5,8]`
- The policy **must be trained for your robot** (locomotion + camera extrinsics
  consistency); training and export are covered in the SRU-Odin repository.
- Always pass **explicit absolute model paths** — auto-discovery only matches
  default file names and may silently load the wrong policy. Both start scripts
  already handle this.

## 3. ROS2 (Jazzy)

### Build

```bash
cd <repo>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sru_nav --symlink-install
source install/setup.bash
```

### Runtime dependencies

```bash
# Block user-site: ~/.local's numpy 2.x conflicts with Jazzy's numpy 1.x ABI.
export PYTHONNOUSERSITE=1
# Inference deps in a dedicated venv, injected via PYTHONPATH (the start
# script does this automatically for <SRU_WS>/venv). See requirements.txt.
python3 -m venv --system-site-packages ~/sru_venv
~/sru_venv/bin/pip install -r <repo>/sru/requirements.txt
```
> Pass `SRU_VENV=~/sru_venv` to `start_sru_ros2.sh` (default: `<SRU_WS>/venv`).

### Run

```bash
# SRU node (Odin driver and chassis cmd_vel bridge must already be running)
bash ros2_ws/scripts/start_sru_ros2.sh

# Gated relay (separate terminal; the robot will not move without it)
python3 ros2_ws/scripts/sru_cmd_vel_relay.py
```

The platform orchestrator (`scripts/ons2_sw01.sh --local-planner=sru`) includes
the package launch directly via the sru backend in `robots/sw01/robot.yaml`;
the relay still needs to be started separately.

### Contract topics

| Direction | Topic | Notes |
|---|---|---|
| in | `/sru/goal_pose` (`PoseStamped`, **odom frame**) | from `mission_manager` (after map→odom transform) |
| out | `/sru/cmd_vel` (`Twist`, 5 Hz, body frame) | forwarded by the gated relay |
| gate | `/sru/cmd_vel_enable` (`Bool`, transient_local) | `true`: relay forwards `/sru/cmd_vel` → `/cmd_vel`; closed during arrival alignment so `atomic_move` owns `/cmd_vel` |

**Safety**: the node itself performs no joystick/deadman checks — permission to
move is entirely the gate chain's job (relay defaults closed, mission layer
opens it). The node publishes zero velocity when the depth frame is older than
`DEPTH_STALE_SEC` or no goal is set.

## 4. ROS1 (Noetic)

```bash
cd <repo>/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg sru_nav
source devel/setup.bash

bash ros_ws/scripts/start_sru.sh              # SRU node (starts the depth projector on demand)
python3 ros_ws/scripts/sru_cmd_vel_relay.py   # gated relay (separate terminal)
```

The orchestrator (`scripts/ons2_go2.sh --local-planner=sru`, etc.) includes
`launch/sru_nav.launch` via the sru backend in `robots/<robot>/robot.yaml`.
Parameters: `ros_ws/src/sru_nav/config/sru_nav.yaml` (private params, flat form).

> The ROS1 wrapper has not been regression-tested on hardware under this
> repository layout yet (the core itself previously ran on ROS1 robots).

## 5. Key behavior and thresholds

- **Arrival**: built-in `ARRIVE_GOAL_THRESHOLD = 0.6 m` (`constants.py`). The
  mission-layer `sru_arrive_threshold_m` must be **larger** (default 0.7 m),
  otherwise the mission layer never sees SRU as stopped.
- **Depth staleness**: `DEPTH_STALE_SEC = 0.5 s`; older frames → zero velocity
  (ahead of the chassis bridge's 0.5 s timeout stop).
- **On arrival**: clear the goal, publish zero, wait for the next goal.
- **Known limits**: ~95° forward field of view (sides/rear invisible); no global
  planning (dead ends / long corridors need upstream waypoints); occasional
  probe-and-back-off at corners; policy quality depends on training-side
  locomotion and camera-extrinsics consistency.

## 6. See also

- ROS2 package: [`ros2_ws/src/sru_nav/README.md`](../ros2_ws/src/sru_nav/README.md)
- ROS1 package: [`ros_ws/src/sru_nav/README.md`](../ros_ws/src/sru_nav/README.md)
- Integration contract and gate design: `docs/architecture/MODULE_DATAFLOW.md`,
  `docs/usage/03_integrate_your_robot.md`
- Training (separate repository): [SRU-Odin](https://github.com/ManifoldTechLtd/SRU-Odin)