# `sru_nav` — ROS1 package of the SRU navigation node

ROS1 catkin package (Noetic) for SRU end-to-end visual navigation (depth + odom
+ goal → cmd_vel). The inference core lives in [`sru/src`](../../../sru/src)
(single source shared with ROS2); this package is only the ROS1 wrapper.
ROS2 counterpart: [`ros2_ws/src/sru_nav`](../../../ros2_ws/src/sru_nav).

## Structure

```
sru/src/              ◄────────┐  navigation library (model / node / shim / utils …)
                               │
ros_ws/src/sru_nav/            │  ROS1 package (this one)
├── sru  ────────────────────┘  relative symlink → ../../../sru/src
├── scripts/sru_nav_node          entry script (calls sru.node_main)
├── setup.py                      catkin_python_setup installs the sru package
├── launch/sru_nav.launch         launch entry (loads the yaml; included by orchestrators)
└── config/sru_nav.yaml           default parameters (private params, flat form)
```

All ROS-version differences are absorbed by `sru/ros_shim.py` (`rospy` ↔
`rclpy`); both sides run the same node source. Symlink, not a copy — edits apply
to both instantly. **Use `git clone`**; zip downloads drop symlinks.

> This wrapper has not been regression-tested on hardware under this repository
> layout yet (the core itself previously ran on ROS1 robots).

## Build

```bash
cd <repo>/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg sru_nav        # or a full catkin_make
source devel/setup.bash
```

Python inference dependencies (onnxruntime etc.): [`sru/requirements.txt`](../../../sru/requirements.txt).

## Run

Entry point `ros_ws/scripts/start_sru.sh`: sources Noetic + the repo devel
space, starts the depth projector (pcd2depth_node) on demand, then
`roslaunch sru_nav sru_nav.launch`. Orchestrators
(`scripts/ons2_*.sh --local-planner=sru`) include this package's launch file
directly via the sru backend in `robots/<robot>/robot.yaml`.

```bash
bash ros_ws/scripts/start_sru.sh
```

The node publishes `/sru/cmd_vel`; the gated relay
(`ros_ws/scripts/sru_cmd_vel_relay.py`) decides whether it reaches the chassis
`/cmd_vel`: `/sru/cmd_vel_enable` defaults to false and is opened by the mission
layer. The node itself performs no joystick/deadman checks — permission to move
belongs entirely to this gate chain.

## Models (not in git)

ONNX models must be provided separately: place them under `<repo>/sru/models/`,
or point to absolute paths with `SRU_VAE_MODEL` / `SRU_POLICY_MODEL` (the launch
file forwards them). When running manually, always pass explicit absolute paths
— auto-discovery only matches default file names.