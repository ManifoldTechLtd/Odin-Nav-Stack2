# `sru_nav` — ROS2 package of the SRU navigation node

ROS2 ament package for SRU end-to-end visual navigation (depth + odom + goal →
cmd_vel). The inference core lives in [`sru/src`](../../../sru/src) (single
source shared with ROS1); this package is only the ROS2 wrapper.

## Structure: one wrapper, no copied code

```
sru/                           inference core (shared by ROS1/ROS2)
└── src/            ◄────────┐  model / node / shim / utils …
                             │
ros2_ws/src/sru_nav/         │  ROS2 package (this one)
├── sru  ──────────────────┘  relative symlink → ../../../sru/src
├── launch/sru_nav.launch.py
├── config/sru_nav_ros2.yaml    default parameters (robot-agnostic)
└── test/                       offline checks (shim contract + control-loop smoke)

ros_ws/src/sru_nav/             ROS1 package, same root-level symlink
```

Per-robot parameters (e.g. SW01's conservative `policy_scale`) do **not** live
here: they go in `robots/<robot>/sru/config.yaml`, declared by `robot.yaml`'s
`config:` key and passed by bringup as `params_file` — the same convention as
NeuPAN's `neupan/config.yaml`.

All ROS-version differences are absorbed by `sru/ros_shim.py` (`rospy` ↔
`rclpy`), so `navigation_policy_node.py` contains no raw `rospy` calls and both
distributions run the same node source. `sru` is a symlink, not a copy — edits
apply to both sides instantly. The `sru_nav_node` executable points directly at
`sru.node_main:main`; there is no forwarding layer.

> setuptools only accepts package paths inside the project, hence the symlink
> instead of `package_dir='../../../sru/src'` (silently ignored — the build
> "succeeds" but the library is not installed). Likewise **use `git clone`**;
> zip downloads drop symlinks.

## Build

```bash
cd <repo>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sru_nav --symlink-install
source install/setup.bash
```

`--symlink-install` keeps the installed sources as symlinks, so edits take
effect immediately.

## Run

Entry point: `ros2_ws/scripts/start_sru_ros2.sh` — detects the workspace, blocks
user-site (`PYTHONNOUSERSITE=1`), injects the venv's onnxruntime, and passes
**absolute model paths** to the node (auto-discovery could silently load the
wrong policy).

```bash
# SRU node only (Odin driver and cmd_vel bridge must already be running)
bash ros2_ws/scripts/start_sru_ros2.sh
```

The node publishes `/sru/cmd_vel`; the gated relay
(`ros2_ws/scripts/sru_cmd_vel_relay.py`) decides whether it reaches the chassis
`/cmd_vel`: `/sru/cmd_vel_enable` defaults to false and is opened by the mission
layer. The node itself performs no joystick/deadman checks — permission to move
belongs entirely to this gate chain.

## Models (not in git)

The ONNX models (vae_encoder + navigation policy) must be provided separately:

- place them under `<repo>/sru/models/` (the start script prefers this), or
- point anywhere with `SRU_MODELS_DIR` / `SRU_VAE_MODEL` / `SRU_POLICY_MODEL`.

> When launching manually, always pass explicit absolute `vae_model_path` /
> `policy_model_path` — auto-discovery only matches default file names.

## Verification (no hardware, no onnxruntime needed)

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=<repo>/ros2_ws/src/sru_nav:$PYTHONPATH   # root sru symlink → core
cd <repo>/ros2_ws/src/sru_nav/test

# 1) shim layer: API contract
python3 ros2_shim_check.py --ros-args --params-file params_test.yaml

# 2) node layer: full ROS2 graph with a stub model
python3 node_smoke_ros2.py --ros-args --params-file params_test.yaml
```

Both scripts share `params_test.yaml` (topics deliberately non-default, proving
parameters reach the node), print `PASS/FAIL`, and return non-zero on failure —
CI-ready.

## ROS2 porting pitfalls (fixed)

| Symptom | Cause | Fix |
|---|---|---|
| timer callback `TypeError: missing 1 required positional argument` | rclpy timers pass no argument; rospy passes `TimerEvent` | shim `Timer` synthesizes a `TimerEvent` |
| `<remap>` has no effect | all topic names come from private params (`~cmd_vel_topic`) | use `--params-file`, not `--remap-from` |
| launch `parameters=[...]` ignored | the shim creates its own node, no `parameter_overrides` | launch uses `--ros-args --params-file <yaml>` + `-p k:=v` (latter wins) |
| model path resolution crashes on ROS2 | ROS1 package name hardcoded; `import rospkg` absent | unified package name `sru_nav`, package lookup via the shim |
| core builds but is not installed | `package_dir` outside the project is silently ignored | root-level symlink into the package |