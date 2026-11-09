# Robot profile schema

This document describes `robots/<robot>/robot.yaml`. The profile is the single
source of truth for robot-specific values that were previously spread across
launch files, YAML configs, and startup scripts.

## Scope

The profile only contains **robot-specific** values. It is not a union of every
parameter in the repository.

- Generic algorithm parameters (point-cloud FOV, Foxglove port, service names,
  QoS, PID gains) stay in the core packages.
- Deployment values (tmux session, log directory, default map) stay in the
  launcher.
- Sensor contract values (`/odin1/*`, internal Odin frames, camera calibration)
  stay in the sensor driver.
- Robot-specific NeuPAN parameters stay in `robots/<robot>/neupan/` as two
  files, not in `robot.yaml`.

## Directory layout

```text
robots/<robot>/
├── robot.yaml
└── neupan/
    ├── config.yaml      # ROS adapter config (topics, frames, behavior)
    └── planner.yaml     # MPC/planner config (robot geometry, DUNE, NRMP)
```

## Top-level sections

```yaml
schema_version: 1
name: sw01
display_name: ASTRALL SW01

frames: ...
topics: ...
geometry: ...
bridge: ...
navigation: ...
mission: ...
global_planner: ...
local_planner: ...
```

### `frames`

```yaml
frames:
  map: map
  odom: odom
  base: base_link
  sensor: odin1_base_link
```

### `topics`

```yaml
topics:
  odom: /odin1/odometry
  scan: /scan
  scan_360: /scan_360
  cmd_vel: /cmd_vel
```

### `geometry`

```yaml
geometry:
  footprint:
    mode: polygon          # polygon | radial
    length: 0.96
    width: 0.55
    margin: 0.10
    radius: 0.35           # used when mode=radial
  sensor_mount:
    parent: odin1_base_link
    child: base_link
    x: -0.42
    y: 0.0
    z: 0.12
    yaw: 0.0
  supports_strafe: true
```

`supports_strafe` replaces a separate capabilities block. `native.fsm`,
`native.action`, and `native.topics.estop` indicate FSM, action, and
emergency-stop support.

### `bridge`

```yaml
bridge:
  common:
    cmd_vel_topic: /cmd_vel
    max_linear_vel: 1.0
    max_angular_vel: 1.0
    cmd_vel_timeout: 0.5
  ros1:
    package: sw01_ros_control
    launch: sw01_control.launch
  ros2:
    package: sw01_ros_control
    launch: sw01_control.launch.py
  native: {}
```

Rules:

- `common` holds only the four values every bridge must obey.
- `native` holds robot-specific parameters (SDK timeouts, network interface,
  deadband, FSM IDs, action playback, emergency-stop topics).
- **ROS2 support is the presence of `bridge.ros2`.** Omit it for ROS1-only
  robots. A dual-runtime robot has both sections.
- A new robot that supports only ROS2 writes `bridge.ros2` only.

### `navigation`

```yaml
navigation:
  inflation_radius: 0.35
  obstacle_cost_safe_distance: 0.80
  obstacle_cost_weight: 8.0
  line_of_sight_safe_distance: 0.35
  smoothing_iterations: 40
  smoothing_max_deviation_m: 0.35
  min_height: -0.3
  max_height: 0.3
  local_planner: neupan
  navigation_mode: replan_periodic
  periodic_replan_sec: 2.0
```

Only values that actually differ between robots belong here. Parameters that
are identical across robots stay in `map_planner/config/nav_params.yaml`: any
key omitted from `navigation:` in `robot.yaml` automatically falls back to the
shared `map_planner/config/nav_params.yaml` baseline.

### `mission`

```yaml
mission:
  enable_alignment: true
  angular_tolerance: 0.3491
  route_segment_enable: true
  route_segment_length_m: 10.0
  route_segment_overlap_m: 3.0
```

Again, only robot-specific differences. Watchdog and PID values that are the
same for all robots stay in `mission_manager.launch`.

### `global_planner`

```yaml
global_planner:
  backend: map_planner
  backends:
    map_planner:
      plan_service: /map_planner/plan
    my_global_planner:
      plan_service: /map_planner/plan
      package: my_global_planner_pkg
      launch: my_global_planner.launch
```

- When `global_planner.backend == map_planner` (default): `ons2_bringup` starts
  the built-in 2D A* `map_planner` node (loading shared defaults from
  `map_planner/config/nav_params.yaml` and applying per-robot overrides from
  `navigation.*`).
- When `global_planner.backend == <custom>`: if `package` and `launch` are
  specified under `backends.<custom>`, `ons2_bringup` (both ROS 1 and ROS 2)
  automatically includes `<package>/launch/<launch>`; if omitted, `ons2_bringup`
  skips launching `map_planner` so the user can start their global planner
  externally.

### `local_planner`

```yaml
local_planner:
  backend: neupan
  backends:
    neupan:
      interface:
        mode: action
        path_topic: /initial_path
        scan_topic: /scan_360
        odom_topic: /odin1/odometry
        cmd_vel_topic: /cmd_vel
        nav_path_action: /neupan/nav_path
        stop_topic: /neupan/stop
      config: neupan/config.yaml
      planner: neupan/planner.yaml
    sru:
      interface:
        mode: goal_pose
        goal_topic: /sru/goal_pose
        cmd_vel_topic: /sru/cmd_vel
        enable_topic: /sru/cmd_vel_enable
        relay_output: /cmd_vel
        odom_topic: /odin1/odometry
      package: sru_nav_go2_ros1
      launch: sru_nav_go2.launch
```

NeuPAN and SRU have different control contracts (verified in the repository):

| Backend | Goal input | Velocity output | Stop/gate | `nav_path` action |
|---|---|---|---|---|
| NeuPAN | action `/neupan/nav_path` | `/cmd_vel` | `/neupan/stop` | yes |
| SRU | topic `/sru/goal_pose` | `/sru/cmd_vel` → relay → `/cmd_vel` | `/sru/cmd_vel_enable` | no |

Because the contracts differ, `interface` is per-backend. `mode: action` and
`mode: goal_pose` are the two supported contracts:

- **Bringup behavior (ROS 1 `ros_ws` & ROS 2 `ros2_ws`)**:
  - When `local_planner.backend == neupan`: `ons2_bringup` launches the built-in `neupan_ros` node with `robots/<robot>/neupan/config.yaml`.
  - When `local_planner.backend == <custom>`: if `backends.<custom>` defines `package` and `launch`, `ons2_bringup` (`go2.launch` / `dobot.launch` in ROS 1, `bringup.launch.py` in ROS 2) automatically includes `<package>/launch/<launch>`; if `package`/`launch` are omitted, `ons2_bringup` skips launching a built-in local planner so the user can start their node externally.
- **`mission_manager` behavior (ROS 1 `ros_ws` & ROS 2 `ros2_ws`)**:
  - `mission_manager` accepts any `local_planner` backend name and selects its runtime state-machine behavior via `local_planner_mode` (`action` vs `goal_pose`, resolved from `backends.<backend>.interface.mode` in `robot.yaml`).
  - Topic and service names declared in `robot.yaml` are automatically injected into `mission_manager` and the chassis bridge at startup:
    - `global_planner.backends.<backend>.plan_service` (default: `/map_planner/plan`)
    - `local_planner.backends.<backend>.interface.nav_path_action` (default: `/neupan/nav_path`)
    - `local_planner.backends.<backend>.interface.stop_topic` (default: `/neupan/stop`)
    - `local_planner.backends.<backend>.interface.goal_topic` (default: `/sru/goal_pose`)
    - `local_planner.backends.<backend>.interface.enable_topic` (default: `/sru/cmd_vel_enable`)
    - `local_planner.backends.<backend>.interface.cmd_vel_topic` (default: `/sru/cmd_vel`)
    - `bridge.{ros1,ros2,common}.cmd_vel_topic` (default: `/cmd_vel`, passed to the chassis bridge launch)

## Merge rules

- `bridge.common` is the default; `bridge.ros1` / `bridge.ros2` override only
  the keys that differ.
- There is no deep merge across runtime sections. A key present in a runtime
  section replaces the `common` key.
- `bridge.native` is not merged into `common`; it is consumed by the
  robot-specific bridge.

## NeuPAN configuration

Each robot has its own pair:

```text
robots/<robot>/neupan/config.yaml
robots/<robot>/neupan/planner.yaml
```

`config.yaml` contains `planner_config_file` and `dune_checkpoint`; `planner.yaml`
contains the robot geometry and MPC/planner weights.

Current loader behavior (`NeuPAN/neupan/ros/neupan_ros.py`):

- `NEUPAN_ROS_CONFIG` is resolved relative to `NeuPAN/neupan/ros/configs/`.
- `planner_config_file` is resolved relative to `NeuPAN/neupan/ros/`, not the
  directory of the config file.
- `dune_checkpoint` is passed through to the DUNE layer.

When the bringup starts using `robots/<robot>/neupan/config.yaml`, the loader
must resolve `planner_config_file` relative to the config file first (or the
path must be adjusted). This is a later-phase change; the first batch only
copies the files and leaves the running stack untouched.

## Adding a robot

1. Copy `robots/_template/robot.yaml` to `robots/<robot>/robot.yaml`.
2. Fill `frames`, `topics`, `geometry`, and `bridge`.
3. Add `bridge.ros2` only if the robot supports ROS2.
4. Copy the NeuPAN pair to `robots/<robot>/neupan/` and point
   `local_planner.backends.neupan` at it.
5. Fill `navigation`, `mission`, and any additional local-planner backend.
6. Run the temporary profile checker and remove it afterwards.

## Current limitations

- `unitree_control` has no ROS parameters; it uses hardcoded `/cmd_vel`,
  network auto-detection, `/unitree/sport_mode_state`, and startup
  `StandUp` / `FreeWalk` / `FreeAvoid(false)`. Profile values are descriptive
  only.
- `sw01_control.launch.py` hardcodes most bridge parameters; only
  `cmd_vel_topic` is exposed as a launch argument.
- `dobot_vel_bridge.py` hardcodes `/cmd_vel`; `dobot_control_py.launch` exposes
  the original parameters. Profile values beyond `dry_run` /
  `allow_walk_switch` are descriptive only.
- `mission_manager` dispatches by `local_planner_mode` (`action` or `goal_pose`), decoupled from the backend name.
- The SW01 ROS2 bringup starts static TF, `pc_to_scan`, `scan_filter`,
  `sliding_map`, `map_planner`, the selected `local_planner` backend, and the bridge.
  `map_server` is started from the shell entry. Static TF (`geometry.sensor_mount`),
  `sliding_map` footprint (`geometry.footprint`), `pc_to_scan` height bounds,
  `map_planner` parameters (`navigation.*`), and `neupan_config` are read
  directly from `robots/sw01/robot.yaml`.
- The ROS1 `ons2_bringup` package starts static TF, `map_server`,
  `pc_to_scan`, `sliding_map`, `map_planner`, the selected `local_planner` backend,
  and the bridge for Go2/Dobot (`dobot.launch` also includes `scan_filter` and
  passes `neupan_launch_prefix` to `neupan.launch`). Static TF
  (`geometry.sensor_mount`), `sliding_map` footprint (`geometry.footprint`),
  `pc_to_scan` height bounds, `map_planner` parameters (`navigation.*`), and
  `local_planner` args are injected from `robots/<robot>/robot.yaml`.
- NeuPAN loads `robots/<robot>/neupan/config.yaml`. `neupan_ros.py` resolves
  `planner_config_file` relative to the config file directory first.
- Mission is started by the shell entry after `/neupan/nav_path` readiness.
- The Go2, SW01, and Dobot entries start their drivers, readiness gates,
  navigation, mission, waypoint, rosbridge, Foxglove, health, and control in
  tmux dashboards.
- `geometry.sensor_mount`, `geometry.footprint`, `navigation.*`,
  `robots/<robot>/neupan/`, and SW01 ROS2 `cmd_vel_topic` are wired from the
  profiles; remaining vendor-specific `bridge.native` values stay in the
  individual bridge launch defaults.

## Safety

Dobot Lumo defaults to `dry_run: true` and `allow_walk_switch: false`. Keep
these defaults unless the robot is supervised and the FSM IDs are verified on
the real hardware.
