# Robot profiles

One directory per robot. Each profile is the single source of truth for the
robot-specific values that used to be scattered across launch files, YAML
configs, and startup scripts.

```text
robots/
├── sw01/
│   ├── robot.yaml
│   └── neupan/
│       ├── config.yaml
│       └── planner.yaml
├── go2/
│   ├── robot.yaml
│   └── neupan/
│       ├── config.yaml
│       └── planner.yaml
├── dobot_lumo/
│   ├── robot.yaml
│   └── neupan/
│       ├── config.yaml
│       └── planner.yaml
└── _template/
    └── robot.yaml
```

## What goes where

| Section | Content |
|---|---|
| `frames` | `map` / `odom` / `base` / `sensor` frame names |
| `topics` | `odom` / `scan` / `scan_360` / `cmd_vel` topics |
| `geometry` | footprint, sensor mount, `supports_strafe` |
| `bridge` | chassis bridge: `common`, `native`, `ros1`, `ros2` |
| `navigation` | robot-specific global planner and scan-slice parameters |
| `mission` | mission/alignment parameters that differ per robot |
| `local_planner` | selected backend and per-backend interface |

## Rules

- **ROS2 support is the presence of `bridge.ros2`.** No `bridge.ros2` means
  ROS1-only. A robot that supports both runtimes has both `bridge.ros1` and
  `bridge.ros2`.
- `bridge.common` only holds the four values every bridge must obey:
  `cmd_vel_topic`, `max_linear_vel`, `max_angular_vel`, `cmd_vel_timeout`.
- Robot-specific bridge parameters go under `bridge.native`. The presence of
  `native.fsm`, `native.action`, or `native.topics.estop` indicates that the
  robot has FSM, action, or emergency-stop support.
- `local_planner.backend` selects one entry from `local_planner.backends`.
  Each backend has its own `interface` because the control contract differs
  (`mode: action` for NeuPAN, `mode: goal_pose` for SRU).
- Do not add `known_issues`, `capabilities`, or `ros2: true/false` fields.
  Copy the actual values into the profile instead.

## Adding a robot

1. Copy `robots/_template/robot.yaml` to `robots/<name>/robot.yaml`.
2. Fill `frames`, `topics`, `geometry`, and `bridge`.
3. Add `bridge.ros2` only if the robot supports ROS2.
4. Copy the robot-specific NeuPAN pair to `robots/<name>/neupan/` and point
   `local_planner.backends.neupan.config` / `.planner` at it.
5. Fill `navigation`, `mission`, and any additional `local_planner.backends`.
6. Run the temporary profile checker and remove it afterwards.

## Status

The profiles are the source of truth for robot-specific values. `geometry.sensor_mount`
(Static TF), `geometry.footprint` (`sliding_map` self-filter), `navigation.*`
(`pc_to_scan` and `map_planner`), `robots/<robot>/neupan/`, and the SW01 ROS2
`cmd_vel_topic` are wired from `robots/<robot>/robot.yaml`. The Go2, SW01, and
Dobot entries start their drivers, readiness gates, navigation stack, mission,
waypoint, rosbridge, Foxglove, health, and control in tmux dashboards.

## Safety

`dobot_lumo` defaults to `bridge.ros1.dry_run: true` and
`allow_walk_switch: false`. Do not change these without a supervised test on
the real robot.

See [docs/architecture/ROBOT_PROFILE.md](../docs/architecture/ROBOT_PROFILE.md)
for the full schema and migration notes.
