# 02 · Parameter Tuning

This guide lists the parameters that matter for each supported platform, the order in which to change them, and a symptom table. Values below are the ones in the repository today. Read the full rationale in the [tuning guide](../TUNING_GUIDE.md) before changing anything that is not listed here.

## 1. Where the parameters live

| Layer | Unitree Go2 (ROS1) | ASTRALL SW01 (ROS1 / ROS2) | Dobot Lumo (ROS1) |
|---|---|---|---|
| Robot profile (TF, footprint, A\*, scan slice) | `robots/go2/robot.yaml` | `robots/sw01/robot.yaml` | `robots/dobot_lumo/robot.yaml` |
| NeuPAN local planner | `robots/go2/neupan/config.yaml` + `planner.yaml` | `robots/sw01/neupan/config.yaml` + `planner.yaml` | `robots/dobot_lumo/neupan/config.yaml` + `planner.yaml` |
| Chassis bridge | `ros_ws/src/unitree_control` | `ros2_ws/src/sw01_ros_control` | `ros_ws/src/dobot_ros_control/launch/dobot_control_py.launch` |

NeuPAN profiles live in `robots/<robot>/neupan/` and are shared between ROS1 and ROS2. Override with `--neupan-config=PATH` if needed. **Never mix the Go2 and SW01 profiles.**

## 2. Platform reference values

### Unitree Go2 (ROS1)

| Parameter | File | Value |
|---|---|---|
| `robot.length` × `robot.width` | `planner.yaml` | 0.70 m × 0.35 m |
| `robot.max_speed`, `robot.max_acce` | `planner.yaml` | `[0.625, 0.875]`, `[0.625, 0.875]` |
| `ref_speed` | `planner.yaml` | 0.625 m/s |
| `receding` × `step_time` | `planner.yaml` | 10 × 0.1 s |
| `ipath.arrive_threshold` | `planner.yaml` | 0.45 m |
| `adjust`: `q_s`, `q_theta`, `p_w`, `eta`, `d_max`, `d_min` | `planner.yaml` | 1.0, 2.0, 0.8, 9.0, 0.10, 0.02 |
| `w_deadband`, `adaptive_d_max`, `smith_predictor` | `config.yaml` | all `enable: false` |
| `stuck_escape` | `config.yaml` | `enable: true` |

### ASTRALL SW01 (ROS2)

| Parameter | File | Value |
|---|---|---|
| `robot.length` × `robot.width` | `robots/sw01/neupan/planner.yaml` | 0.96 m × 0.55 m |
| `robot.max_speed`, `robot.max_acce` | `robots/sw01/neupan/planner.yaml` | `[1.0, 1.0]`, `[0.5, 0.5]` |
| `ref_speed` | `robots/sw01/neupan/planner.yaml` | 0.4 m/s |
| `receding` × `step_time` | `robots/sw01/neupan/planner.yaml` | 10 × 0.2 s |
| `ipath.arrive_threshold` | `robots/sw01/neupan/planner.yaml` | 0.25 m |
| `adjust`: `q_s`, `q_theta`, `p_w`, `eta`, `d_max`, `d_min` | `robots/sw01/neupan/planner.yaml` | 1.0, 1.0, 0.8, 12.0, 0.4, 0.05 |
| `w_deadband` | `robots/sw01/neupan/config.yaml` | `enable: true`, `enter_rad: 0.03`, `exit_rad: 0.067` |
| `adaptive_d_max`, `smith_predictor`, `output_gain` | `robots/sw01/neupan/config.yaml` | all `enable: false` |
| `stuck_escape.rotate_clearance_m` / `rotate_open_clearance_m` | `robots/sw01/neupan/config.yaml` | 0.85 / 0.57 |
| `stuck_escape.strafe_enable`, `corridor_guard_enable` | `robots/sw01/neupan/config.yaml` | both `true` |
| `inflation_radius`, `line_of_sight_safe_distance` | `robots/sw01/robot.yaml` (`navigation.*`) | 0.35 m, 0.35 m |
| `footprint.length`, `footprint.width`, `footprint.margin` | `robots/sw01/robot.yaml` (`geometry.footprint`) | 0.96 m, 0.55 m, 0.10 m |

### Dobot Lumo (ROS1, bridge)

| Parameter | File | Value |
|---|---|---|
| `max_linear_vel`, `max_linear_backward`, `max_angular_vel` | `dobot_control.yaml` | 0.30, 0.20, 0.50 (m/s, m/s, rad/s) |
| `max_accel`, `max_yaw_accel` | `dobot_control.yaml` | 0.40 m/s², 1.00 rad/s² |
| `cmd_vel_timeout` | `dobot_control.yaml` | 0.40 s |
| `enable_strafe` | `dobot_control.yaml` | `false` (Lumo cannot move sideways) |
| `fsm_prepare`, `fsm_standing`, `fsm_walk` | `dobot_control.yaml` | **placeholders**: replace with the IDs reported by `--getfsm` |

## 3. Tuning order

Change one group at a time, and log the result.

1. **Footprint first.** Enter the real `length` and `width` (and `footprint_length_m` / `footprint_width_m` in `sliding_map`). Wrong geometry causes every later symptom.
2. **Speed.** Lower `ref_speed` and `max_speed` until the robot tracks the path without swaying.
3. **Clearance.** Set `d_max` and `inflation_radius` to fit your narrowest passage. `d_max` should stay at or below half the corridor clearance.
4. **Tracking.** Adjust `q_s` (position) and `q_theta` (heading), then `p_w` (angular damping).
5. **Arrival and recovery.** Set `ipath.arrive_threshold`, then the `stuck_escape` clearances.
6. **Solver.** Leave `solver: 'SCS'`, `nrmp_inference_only: true`, `nrmp_warm_start: true`. Do not loosen solver precision to gain speed: it removes the safety margin.

### Rotation clearance rule

For a rectangular footprint, the circumscribed radius is

`R_circ = sqrt(L² + W²) / 2`

Set `stuck_escape.rotate_open_clearance_m ≥ R_circ + 0.02 m` so the robot does not rotate into an obstacle inside its swing circle.

| Robot | `R_circ` | Minimum `rotate_open_clearance_m` |
|---|---|---|
| Go2 (0.70 × 0.35) | 0.39 m | 0.41 m |
| SW01 (0.96 × 0.55) | 0.55 m | 0.57 m (current value) |

## 4. Online tuning (ROS1)

The NeuPAN node subscribes to these topics while it runs, so you can test a value without a restart:

| Topic | Parameter |
|---|---|
| `/neupan/q_s`, `/neupan/q_theta` | position and heading tracking weights |
| `/neupan/p_u`, `/neupan/p_w`, `/neupan/p_dw` | control and angular-velocity penalties |
| `/neupan/eta`, `/neupan/d_max`, `/neupan/d_min` | obstacle clearance terms |

```bash
rostopic pub -1 /neupan/q_theta std_msgs/Float32 "data: 2.0"
rostopic pub -1 /neupan/d_max   std_msgs/Float32 "data: 0.15"
```

Write the value you like back into the planner YAML. Online changes are lost on restart.

ROS2 uses the same NeuPAN core. Verify the topics on your build (`ros2 topic list | grep neupan`) before relying on them.

## 5. Symptom table

| Symptom | Likely cause | Change first |
|---|---|---|
| Hesitates or deadlocks at a narrow door | `d_max` or `inflation_radius` too wide for the gap | Lower `d_max` (e.g. 0.40 → 0.20), lower `eta`, raise `q_theta` |
| Sways left and right on a straight corridor | Angular oscillation | Keep `w_deadband` on (SW01); raise `p_w` (0.8 → 1.0); enable `smith_predictor` only if the lag is measured |
| Large overshoot after a 90° corner | Position tracking too weak or horizon too short | Raise `q_s`; lower `ref_speed`; increase `receding` or `step_time` |
| Circles near the final waypoint and never finishes | `arrive_threshold` smaller than the turning radius | Raise `ipath.arrive_threshold` (SW01 0.25 → 0.45 m) so `atomic_move` takes over earlier |
| Rotates into a wall during escape | Rotation clearance below the circumscribed radius | `rotate_open_clearance_m ≥ R_circ + 0.02`; keep `corridor_guard_enable: true` |
| Brakes for obstacles that are gone | Stale `sliding_map` memory or self-footprint hits | Lower `cell_decay_sec`; raise `footprint_margin_m` by 0.05 |
| Solve time is high (jerky motion) | Too many obstacle points or receding too long | Lower `receding`; check `solver: 'SCS'` and `nrmp_warm_start: true` |

## 6. Platform-specific cautions

- **Go2:** `w_deadband`, `adaptive_d_max`, and `smith_predictor` are off by default. Enable them one at a time.
- **SW01:** `output_gain` is a SW01-only compensation. Keep it off on other robots.
- **Dobot Lumo:** when `/cmd_vel` goes stale, the bridge sends zero velocity after `cmd_vel_timeout` and damps the robot after a further `cmd_vel_damp_timeout`. Damping is terminal: send `/dobot/reset` to recover. Do not run with `dry_run:=false` unless the robot is on a gantry with an operator at the emergency stop.
