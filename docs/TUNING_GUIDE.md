# ONS2 Advanced Parameter Tuning & Troubleshooting Reference

Practical parameter dictionary and symptom-based troubleshooting guide for the ONS2 navigation stack (`map_planner` global A*, `sliding_map` 360-degree obstacle memory, and `NeuPAN` MPC + DUNE local planner). For a quick-start overview of per-robot defaults, see [02 · Parameter Tuning](usage/02_parameter_tuning.md).

---

## 1. Configuration Layers & Reloading

| File | Scope | How to Apply Changes |
|---|---|---|
| `robots/<robot>/robot.yaml` | Robot-specific sensor mount TF, footprint, global A* clearance overrides, and active planner/bridge backends | Restart `bash scripts/ons2_<robot>.sh` |
| `robots/<robot>/neupan/planner.yaml` | Robot kinematics (`length`, `width`, `max_speed`, `max_acce`), MPC horizon, DUNE/NRMP solver settings, and `adjust` weights | Restart `bash scripts/ons2_<robot>.sh` |
| `robots/<robot>/neupan/config.yaml` | NeuPAN ROS wrapper topics, `w_deadband`, `adaptive_d_max`, `smith_predictor`, `output_gain`, and `stuck_escape` | Restart `bash scripts/ons2_<robot>.sh` |
| `ros_ws/src/map_planner/config/nav_params.yaml`<br/>`ros2_ws/src/map_planner/config/nav_params.yaml` | Shared global A* algorithm baseline across all robots | Restart `bash scripts/ons2_<robot>.sh` |

**Weight Scaling Rule**: Weights under `adjust:` (`q_s`, `q_theta`, `p_u`, `p_w`) are squared inside the MPC objective `(w * x - w * x_ref)^2`. Reducing a weight from `1.0` to `0.1` relaxes that term by **100x** (not 10x).

**Priority Legend**: `★★★` = primary tuning knob; `★★` = secondary knob; `★` = rarely changed.

---

## 2. NeuPAN MPC & DUNE Parameters (`robots/<robot>/neupan/planner.yaml`)

### 2.1 MPC Horizon

| Parameter | SW01 / Go2 Default | Purpose | Effect of Increasing ↑ / Decreasing ↓ | Priority |
|---|---|---|---|---|
| `receding` | `10` / `10` | Number of MPC prediction steps (`T = receding * step_time`). | ↑ Longer lookahead and smoother early avoidance, higher CPU solve time; ↓ faster solve, shorter lookahead. | `★★` |
| `step_time` | `0.2` / `0.1` | Discretization step size (s). | ↑ Longer time horizon per step, coarser Euler linearization; ↓ finer integration, shorter total horizon. | `★` |
| `ref_speed` | `0.4` / `0.625` | Nominal reference forward speed (m/s). | ↑ Faster traversal, but braking distance grows as `v^2`; ↓ safer and smoother in tight spaces (**first knob to lower when debugging collisions**). | `★★★` |
| `collision_threshold` | `0.02` / `0.05` | Minimum clearance (m) below which NeuPAN halts immediately. | ↑ More conservative stop trigger; ↓ allows closer pass in dense clutter. | `★` |

### 2.2 Robot Kinematics (`robot:`)

| Parameter | SW01 / Go2 Default | Purpose | Effect of Increasing ↑ / Decreasing ↓ | Priority |
|---|---|---|---|---|
| `kinematics` | `'diff'` | Kinematic model (`diff`, `acker`, `omni`). | Keep `'diff'` for differential-drive, quadruped, and humanoid walking modes. | — |
| `max_speed` | `[1.0, 1.0]` / `[0.625, 0.875]` | Upper velocity bounds `[v_max (m/s), w_max (rad/s)]`. | ↑ Higher peak speed; ↓ acts as a hard global speed clamp. | `★` |
| `max_acce` | `[0.5, 0.5]` / `[0.625, 0.875]` | Upper acceleration bounds `[a_v (m/s^2), a_w (rad/s^2)]`. | ↑ Snappier acceleration and braking; ↓ smoother ramps, longer stopping distance (`v^2 / 2a`). | `★★` |
| `length` / `width` | `0.96 / 0.55` (SW01)<br/>`0.70 / 0.35` (Go2) | Physical robot bounding box (m) used by analytical DUNE obstacle geometry. | Must match measured chassis dimensions. | `★★★` |

### 2.3 Tracking & Obstacle Avoidance Weights (`adjust:`)

| Parameter | SW01 / Go2 Default | Purpose | Effect of Increasing ↑ / Decreasing ↓ | Priority |
|---|---|---|---|---|
| `q_s` | `1.0` / `1.0` | Position tracking weight along the reference path. | ↑ Tighter path adherence, but may cause overshoot at sharp A* corners; ↓ more freedom to deviate around obstacles. | `★★★` |
| `q_theta` | `1.0` / `2.0` | Heading alignment weight along the reference path tangent. | ↑ Locks heading to path tangent (reluctant to turn away from obstacles); ↓ allows MPC to steer around obstacles autonomously. | `★★★` |
| `p_u` | `1.0` / `1.0` | Forward speed tracking weight toward `ref_speed`. | ↑ Holds `ref_speed` more aggressively; ↓ slows down more readily near obstacles. | `★` |
| `p_w` | `0.8` / `0.8` | Angular velocity damping weight. | ↑ Suppresses yaw oscillation, slower turn-in; ↓ more agile turning, risk of straight-line wobble. | `★★` |
| `eta` | `12.0` / `9.0` | Clearance reward slope (desired margin from obstacles). | ↑ Stays further from walls (if too high, squeezes speed below chassis deadband in narrow aisles); ↓ passes tighter gaps. | `★★★` |
| `ro_obs` | `400` / `400` | Soft obstacle constraint violation penalty stiffness. | ↑ Harder close-range repulsion; ↓ softer repulsion. | `★★` |
| `d_max` | `0.40` / `0.10` | Outer safety distance band (m) beyond which obstacles exert no repulsion. | ↑ Wider repulsion buffer (can deadlock narrow corridors if > half net clearance); ↓ passes narrower corridors. | `★★★` |
| `d_min` | `0.05` / `0.02` | Inner safety distance floor (m). | ↑ More conservative minimum margin; ↓ allows tighter pass. | `★` |
| `bk` | `0.4` / `0.4` | Proximal / reverse velocity ratio. | ↑ More stable inter-frame solutions; ↓ faster response. | `★` |
| `solver` | `'SCS'` | Conic QP solver. | Keep `'SCS'`. | — |

### 2.4 Reference Path (`ipath:`) & PAN Solver (`pan:`)

| Parameter | SW01 / Go2 Default | Purpose | Effect of Increasing ↑ / Decreasing ↓ | Priority |
|---|---|---|---|---|
| `ipath.arrive_threshold` | `0.25` / `0.45` | Goal arrival distance threshold (m) before handing over to `atomic_move` alignment. | ↑ Prevents orbiting near the goal when turning radius is large; ↓ tighter position stop before alignment. | `★★` |
| `pan.iter_num` | `2` | Alternating DUNE-NRMP iterations per control cycle. | Keep `2` (1 degrades dynamic obstacle convergence; >2 doubles solve time). | `★★` |
| `pan.dune_max_num` | `50` / `100` | Max obstacle points retained after DUNE filtering. | ↑ Finer obstacle representation, higher CPU load; ↓ faster solve. | `★` |
| `pan.nrmp_max_num` | `10` | Max active obstacle constraints in the NRMP QP. | Keep `10`. | `★` |
| `pan.dune_use_analytic` | `true` | Closed-form analytical convex-polygon DUNE solver (~10x faster than MLP). | Keep `true`; adapts automatically to any rectangular `robot.length` × `robot.width`. | `★★★` |

---

## 3. NeuPAN ROS Wrapper Parameters (`robots/<robot>/neupan/config.yaml`)

### 3.1 Scan Preprocessing & Velocity Conditioning

| Section / Parameter | SW01 / Go2 Default | Purpose | Tuning Notes |
|---|---|---|---|
| `topic.scan` | `/scan_360` / `/scan` | Obstacle scan input topic. | `/scan_360` includes `sliding_map` short-term blind-spot memory; `/scan` is live forward FOV only. |
| `scan_range` | `[0.2, 4.5]` | `[min_m, max_m]` laser range filter. | Keep `max <= 4.5` aligned with `sliding_map`. |
| `w_deadband.enable` | `true` / `false` | Hysteresis angular-velocity deadband on straight segments. | Suppresses `|w| < enter_rad` (`0.03`) until `|w| > exit_rad` (`0.067`) when heading error `< 8 deg`, curvature `< 0.15`, and obstacle clearance `> 0.9 m`. Eliminates straight-line yaw hunting. |
| `odom_feedback.enable` | `false` | Feed measured chassis twist into MPC initial state. | Enable alongside `smith_predictor` if compensating large actuation transport delay. |
| `smith_predictor.enable` | `false` | Forward-predict robot pose over `fixed_lag_sec` + MPC solve time. | Compensates transport lag (`max_tau_sec: 0.25`). |
| `output_gain.enable` | `false` | Inverse velocity gain (`v_fwd_gain`, `v_bwd_gain`, `w_gain`). | Used when open-loop chassis velocity tracking exhibits a constant scale factor. |

### 3.2 Adaptive Safety Band (`adaptive_d_max:`)

Estimates local corridor half-width from scan slices and dynamically clamps `d_max = clip(half_width - robot_half_width - safety_margin, min, max)`.

| Parameter | Default | Purpose | Priority |
|---|---|---|---|
| `enable` | `false` | Enable dynamic corridor `d_max` scaling. | `★★` |
| `min` / `max` | `0.10` / `0.30` (SW01)<br/>`0.10` / `0.20` (Go2) | Lower and upper bounds on dynamic `d_max` (m). `min` must stay below the tightest corridor's per-side clearance. | `★★★` |
| `safety_margin` | `0.05` | Extra margin subtracted from estimated corridor half-width (m). | `★★` |
| `guard_min_dist_m` / `guard_frames` | `0.05` / `3` | Safety net: if obstacle distance drops below `0.05 m` for 3 frames, snap `d_max` back to baseline for `guard_cooldown_sec` (`5.0 s`). | `★★` |

### 3.3 Stuck Detection & Escape Recovery (`stuck_escape:`)

| Parameter | Default | Purpose | Priority |
|---|---|---|---|
| `enable` | `true` | Enable local backup + rotation escape when stalled behind an obstacle. | `★★` |
| `criterion` | `progress` | Stall metric: `progress` (arc-length advance along reference path) or `displacement`. Keep `progress`. | `★★★` |
| `progress_threshold_m` | `0.05` | Minimum arc-length progress (m) required within `displacement_window_sec` (`1.0 s`). | `★★` |
| `trigger_frames` | `10` | Consecutive stalled frames before entering escape. | `★★` |
| `front_obs_threshold` | `1.0` | Front cone (`cone_half_width_rad: 0.9`) obstacle distance threshold (m) required to count as blocked. | `★★` |
| `back_speed` / `rotate_speed` | `0.15` / `0.5` | Reverse linear speed (m/s) and rotation angular speed (rad/s) during escape. | `★★` |
| `rotate_gate_enable` / `rotate_clearance_m` | `true` / `0.85` | Require `>= rotate_clearance_m` (circumradius + margin) before unconstrained in-place rotation; otherwise back up straight first so the rear corner does not sweep into walls. | `★★★` |
| `corridor_guard_enable` | `true` | Suppress premature escape while slowly crawling inside a narrow symmetric corridor (`corridor_standstill_sec: 3.0`). | `★★★` |
| `path_block_grace_enable` | `true` (SW01) | Temporarily relax `q_theta` (`0.25x`) and `q_s` (`0.5x`) when live scan points block the reference path within `path_block_dist_m` (`0.80 m`), allowing MPC to steer around the obstacle before triggering escape. | `★★★` |

---

## 4. Global Planner (`map_planner`) & `sliding_map` Parameters

Configured in `map_planner/config/nav_params.yaml` (shared baseline) and overridden per robot in `robots/<robot>/robot.yaml` (`navigation:` and `geometry.footprint:`).

| Parameter | SW01 / Go2 Value | Purpose |
|---|---|---|
| `navigation.inflation_radius` | `0.35` / `0.16` | Hard obstacle inflation radius (m) on the static map. Must be `<=` half of the narrowest doorway/corridor width. |
| `navigation.obstacle_cost_safe_distance` | `0.80` / `0.40` | Soft proximity cost decay radius (m) that biases A* paths toward the corridor center. |
| `navigation.obstacle_cost_weight` | `8.0` / `3.0` | Weight of the soft proximity cost in A*. |
| `navigation.line_of_sight_safe_distance` | `0.35` / `0.25` | Minimum obstacle clearance (m) required during line-of-sight string-pulling path simplification. |
| `navigation.smoothing_iterations` | `40` / `20` | Iterative Laplacian path smoothing passes. |
| `navigation.smoothing_max_deviation_m` | `0.35` / `0.20` | Maximum displacement (m) allowed from the raw A* path during smoothing. |
| `navigation.min_height` / `max_height` | `-0.3..0.3` / `-0.1..0.3` | Z height slice (m in `odin1_base_link` frame) converted from `/odin1/cloud_slam` into 2D `/scan`. |
| `sliding_map.cell_decay_sec` | `8.0` | Retention time (s) for out-of-FOV obstacles in `/sliding_map` and `/scan_360` (visible free rays clear cells immediately). |

---

## 5. Symptom-Based Troubleshooting Order

Always verify `/neupan/control_trace`, `/waypoint_manager/status`, and TF (`map -> base_link`) before changing parameters, and modify only one parameter at a time.

| Symptom | 1st Priority | 2nd Priority | 3rd Priority | Root Cause & Rationale |
|---|---|---|---|---|
| **Hesitates / grazes walls in narrow passages** | Lower `ref_speed` (braking distance $\propto v^2$) | Lower `q_theta` (let MPC yaw away from obstacles) | Lower `adjust.d_max` or `eta` (ensure `d_max <= net_clearance / 2`) | High `d_max` or `eta` in a narrow aisle pushes from both walls simultaneously, squeezing plan speed below the chassis deadband. |
| **Overshoots after 90-degree corners** | Check A* path kinks & actuation lag | Lower `ref_speed` or `q_s` | Increase `p_w` slightly | Corner overshoot is primarily driven by `speed * actuation_lag` plus aggressive `q_s` pull-back. |
| **Slight left-right yaw hunting on straight lines** | Enable `w_deadband` (`enable: true`) | Increase `p_w` | Lower `q_theta` slightly | Small MPC zero-crossings (`±0.03 rad/s`) get amplified by chassis deadband boost if not gated by `w_deadband`. |
| **Refuses to detour around an obstacle on the path** | Lower `q_theta` and `q_s` | Enable `path_block_grace_enable: true` | Increase `eta` | Strong path-locking (`q_s`, `q_theta`) prevents MPC from deviating laterally around a newly appeared obstacle. |
| **Repeated backup/escape loop in a narrow corridor** | Verify `corridor_guard_enable: true` | Check `front_obs_threshold` (do not set `> 1.0`) | Check `rotate_clearance_m` (`>= circumradius + 0.25 m`) | Side walls entering the front cone at slow crawl speed can false-trigger escape if `corridor_guard` is disabled. |
| **Rear corner clips wall when rotating during escape** | Increase `stuck_escape.rotate_clearance_m` | Enable `rotate_gate_enable: true` | Enable `rotate_bounded_enable: true` | In-place rotation sweeps the robot's circumscribed radius (`sqrt((L/2)^2 + (W/2)^2)`). |
| **Plans fails with `START_OCCUPIED` or `GOAL_OCCUPIED`** | Lower `navigation.inflation_radius` | Check `geometry.footprint` in `robot.yaml` | Check `min_height` (ground reflections in `/scan`) | `inflation_radius` larger than the available doorway clearance marks valid start/goal cells as lethal obstacles. |
