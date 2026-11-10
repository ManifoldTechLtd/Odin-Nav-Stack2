# 02 · 参数调优

[English](02_parameter_tuning.md) | **中文**

本文列出各支持平台上真正重要的参数、改动顺序，以及一张症状对照表。下面的数值是当前仓库里的值。

> **本文与[高级调参指南](../TUNING_GUIDE.zh-CN.md)的分工：**
> 本文回答「**我这台机器人该填什么值**」—— 各平台实测默认值 + 调参顺序；
> 高级调参指南回答「**这个参数到底干什么、该怎么调**」—— 全参数词典 + 症状排障，
> 改动本文未列出的参数前请先读它。

## 1. 参数都在哪

| 层 | Unitree Go2（ROS1） | ASTRALL SW01（ROS1 / ROS2） | Dobot Lumo（ROS1） |
|---|---|---|---|
| 机器人档案（TF、外形、A\*、扫描切片） | `robots/go2/robot.yaml` | `robots/sw01/robot.yaml` | `robots/dobot_lumo/robot.yaml` |
| NeuPAN 局部规划器 | `robots/go2/neupan/config.yaml` + `planner.yaml` | `robots/sw01/neupan/config.yaml` + `planner.yaml` | `robots/dobot_lumo/neupan/config.yaml` + `planner.yaml` |
| 底盘桥 | `ros_ws/src/unitree_control` | `ros2_ws/src/sw01_ros_control` | `ros_ws/src/dobot_ros_control/launch/dobot_control_py.launch` |

NeuPAN 档案在 `robots/<robot>/neupan/` 下，ROS1 与 ROS2 共用。需要时用 `--neupan-config=PATH` 覆盖。**Go2 与 SW01 的档案千万不要混用。**

## 2. 各平台参考值

### Unitree Go2（ROS1）

| 参数 | 文件 | 取值 |
|---|---|---|
| `robot.length` × `robot.width` | `planner.yaml` | 0.70 m × 0.35 m |
| `robot.max_speed`、`robot.max_acce` | `planner.yaml` | `[0.625, 0.875]`、`[0.625, 0.875]` |
| `ref_speed` | `planner.yaml` | 0.625 m/s |
| `receding` × `step_time` | `planner.yaml` | 10 × 0.1 s |
| `ipath.arrive_threshold` | `planner.yaml` | 0.45 m |
| `adjust`：`q_s`、`q_theta`、`p_w`、`eta`、`d_max`、`d_min` | `planner.yaml` | 1.0、2.0、0.8、9.0、0.10、0.02 |
| `w_deadband`、`adaptive_d_max`、`smith_predictor` | `config.yaml` | 全部 `enable: false` |
| `stuck_escape` | `config.yaml` | `enable: true` |

### ASTRALL SW01（ROS2）

| 参数 | 文件 | 取值 |
|---|---|---|
| `robot.length` × `robot.width` | `robots/sw01/neupan/planner.yaml` | 0.96 m × 0.55 m |
| `robot.max_speed`、`robot.max_acce` | `robots/sw01/neupan/planner.yaml` | `[1.0, 1.0]`、`[0.5, 0.5]` |
| `ref_speed` | `robots/sw01/neupan/planner.yaml` | 0.4 m/s |
| `receding` × `step_time` | `robots/sw01/neupan/planner.yaml` | 10 × 0.2 s |
| `ipath.arrive_threshold` | `robots/sw01/neupan/planner.yaml` | 0.25 m |
| `adjust`：`q_s`、`q_theta`、`p_w`、`eta`、`d_max`、`d_min` | `robots/sw01/neupan/planner.yaml` | 1.0、1.0、0.8、12.0、0.4、0.05 |
| `w_deadband` | `robots/sw01/neupan/config.yaml` | `enable: true`、`enter_rad: 0.03`、`exit_rad: 0.067` |
| `adaptive_d_max`、`smith_predictor`、`output_gain` | `robots/sw01/neupan/config.yaml` | 全部 `enable: false` |
| `stuck_escape.rotate_clearance_m` / `rotate_open_clearance_m` | `robots/sw01/neupan/config.yaml` | 0.85 / 0.57 |
| `stuck_escape.strafe_enable`、`corridor_guard_enable` | `robots/sw01/neupan/config.yaml` | 都是 `true` |
| `inflation_radius`、`line_of_sight_safe_distance` | `robots/sw01/robot.yaml`（`navigation.*`） | 0.35 m、0.35 m |
| `footprint.length`、`footprint.width`、`footprint.margin` | `robots/sw01/robot.yaml`（`geometry.footprint`） | 0.96 m、0.55 m、0.10 m |

### Dobot Lumo（ROS1，桥接层）

| 参数 | 文件 | 取值 |
|---|---|---|
| `max_linear_vel`、`max_linear_backward`、`max_angular_vel` | `dobot_control.yaml` | 0.30、0.20、0.50（m/s、m/s、rad/s） |
| `max_accel`、`max_yaw_accel` | `dobot_control.yaml` | 0.40 m/s²、1.00 rad/s² |
| `cmd_vel_timeout` | `dobot_control.yaml` | 0.40 s |
| `enable_strafe` | `dobot_control.yaml` | `false`（Lumo 不能侧移） |
| `fsm_prepare`、`fsm_standing`、`fsm_walk` | `dobot_control.yaml` | **占位值**：请用 `--getfsm` 读到的实际 ID 替换 |

## 3. 调参顺序

一次只改一组，并把结果记下来。

1. **先定外形。** 填入真实的 `length` 和 `width`（以及 `sliding_map` 里的 `footprint_length_m` / `footprint_width_m`）。几何错了，后面每一个症状都会跟着错。

> **换机器人尺寸：改长宽即可，不用重训 DUNE。**
> 把 `robot.length` / `robot.width` 改成新机器人的实测值就够了（同时更新 `sliding_map` 的
> `footprint_length_m` / `footprint_width_m`）。出厂默认 `pan.dune_use_analytic: true` 走的是
> **解析闭式解**，会按 `length × width` 自动适配任意矩形外形，**不需要重新训练 DUNE 模型**。
> 只有当你显式关掉 `dune_use_analytic`、退回神经网络版 DUNE 时，才需要用新尺寸重新训练。
2. **速度。** 降低 `ref_speed` 与 `max_speed`，直到机器人能贴着路径走而不摆动。
3. **净空。** 按你最窄的通道来设 `d_max` 和 `inflation_radius`。`d_max` 应当保持在走廊净空的一半或更低。
4. **跟踪。** 调 `q_s`（位置）与 `q_theta`（航向），再调 `p_w`（角速度阻尼）。
5. **到达与恢复。** 设 `ipath.arrive_threshold`，再设 `stuck_escape` 的净空值。
6. **求解器。** 保持 `solver: 'SCS'`、`nrmp_inference_only: true`、`nrmp_warm_start: true`。不要为了提速而放松求解精度 —— 那等于放弃安全余量。

### 旋转净空规则

矩形外形的外接圆半径是

`R_circ = sqrt(L² + W²) / 2`

把 `stuck_escape.rotate_open_clearance_m` 设为 `≥ R_circ + 0.02 m`，机器人旋转时扫过的圆才不会扫进障碍。

| 机器人 | `R_circ` | `rotate_open_clearance_m` 最小值 |
|---|---|---|
| Go2（0.70 × 0.35） | 0.39 m | 0.41 m |
| SW01（0.96 × 0.55） | 0.55 m | 0.57 m（当前值） |

## 4. 在线调参（ROS1）

NeuPAN 节点运行时会订阅下面这些话题，所以你可以不重启就试一个值：

| 话题 | 对应参数 |
|---|---|
| `/neupan/q_s`、`/neupan/q_theta` | 位置与航向跟踪权重 |
| `/neupan/p_u`、`/neupan/p_w`、`/neupan/p_dw` | 控制量与角速度惩罚 |
| `/neupan/eta`、`/neupan/d_max`、`/neupan/d_min` | 障碍净空项 |

```bash
rostopic pub -1 /neupan/q_theta std_msgs/Float32 "data: 2.0"
rostopic pub -1 /neupan/d_max   std_msgs/Float32 "data: 0.15"
```

调到满意后，把值写回 planner 的 YAML。在线改的值重启就没了。

ROS2 用的是同一套 NeuPAN 内核。依赖这些话题之前，先在你自己的构建上确认（`ros2 topic list | grep neupan`）。

## 5. 症状对照表

| 症状 | 可能原因 | 先改哪个 |
|---|---|---|
| 窄门口犹豫或死锁 | `d_max` 或 `inflation_radius` 相对缝隙太宽 | 降 `d_max`（如 0.40 → 0.20）、降 `eta`、提 `q_theta` |
| 直线走廊里左右摆头 | 角速度振荡 | SW01 上保持 `w_deadband` 开启；提 `p_w`（0.8 → 1.0）；**只有实测到延迟**才开 `smith_predictor` |
| 90° 转角后大幅过冲 | 位置跟踪太弱或视野太短 | 提 `q_s`；降 `ref_speed`；增大 `receding` 或 `step_time` |
| 在终点附近绕圈停不下来 | `arrive_threshold` 小于转弯半径 | 提 `ipath.arrive_threshold`（SW01 0.25 → 0.45 m），让 `atomic_move` 更早接管 |
| 脱困旋转时转进墙里 | 旋转净空低于外接圆半径 | `rotate_open_clearance_m ≥ R_circ + 0.02`；保持 `corridor_guard_enable: true` |
| 为已经消失的障碍刹车 | `sliding_map` 记忆残留，或打到自身外形 | 降 `cell_decay_sec`；把 `footprint_margin_m` 提 0.05 |
| 求解耗时长（动作一顿一顿） | 障碍点太多或视野太长 | 降 `receding`；确认 `solver: 'SCS'` 与 `nrmp_warm_start: true` |

## 6. 各平台注意事项

- **Go2：** `w_deadband`、`adaptive_d_max`、`smith_predictor` 默认都是关的。要开就一个一个开。
- **SW01：** `output_gain` 是 SW01 专用补偿，其他机器人上保持关闭。
- **Dobot Lumo：** `/cmd_vel` 断流时，桥会在 `cmd_vel_timeout` 后下发零速，再经过 `cmd_vel_damp_timeout` 让机器人阻尼停下。**阻尼是终态**：需要发 `/dobot/reset` 才能恢复。除非机器人架在台架上、且有人守在急停旁边，否则不要用 `dry_run:=false` 运行。
