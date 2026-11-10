# ONS2 高级参数调优与排障参考

[English](TUNING_GUIDE.md) | **中文**

ONS2 导航栈的实用参数词典与症状导向排障指南（涉及 `map_planner` 全局 A*、`sliding_map` 360° 障碍记忆、`NeuPAN` MPC + DUNE 局部规划器）。各机器人默认值的快速上手见 [02 · 参数调优](usage/02_parameter_tuning.zh-CN.md)。

---

## 1. 配置分层与生效方式

| 文件 | 作用范围 | 如何生效 |
|---|---|---|
| `robots/<robot>/robot.yaml` | 该机器人的传感器安装 TF、外形尺寸、全局 A* 净空覆盖、启用的规划器/桥接后端 | 重启 `bash scripts/ons2_<robot>.sh` |
| `robots/<robot>/neupan/planner.yaml` | 机器人运动学（`length`、`width`、`max_speed`、`max_acce`）、MPC 视野、DUNE/NRMP 求解设置、`adjust` 权重 | 重启 `bash scripts/ons2_<robot>.sh` |
| `robots/<robot>/neupan/config.yaml` | NeuPAN ROS 封装层话题、`w_deadband`、`adaptive_d_max`、`smith_predictor`、`output_gain`、`stuck_escape` | 重启 `bash scripts/ons2_<robot>.sh` |
| `ros_ws/src/map_planner/config/nav_params.yaml`<br/>`ros2_ws/src/map_planner/config/nav_params.yaml` | 所有机器人共享的全局 A* 算法基线 | 重启 `bash scripts/ons2_<robot>.sh` |

**权重缩放规则**：`adjust:` 下的权重（`q_s`、`q_theta`、`p_u`、`p_w`）在 MPC 目标函数里是**平方后**使用的 —— `(w * x - w * x_ref)^2`。把权重从 `1.0` 降到 `0.1`，该项是放松 **100 倍**（不是 10 倍）。

**优先级图例**：`★★★` = 首要调参项；`★★` = 次要；`★` = 很少改动。

---

## 2. NeuPAN MPC & DUNE 参数（`robots/<robot>/neupan/planner.yaml`）

### 2.1 MPC 视野

| 参数 | SW01 / Go2 默认 | 作用 | 增大 ↑ / 减小 ↓ 的效果 | 优先级 |
|---|---|---|---|---|
| `receding` | `10` / `10` | MPC 预测步数（`T = receding * step_time`） | ↑ 看得更远、提前避障更平滑，但 CPU 求解更贵；↓ 求解更快、视野更短 | `★★` |
| `step_time` | `0.2` / `0.1` | 离散步长（s） | ↑ 单步时间跨度更长、欧拉线性化更粗；↓ 积分更细、总时域更短 | `★` |
| `ref_speed` | `0.4` / `0.625` | 参考前向速度（m/s） | ↑ 通行更快，但制动距离按 `v^2` 增长；↓ 窄空间更安全平滑（**调试碰撞时第一个该降的旋钮**） | `★★★` |
| `collision_threshold` | `0.02` / `0.05` | 最小净空（m），低于此值 NeuPAN 立即停车 | ↑ 停车触发更保守；↓ 允许在密集环境里贴得更近通过 | `★` |

### 2.2 机器人运动学（`robot:`）

| 参数 | SW01 / Go2 默认 | 作用 | 增大 ↑ / 减小 ↓ 的效果 | 优先级 |
|---|---|---|---|---|
| `kinematics` | `'diff'` | 运动学模型（`diff`、`acker`、`omni`） | 差速底盘、四足、人形行走模式都保持 `'diff'` | — |
| `max_speed` | `[1.0, 1.0]` / `[0.625, 0.875]` | 速度上限 `[v_max (m/s), w_max (rad/s)]` | ↑ 峰值速度更高；↓ 相当于全局硬限速 | `★` |
| `max_acce` | `[0.5, 0.5]` / `[0.625, 0.875]` | 加速度上限 `[a_v (m/s^2), a_w (rad/s^2)]` | ↑ 加减速更利落；↓ 更平滑，制动距离更长（`v^2 / 2a`） | `★★` |
| `length` / `width` | `0.96 / 0.55`（SW01）<br/>`0.70 / 0.35`（Go2） | 机器人物理外廓（m），供解析 DUNE 计算障碍几何 | 必须与实测尺寸一致 | `★★★` |

### 2.3 跟踪与避障权重（`adjust:`）

| 参数 | SW01 / Go2 默认 | 作用 | 增大 ↑ / 减小 ↓ 的效果 | 优先级 |
|---|---|---|---|---|
| `q_s` | `1.0` / `1.0` | 沿参考路径的位置跟踪权重 | ↑ 贴路径更紧，但在 A* 尖角处易过冲；↓ 允许更大绕障自由度 | `★★★` |
| `q_theta` | `1.0` / `2.0` | 与路径切向的航向对齐权重 | ↑ 把姿态锁在切向上（不愿偏离避障）；↓ 让 MPC 自主绕开障碍 | `★★★` |
| `p_u` | `1.0` / `1.0` | 对 `ref_speed` 的前向速度跟踪权重 | ↑ 更积极地维持 `ref_speed`；↓ 靠近障碍时更愿意减速 | `★` |
| `p_w` | `0.8` / `0.8` | 角速度阻尼权重 | ↑ 抑制偏航振荡，转向变慢；↓ 转向更灵活，但直线可能摆动 | `★★` |
| `eta` | `12.0` / `9.0` | 净空奖励斜率（期望离障裕度） | ↑ 离墙更远（过高会在窄通道把速度压到低于底盘死区）；↓ 能过更窄的缝隙 | `★★★` |
| `ro_obs` | `400` / `400` | 软障碍约束违反惩罚的刚度 | ↑ 近距排斥更硬；↓ 排斥更软 | `★★` |
| `d_max` | `0.40` / `0.10` | 外侧安全距离带（m），超出此距离的障碍不再产生排斥 | ↑ 排斥缓冲更宽（若大于单侧净空的一半，窄走廊会死锁）；↓ 能过更窄的走廊 | `★★★` |
| `d_min` | `0.05` / `0.02` | 内侧安全距离下限（m） | ↑ 最小裕度更保守；↓ 允许贴得更近 | `★` |
| `bk` | `0.4` / `0.4` | 近端/反向速度比 | ↑ 帧间解更稳；↓ 响应更快 | `★` |
| `solver` | `'SCS'` | 锥规划 QP 求解器 | 保持 `'SCS'` | — |

### 2.4 参考路径（`ipath:`）与 PAN 求解器（`pan:`）

| 参数 | SW01 / Go2 默认 | 作用 | 增大 ↑ / 减小 ↓ 的效果 | 优先级 |
|---|---|---|---|---|
| `ipath.arrive_threshold` | `0.25` / `0.45` | 判定到达目标的距离阈值（m），之后交给 `atomic_move` 做对齐 | ↑ 转弯半径大时避免在目标附近绕圈；↓ 对齐前停得更精准 | `★★` |
| `pan.iter_num` | `2` | 每个控制周期 DUNE-NRMP 交替迭代次数 | 保持 `2`（1 会让动态障碍收敛变差；>2 求解时间翻倍） | `★★` |
| `pan.dune_max_num` | `50` / `100` | DUNE 过滤后保留的最大障碍点数 | ↑ 障碍表达更精细，CPU 负载更高；↓ 求解更快 | `★` |
| `pan.nrmp_max_num` | `10` | NRMP QP 中最大活跃障碍约束数 | 保持 `10` | `★` |
| `pan.dune_use_analytic` | `true` | 解析闭式凸多边形 DUNE 求解（比 MLP 快约 10 倍） | 保持 `true`；会随 `robot.length` × `robot.width` 自动适配 | `★★★` |

---

## 3. NeuPAN ROS 封装层参数（`robots/<robot>/neupan/config.yaml`）

### 3.1 扫描预处理与速度整形

| 配置节 / 参数 | SW01 / Go2 默认 | 作用 | 调参说明 |
|---|---|---|---|
| `topic.scan` | `/scan_360` / `/scan` | 障碍扫描输入话题 | `/scan_360` 含 `sliding_map` 的短时盲区记忆；`/scan` 只有前向视野的实时数据 |
| `scan_range` | `[0.2, 4.5]` | 激光测距过滤 `[min_m, max_m]` | 保持 `max <= 4.5`，与 `sliding_map` 一致 |
| `w_deadband.enable` | `true` / `false` | **软件侧**直线段角速度迟滞死区 | 抑制 `|w| < enter_rad`（`0.03`）直到 `|w| > exit_rad`（`0.067`），生效条件为航向误差 `< 8°`、曲率 `< 0.15`、净空 `> 0.9 m`。**它的用途是防止响应过于灵敏的底盘对零角速度附近的微小抖动做出反应**（表现为直线段左右摆头／S 形）。**如果机器人本身自带速度死区**（多数厂商 SDK 都有），**保持 `false`** —— 叠两层死区会让微调发黏。 |
| `odom_feedback.enable` | `false` | 把实测底盘速度反馈进 MPC 初值 | 默认 `false`。只有当你**实测到**明显的执行延迟、并希望 MPC 从实测状态而非指令状态起步时，才配合 `smith_predictor` 一起打开 |
| `smith_predictor.enable` | `false` | 按 `fixed_lag_sec` + MPC 求解时间前向预测位姿 | 默认 `false` —— 出厂配置**没有启用**。它是给延迟明显的底盘做补偿的辅助手段；没有实测过延迟曲线就打开，跟踪反而可能变差 |
| `output_gain.enable` | `false` | 逆速度增益（`v_fwd_gain`、`v_bwd_gain`、`w_gain`） | **不要凭感觉填这个值。** 先对底盘做**阶跃响应测试**（下发一个已知速度，记录实际达到的稳态速度），再按实测的比例因子来设。没有阶跃数据就是猜 |

### 3.1b 点云高度切片（`min_height` / `max_height`）

这两个参数在 **launch 文件**里（`pc_to_scan` 节点），不在 `config.yaml` 中 —— 它们取决于传感器在你的机器人上怎么安装，而不是规划器行为。

切片高度是**相对传感器坐标系**（如 `odin1_base_link`）给出的，所以同一组数值在不同机器人上对应不同的物理高度。先换算成**相对地面**的高度：

```
地面高度 = 传感器安装高度 + min_height / max_height
```

按"机器人真正会撞到什么"来选这条带子：

| 约束 | 经验取值 |
|---|---|
| **下限** | 略高于地面 —— 通常取**地面以上 0.10~0.20 m**。贴着地面切会把地面纹理、坡道和悬架起伏都当成障碍。 |
| **上限** | 略**高于机器人车身高度**。任何能真正挡住底盘的东西（台阶、托盘边缘、矮推车）都必须落在这条带子里。 |
| **带宽** | 覆盖**整个碰撞包络**，而不只是传感器自身高度。带子过窄是"机器人明明看到墙却还是撞上去"最常见的原因。 |

实例（Go2）：传感器安装在 `0.545 m`，`min_height = -0.1`、`max_height = 0.3` → **地面以上 0.445~0.845 m** 的障碍会被当作障碍物。

这些情况下需要调整：

- 地面**反光、有玻璃或带坡度**，或存在明显俯仰／颠簸 → 抬高下限。
- 经常遇到**底部高于当前带子的高障碍**（货架、桌面、车斗）→ 抬高上限。
- 机器人有**低裙边／保险杠**会比车身更早撞上 → 降低下限，让裙边高度的障碍也能被看到。

这些是 **launch 运行时参数**：改完重启节点即可生效，**不需要重新编译**。

### 3.2 自适应安全带（`adaptive_d_max:`）

根据扫描切片估计局部走廊半宽，并动态收敛 `d_max = clip(half_width - robot_half_width - safety_margin, min, max)`。

| 参数 | 默认 | 作用 | 优先级 |
|---|---|---|---|
| `enable` | `false` | 启用走廊自适应 `d_max` 缩放 | `★★` |
| `min` / `max` | `0.10` / `0.30`（SW01）<br/>`0.10` / `0.20`（Go2） | 动态 `d_max` 的上下限（m）。`min` 必须低于最窄走廊的单侧净空 | `★★★` |
| `safety_margin` | `0.05` | 从估计的走廊半宽里额外扣除的裕度（m） | `★★` |
| `guard_min_dist_m` / `guard_frames` | `0.05` / `3` | 安全网：若障碍距离连续 3 帧低于 `0.05 m`，把 `d_max` 打回基线并保持 `guard_cooldown_sec`（`5.0 s`） | `★★` |

### 3.3 卡滞检测与脱困（`stuck_escape:`）

> **本节是旧版模块。** 下面的参数描述的是早期脱困实现，仅作参考。
> **不推荐在这里调参。**
>
> **保持 `enable: true`**（出厂默认）即可，让规划器自身的校正能力处理卡滞。
> 只有当你要在自己平台上排查脱困行为时，才需要逐个看下面的参数 —— 而且大多数机器人上，
> 出厂默认值本身就是更好的答案。

| 参数 | 默认 | 作用 | 优先级 |
|---|---|---|---|
| `enable` | `true` | 在障碍后卡滞时启用局部后退 + 旋转脱困 | `★★` |
| `criterion` | `progress` | 卡滞判据：`progress`（沿参考路径的弧长推进）或 `displacement`。保持 `progress` | `★★★` |
| `progress_threshold_m` | `0.05` | 在 `displacement_window_sec`（`1.0 s`）内要求的最小弧长推进（m） | `★★` |
| `trigger_frames` | `10` | 进入脱困前需要连续卡滞的帧数 | `★★` |
| `front_obs_threshold` | `1.0` | 前锥（`cone_half_width_rad: 0.9`）判定为"被挡住"的障碍距离阈值（m） | `★★` |
| `back_speed` / `rotate_speed` | `0.15` / `0.5` | 脱困时的后退线速度（m/s）与旋转角速度（rad/s） | `★★` |
| `rotate_gate_enable` / `rotate_clearance_m` | `true` / `0.85` | 原地旋转前要求 `>= rotate_clearance_m`（外接圆半径 + 裕度）；不足时先直后退，避免后角扫进墙里 | `★★★` |
| `corridor_guard_enable` | `true` | 在窄对称走廊里慢速蠕行时，抑制过早脱困（`corridor_standstill_sec: 3.0`） | `★★★` |
| `path_block_grace_enable` | `true`（SW01） | 当实时扫描点在 `path_block_dist_m`（`0.80 m`）内挡住参考路径时，临时放松 `q_theta`（`0.25x`）与 `q_s`（`0.5x`），让 MPC 先尝试绕行再触发脱困 | `★★★` |

---

## 4. 全局规划器（`map_planner`）与 `sliding_map` 参数

配置在 `map_planner/config/nav_params.yaml`（共享基线），并按机器人在 `robots/<robot>/robot.yaml` 的 `navigation:` 与 `geometry.footprint:` 中覆盖。

| 参数 | SW01 / Go2 取值 | 作用 |
|---|---|---|
| `navigation.inflation_radius` | `0.35` / `0.16` | 静态地图上的硬膨胀半径（m）。必须 `<=` 最窄门洞／走廊宽度的一半 |
| `navigation.obstacle_cost_safe_distance` | `0.80` / `0.40` | 软近距代价衰减半径（m），把 A* 路径往走廊中央偏 |
| `navigation.obstacle_cost_weight` | `8.0` / `3.0` | A* 中软近距代价的权重 |
| `navigation.line_of_sight_safe_distance` | `0.35` / `0.25` | 视线拉直（string-pulling）路径简化时要求的最小障碍净空（m） |
| `navigation.smoothing_iterations` | `40` / `20` | 拉普拉斯路径平滑的迭代次数 |
| `navigation.smoothing_max_deviation_m` | `0.35` / `0.20` | 平滑过程中相对原始 A* 路径允许的最大偏移（m） |
| `navigation.min_height` / `max_height` | `-0.3..0.3` / `-0.1..0.3` | 把 `/odin1/cloud_slam` 转成 2D `/scan` 时的 Z 高度切片（m，`odin1_base_link` 系）。选法见 §3.1b |
| `sliding_map.cell_decay_sec` | `8.0` | `/sliding_map` 与 `/scan_360` 里视野外障碍的保留时长（s）；可见的空闲射线会立即清除栅格 |

---

## 5. 症状导向的排障顺序

改动参数前，**先**确认 `/neupan/control_trace`、`/waypoint_manager/status` 与 TF（`map -> base_link`）都正常，并且**每次只改一个参数**。

| 症状 | 第一优先 | 第二优先 | 第三优先 | 根因与理由 |
|---|---|---|---|---|
| **窄通道里犹豫／蹭墙** | 降 `ref_speed`（制动距离 $\propto v^2$） | 降 `q_theta`（让 MPC 敢偏航避障） | 降 `adjust.d_max` 或 `eta`（确保 `d_max <= 净空/2`） | 窄通道里 `d_max` 或 `eta` 过大会被两侧墙同时排斥，把规划速度压到低于底盘死区 |
| **90° 转角后过冲** | 检查 A* 路径的折点与执行延迟 | 降 `ref_speed` 或 `q_s` | 略增 `p_w` | 转角过冲主因是 `速度 × 执行延迟`，叠加过强的 `q_s` 回拉 |
| **直线段轻微左右摆头** | 打开 `w_deadband`（`enable: true`） | 增 `p_w` | 略降 `q_theta` | MPC 在零附近的小过零（`±0.03 rad/s`）若未被 `w_deadband` 门控，会被底盘死区放大 |
| **路径上有障碍却不肯绕行** | 降 `q_theta` 与 `q_s` | 打开 `path_block_grace_enable: true` | 增 `eta` | 强路径锁定（`q_s`、`q_theta`）让 MPC 无法横向偏离去绕过新出现的障碍 |
| **窄走廊里反复后退／脱困循环** | 确认 `corridor_guard_enable: true` | 检查 `front_obs_threshold`（不要设 `> 1.0`） | 检查 `rotate_clearance_m`（`>= 外接圆半径 + 0.25 m`） | 慢速蠕行时侧墙会进入前锥，若关闭 `corridor_guard` 就会误触发脱困 |
| **脱困旋转时后角蹭墙** | 增大 `stuck_escape.rotate_clearance_m` | 打开 `rotate_gate_enable: true` | 打开 `rotate_bounded_enable: true` | 原地旋转会扫过机器人外接圆（`sqrt((L/2)^2 + (W/2)^2)`） |
| **规划报 `START_OCCUPIED` 或 `GOAL_OCCUPIED`** | 降 `navigation.inflation_radius` | 检查 `robot.yaml` 里的 `geometry.footprint` | 检查 `min_height`（`/scan` 里的地面反射） | `inflation_radius` 大于门洞可用净空时，会把有效的起终点栅格标成致命障碍 |
