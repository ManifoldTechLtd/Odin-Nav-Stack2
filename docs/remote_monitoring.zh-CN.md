# Web 控制台（远程监视与控制）

[English](remote_monitoring.md) | **中文**

`web_console/` 是 ONS2 机器人的浏览器控制台，替代了终端导航控制脚本（`test_nav_control_*.py`），
以及旧的 `ros2_ws/scripts/remote_monitor.py`（已废弃）。一个进程、一个端口、一个页面：

- **全局地图与机器人位姿**：实时显示 `/map`，并画出机器人位置和朝向（`map → base_link`）。
- **航点任务**：在地图上左键添加航点（拖动设置朝向），可改停留时间，支持循环、临时插入，
  导入 / 导出 JSON，下发执行。支持暂停、恢复、跳过当前点、取消、回起点、重置。
- **急停与人工接管**：锁存式急停；遥控带看门狗（松开按钮或按键即停），支持键盘 `W A S D Q E`。
- **底盘使能**（仅 SW01）：在页面上使能或断开底盘运动桥。
- **安全告警**：任务执行中机器人停止或原地蠕动时报警；可强制监测、静音、调整窗口和半径。
- **按需相机**：只在打开相机时才拉取图像（蜂窝网络省流量）。
- **喊话**：文字转语音、预设短语、浏览器麦克风录音、上传音频，由机器人扬声器播放。

ROS 2（璇玑 SW01）是已验证的目标平台。ROS 1（Go2、Dobot Lumo）功能一致，但尚未在实机上验证。

---

## 网络前提（请先阅读）

控制台只监听在机器人上，从局域网外访问需要你自行解决连通性：

| 方式 | 需要什么 | 说明 |
|---|---|---|
| **同一局域网** | 无需额外准备 | 直接开 `http://<机器人IP>:18090`。非 `localhost` 场景下，浏览器使用麦克风必须走 HTTPS。 |
| **4G / 蜂窝网络** | 机器人上的 4G 模块（或手机热点）和可达的公网地址 | 运营商是共享 NAT，无法直接拨入，需要配合隧道。 |
| **内网穿透 / 隧道** | frp、natfrp 或同类服务；自建还需要一台 VPS | 把公网一个端口映射到 `127.0.0.1:18090`。HTTP 与 HTTPS 共用这一个端口。 |

> 本组件没有云账号、中继或默认凭据。

---

## 安全

- **没有认证。** 任何能访问端口的人都能驱动机器人，包括运动和使能底盘。请只在可信网络上使用，
  或放在 VPN / 带认证的反向代理之后。
- HTTPS 使用首次启动时生成的自签名证书，浏览器会提示不受信任，这是预期行为。
- 急停是软件锁存，**不能**替代机器人本体的物理急停。
- 遥控有看门狗：页面停止发送速度指令后，机器人会自动停止。

---

## 启动

在仓库根目录执行，先 source ROS 环境：

```bash
# ROS 2（SW01）
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m web_console.run --port 18090

# ROS 1（Go2 / Dobot Lumo）：先 source noetic 环境，并设置 ROS_VERSION=1
ROS_VERSION=1 /usr/bin/python3 -m web_console.run --port 18090
```

也可以由启动脚本在机器人的 tmux 会话里拉起（默认关闭）：

```bash
ENABLE_WEB_CONSOLE=1 bash scripts/ons2_sw01.sh     # 同样支持 ons2_go2.sh / ons2_dobot_lumo.sh
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--host` | `0.0.0.0` | 监听地址 |
| `--port` | `18090` | HTTP 与 HTTPS 共用的端口 |
| `--no-tls` | 关闭 | 只提供明文 HTTP |
| `--map-topic` | `/map` | 占据栅格地图 |
| `--map-frame` / `--base-frame` | `map` / `base_link` | 位姿取自 `map → base_link` |
| `--pose-hz` | `10` | 位姿刷新频率 |
| `--odom-topic` | `/odin1/odometry` | 停止检测使用的里程计 |
| `--cmd-vel-topic` | `/cmd_vel` | 速度指令话题 |
| `--image-topic` | `/odin1/image/compressed` | 相机话题（仅在打开相机时订阅） |
| `--stall-window-sec` / `--stall-radius-m` | `30` / `0.20` | 停止检测的窗口与半径 |
| `--ros-version` | `$ROS_VERSION` | `1` 或 `2` |

启动脚本使用的环境变量：`WEB_CONSOLE_PORT`（默认 `18090`）。

---

## 使用的接口

- 话题（订阅）：`/map`、`/odin1/odometry`、`/cmd_vel`、`/odin1/image/compressed`、
  `/waypoint_manager/status`、`/waypoint_manager/system_health`、`/sw01_cmd_vel_bridge/motion_enabled`；
  （发布）`/cmd_vel`、`/neupan/stop`、`/emergency_stop`。
- 服务：`/waypoint_manager/start_mission`、`pause`、`resume`、`cancel`、`skip`、`return_home`、`reset`，
  `/waypoint/goal/skip`、`/waypoint/goal/reset`，以及 `/sw01_cmd_vel_bridge/enable`（仅 SW01）。

页面只与控制台自身的 HTTP 接口通信（`/api/state`、`/api/mission/*`、`/api/nav/command`、
`/api/teleop/*`、`/api/chassis/control`、`/api/safety/*`、`/api/camera/*`、`/api/audio/*`、`/api/map.png`）。

---

## 常见问题

| 现象 | 可能原因 |
|---|---|
| 徽标显示"无定位" | `map → base_link` TF 不可用（定位未运行） |
| 地图一直是空的 | `/map` 没有发布（map_server 未激活），或 `--map-topic` 不对 |
| 下发任务失败，提示"服务 ... 未就绪" | 导航栈未启动，或服务名不一致 |
| 底盘卡片提示平台无使能接口 | 不是 SW01 版本，Go2 / Dobot 上属正常 |
| 遥控被拒绝 | 先开启人工接管；页面停止发送指令后看门狗会停止运动 |
| 浏览器没有麦克风 | 通过非 `localhost` 的 HTTP 访问，改用 HTTPS 地址 |
| 相机没有画面 | 相机未开启，或 `/odin1/image/compressed` 没有发布 |
| 运动时误报停止告警 | 里程计话题不匹配（`--odom-topic`），或窗口 / 半径设置过小 |
