# ONS2 Web Console User Guide

**English** | [中文](web_console.zh-CN.md)

`web_console/` is the built-in all-in-one **browser control terminal** for ONS2 robots (single process, single port `18090` serving both HTTP and HTTPS). It replaces the legacy terminal navigation scripts (`test_nav_control_*.py`) and deprecated `remote_monitor.py`, letting you manage the entire navigation lifecycle from a browser:

- **Full Navigation Stack Start / Stop (SW01 / ROS 2)** — pick a map, choose the local planner (`NeuPAN` or `SRU` end-to-end policy), select the navigation mode, start or stop the entire stack, and watch real-time readiness gate logs.
- **Global Map & Live Robot Pose** — live `/map` occupancy grid rendering with robot position, heading (`map → base_link`), and planned waypoint routes.
- **Interactive Waypoint Queue & Mission Control** — left-click on the map to add waypoints (hold and drag to set target yaw), edit dwell times, run looped or temporary missions, import/export JSON routes, and pause, resume, skip, cancel, return home, or reset.
- **Chassis Motion Enable Gate (SW01)** — enable or disable the chassis velocity bridge directly from the page, decoupled from stack launch for safety.
- **Latched E-Stop & Manual Teleop Takeover** — latched software emergency stop plus watchdog-protected teleop via on-screen pads or keyboard `W/A/S/D/Q/E` (stops immediately on key/button release).
- **Online ROS 2 Bag Recording** — built-in `轻量 (LIGHT)`, `全量 (TOTAL)`, `导航核心` (Nav Core), `感知` (Perception), and `调试` (Debug) presets that cleanly replace topic selections on click and record inside an isolated `ons2_bag` tmux session.
- **Stall Safety Alarm, On-Demand Camera & Intercom** — automatic stall detection during missions, zero-idle-bandwidth camera streaming (1/2/5 fps), and text-to-speech (TTS), microphone recording, or audio file playback over the robot speaker.

> **Platform support:** ROS 2 Jazzy (ASTRALL SW01) is hardware-verified end-to-end. ROS 1 Noetic (Unitree Go2, Dobot Lumo) shares the same map, mission, e-stop, teleop, camera, and intercom features (the stack launcher and ROS 2 bag recorder cards target SW01 ROS 2).

---

## 1. Deployment and Startup

### 1.1 Autostart via systemd (Recommended on SW01)

Installing `scripts/systemd/web-console.service` starts the Web Console automatically at boot, independent of the navigation `tmux` session (stopping navigation never disconnects the browser):

```bash
REPO=/home/mt/base_inspection   # replace with your repository path

sudo install -m 644 "$REPO/scripts/systemd/web-console.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now web-console.service

# Verify status
systemctl is-active web-console.service
```

> **Note:** When navigation is launched from the Web Console, the `ons2_sw01` session runs inside `web-console.service`'s cgroup. While the robot is standing or navigating, always stop navigation via the **Stop Navigation** button on the page rather than running `systemctl restart web-console.service`.

### 1.2 Manual CLI Launch

Source your ROS workspace from the repository root and run:

```bash
# ROS 2 (Ubuntu 24.04 / Jazzy)
export ROS_DOMAIN_ID=98
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m web_console.run --port 18090

# ROS 1 (Ubuntu 20.04 / Noetic)
source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash
ROS_VERSION=1 /usr/bin/python3 -m web_console.run --port 18090
```

Common flags:

| Flag | Default | Description |
|---|---|---|
| `--host` | `0.0.0.0` | Listen address |
| `--port` | `18090` | Shared HTTP and HTTPS port |
| `--no-tls` | off | Serve plain HTTP only (disable self-signed HTTPS) |
| `--map-topic` | `/map` | Occupancy grid topic |
| `--map-frame` / `--base-frame` | `map` / `base_link` | TF frames for robot pose lookup |
| `--odom-topic` | `/odin1/odometry` | Odometry topic for stall detection |
| `--cmd-vel-topic` | `/cmd_vel` | Teleop velocity command topic |
| `--image-topic` | `/odin1/image/compressed` | On-demand camera topic |
| `--stall-window-sec` / `--stall-radius-m` | `30` / `0.20` | Stall alarm window (s) and radius (m) |

### 1.3 Browser Access & Networking

- **LAN access:** Open `http://<robot-ip>:18090` from any device on the same wired/Wi-Fi network (use `https://<robot-ip>:18090` and accept the self-signed certificate if you need browser microphone access).
- **4G / remote access:** The console includes no cloud relay or built-in authentication. For remote access over cellular/NAT, expose `127.0.0.1:18090` through a VPN or reverse tunnel (e.g., frp) behind an authenticated proxy.

---

## 2. Controlling the Entire Navigation Workflow

Below is the complete workflow for operating **ASTRALL SW01 (ROS 2)** from the browser:

### Step 1: Start the Navigation Stack (`导航（SW01）` Card)

1. **Select Map:** Lists `.yaml/.pgm` maps in `ros2_ws/src/map_planner/maps/` and indicates whether a matching `customer_file/<name>.bin` relocalization map exists.
2. **Select Local Planner:**
   - `NeuPAN` (default): Global A\* path + analytical DUNE local MPC obstacle avoidance.
   - `SRU`: End-to-end depth policy; reveals a policy selector (`main`, `conservative`, `roll_v2`) checking model availability under `sru/models/`.
3. **Select Navigation Mode** (used with NeuPAN):
   - `replan_periodic` (default): Periodic global replanning with immediate blocked-path hot updates.
   - `escape`: Local reverse/turn/strafe stuck escape.
   - `neupan_only`: Single initial plan tracked purely by NeuPAN.
4. Click **Start Navigation** (`启动导航`) and confirm. The console runs `scripts/ons2_sw01.sh` and streams live output with green `[OK]` readiness gates and red error highlights. Once ready, the global map and live robot pose appear on the canvas.

### Step 2: Enable Chassis Motion (`底盘使能` Card)

Starting the navigation stack leaves chassis motion **disabled** by default:
1. Stand the robot up with the handheld remote controller, switch to `MOVE` gait mode, and release joystick authority.
2. Verify surrounding clearance, then click **Enable Chassis** (`使能底盘`, calls `/sw01_cmd_vel_bridge/enable`).
3. Motion automatically disables again on authority loss, SDK link drop, or chassis fault; re-enable from this card once cleared.

### Step 3: Build Waypoint Routes & Run Missions (`航点队列` & `任务` Cards)

1. **Pick waypoints on the map:** With `点选航点` active in the top-right toolbar, **left-click** on the map to append a waypoint, or **left-click and drag** to set its arrival heading `yaw°` (right-drag pans, scroll wheel zooms).
2. **Edit & manage routes:** Adjust waypoint name, `x`, `y`, `yaw°`, and dwell seconds (`停留s`) in the table; toggle **Loop** (`循环`) or **Temporary Insert** (`临时插入`); save or reload routes with **Export JSON** / **Import JSON**.
3. **Dispatch & monitor:** Click **Run Mission** (`执行任务`). The **Mission** (`任务`) card displays live state (`NAVIGATING`, `ALIGNING`, `DWELLING`, `PATH_SAFETY_WAIT`, etc.), progress, active waypoint index, and remaining distance, with controls for **Pause**, **Resume**, **Skip**, **Cancel**, **Return Home**, and **Reset**.

### Step 4: Emergency Stop & Manual Teleop (`急停与遥控` Card)

- **Emergency Stop:** Clicking the red **E-Stop** (`急停`) button latches a stop command on `/emergency_stop` and `/neupan/stop`. Click **Release E-Stop** (`解除急停`) before resuming motion.
- **Manual Takeover:** Click **Enable Manual Takeover** (`开启人工接管`), set linear (`m/s`) and angular (`rad/s`) speeds, and drive using the directional buttons or keyboard **`W` / `S` / `A` / `D` / `Q` / `E`**. A built-in watchdog zeroes velocity as soon as you release the key/button.

### Step 5: Record Diagnostic Bags (`录包` Card)

Bag recording runs `ros2 bag record` in a dedicated `ons2_bag` tmux session and writes to `~/sw01_bag_recordings/` (override via `ONS2_BAG_ROOT`):

1. **Select a preset** (clicking any preset automatically clears and replaces the previous topic selection):
   - **`轻量 (LIGHT)`** (recommended for navigation debugging): Records TF, odometry, `/cmd_vel`, chassis bridge traces, `/scan` & `/scan_360`, global/local paths, `replan_gate_shadow`, `control_trace` / `escape_trace`, mission feedback, and logs with minimal disk I/O overhead.
   - **`全量 (TOTAL)`**: Adds `/odin1/image/compressed`, `/odin1/cloud_slam`, `/odin1/cloud_raw`, `/map`, and `/sliding_map` (topics badged `大` generate large files quickly).
   - Additional quick presets (`导航核心`, `感知`, `调试`) and **Clear Selection** (`清空已选`) are also provided.
2. Optionally filter or toggle individual topics, append custom topic names, enter an optional bag name prefix, and click **Start Recording** (`开始录包`).
3. Click **Stop Recording** (`结束录包`) to send `Ctrl-C` and cleanly finalize `metadata.yaml`.

### Step 6: Safety Alarm, Camera & Intercom

- **Safety Monitor:** Triggers a visual/audible alarm if robot displacement stays within `--stall-radius-m` over `--stall-window-sec` during an active mission; supports forced monitoring, 60 s mute, and live threshold updates.
- **On-Demand Camera:** Subscribes to `/odin1/image/compressed` only while enabled (1 / 2 / 5 fps) to save cellular bandwidth.
- **Intercom:** Supports custom TTS speech, preset phrases, live browser microphone recording (requires HTTPS or `localhost`), and audio file upload played through the robot's speaker.

---

## 3. Troubleshooting

| Symptom | Cause & Resolution |
|---|---|
| Top badge shows `无定位` or map is blank | Stack is not started yet, or Odin relocalization has not converged (verify `.bin` map availability and `map → odom → base_link` TF) |
| Start Navigation says `ons2_sw01` already exists | Click **Stop Navigation** (or run `bash scripts/ons2_sw01_stop.sh`) to clean up the existing session first |
| Robot stays still after dispatching a mission | Ensure **Enable Chassis** (`使能底盘`) is active, the remote controller switched the robot to `MOVE` and released authority, and E-Stop is not latched |
| An SRU policy option is disabled | Place the corresponding `.onnx` model into `sru/models/` (see `sru/README.md`) |
| Control lag during bag recording | Recording raw uncompressed images (`/odin1/image`) or multiple dense clouds saturated disk I/O; use the **`轻量 (LIGHT)`** preset for control debugging |
| Microphone button unavailable | Browsers require HTTPS on non-`localhost` origins; open `https://<robot-ip>:18090` |
