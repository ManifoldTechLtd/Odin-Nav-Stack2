# Web Console (remote monitoring and control)

**English** | [中文](remote_monitoring.zh-CN.md)

`web_console/` is the browser console for ONS2 robots. It replaces the terminal navigation
console (`test_nav_control_*.py`) and the older `ros2_ws/scripts/remote_monitor.py`, which is now
deprecated. One process, one port, one page:

- **Global map and robot pose** — live map (`/map`) with the robot position and heading (`map → base_link`).
- **Waypoint missions** — click the map to add waypoints (drag to set heading), edit dwell times,
  loop / temporary options, import and export JSON, then run. Pause, resume, skip, cancel,
  return home and reset.
- **Emergency stop and manual takeover** — latched e-stop, teleop with a watchdog (release the
  button or key and the robot stops), keyboard `W A S D Q E`.
- **Chassis enable** (SW01) — enable or disable the motion bridge from the page.
- **Stall alarm** — flags a robot that stops or creeps during a mission; force monitoring, mute and
  window / radius settings.
- **On-demand camera** — frames are only pulled while the camera is on (cellular bandwidth switch).
- **Intercom** — text-to-speech, preset phrases, browser microphone recording and audio upload,
  played on the robot speaker.

ROS 2 (ASTRALL SW01) is the verified target. ROS 1 (Go2, Dobot Lumo) is implemented with the same
behaviour but has not yet been verified on hardware.

---

## Network prerequisite (read this first)

The console only listens on the robot. Reaching it from outside the LAN is your responsibility:

| Option | What you need | Notes |
|---|---|---|
| **Same LAN** | Nothing extra | `http://<robot-ip>:18090`. The browser microphone needs HTTPS unless you use `localhost`. |
| **4G / cellular** | A 4G module (or phone hotspot) plus a reachable public address | Carriers use shared NAT, so you cannot dial in directly. Combine with a tunnel. |
| **NAT traversal / tunnel** | frp, natfrp or an equivalent, plus a VPS if you self-host | Map one public port to `127.0.0.1:18090`. HTTP and HTTPS share that single port. |

> There is no cloud account, relay or default credential in this component.

---

## Security

- **No authentication.** Anyone who can reach the port can drive the robot, including moving it
  and enabling the chassis. Keep it on networks you trust, or put it behind a VPN or a reverse
  proxy with authentication.
- HTTPS uses a self-signed certificate generated on first start; browsers will warn.
- The emergency stop is a software latch. It is **not** a substitute for the physical e-stop.
- Teleop has a watchdog: if the page stops sending velocity commands, the robot stops.

---

## Start

From the repository root, after sourcing the ROS environment:

```bash
# ROS 2 (SW01)
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m web_console.run --port 18090

# ROS 1 (Go2 / Dobot Lumo): source the noetic environment first, then set ROS_VERSION=1
ROS_VERSION=1 /usr/bin/python3 -m web_console.run --port 18090
```

On the SW01 host it runs as the systemd service `scripts/systemd/web-console.service`, so it starts at
boot and is not tied to the navigation tmux session (stopping navigation never stops the console).
The launchers no longer start it.

| Flag | Default | Meaning |
|---|---|---|
| `--host` | `0.0.0.0` | Bind address |
| `--port` | `18090` | HTTP and HTTPS port |
| `--no-tls` | off | Serve plain HTTP only |
| `--map-topic` | `/map` | Occupancy grid |
| `--map-frame` / `--base-frame` | `map` / `base_link` | Pose is `map → base_link` |
| `--pose-hz` | `10` | Pose refresh rate |
| `--odom-topic` | `/odin1/odometry` | Used by the stall detector |
| `--cmd-vel-topic` | `/cmd_vel` | Velocity command topic |
| `--image-topic` | `/odin1/image/compressed` | Camera topic (subscribed only while the camera is on) |
| `--stall-window-sec` / `--stall-radius-m` | `30` / `0.20` | Stall detection window and radius |
| `--ros-version` | `$ROS_VERSION` | `1` or `2` |

---

## Navigation stack and bag recording (SW01)

**Navigation.** Pick a map and a navigation mode, then start or stop the stack. The console runs the
existing launchers (`scripts/ons2_sw01.sh --map=<map> --navigation-mode=<mode>` and
`scripts/ons2_sw01_stop.sh`) and streams their output into the log panel, where `[OK]` gates are green and
failures are red. Maps come from `ros2_ws/src/map_planner/maps/`; `*` marks a map with a relocalization file in
`customer_file/`. Starting is refused while the `ons2_sw01` session exists. Starting navigation does not move
the robot: chassis motion stays disabled until enabled.

**Bag recording.** Tick the topics to record (live list from ROS, with presets for navigation core,
perception and debug). Recording runs `ros2 bag record` in its own tmux session `ons2_bag`, so a console
restart does not interrupt it. Stop sends Ctrl-C so the bag is finalized. Bags are written to
`~/sw01_bag_recordings/` (override with `ONS2_BAG_ROOT`). Topics marked **大** (point clouds, images) make
large files.

Both start/stop actions and recording start/stop ask for confirmation first.

## Interfaces used

- Topics: `/map`, `/odin1/odometry`, `/cmd_vel`, `/odin1/image/compressed`,
  `/waypoint_manager/status`, `/waypoint_manager/system_health`, `/sw01_cmd_vel_bridge/motion_enabled`
  (read); `/cmd_vel`, `/neupan/stop`, `/emergency_stop` (published).
- Services: `/waypoint_manager/start_mission`, `pause`, `resume`, `cancel`, `skip`, `return_home`,
  `reset`, `/waypoint/goal/skip`, `/waypoint/goal/reset`, `/sw01_cmd_vel_bridge/enable` (SW01 only).

The web page only talks to the console's own HTTP API (`/api/state`, `/api/mission/*`,
`/api/nav/command`, `/api/teleop/*`, `/api/chassis/control`, `/api/safety/*`, `/api/camera/*`,
`/api/audio/*`, `/api/map.png`).

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Badge says "无定位" | `map → base_link` TF is not available (localization not running) |
| Map stays empty | `/map` not published (map_server not active) or a different `--map-topic` |
| Start mission fails with "服务 ... 未就绪" | Navigation stack not running, or the service name differs |
| Chassis card says the platform has no enable interface | Not an SW01 build; expected on Go2 / Dobot |
| Teleop refused | Turn on manual takeover first; the watchdog stops motion when the page goes quiet |
| No microphone | Served over plain HTTP on a non-`localhost` origin; use the HTTPS URL |
| Camera stays empty | Camera is off, or `/odin1/image/compressed` is not published |
| Stall alarm while moving | Odometry topic mismatch (`--odom-topic`) or window / radius too tight |
