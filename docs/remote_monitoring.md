# Remote Web Monitor

**English** | [中文](remote_monitoring.zh-CN.md)

> [!NOTE]
> **ROS 2 only.** The monitor is written against `rclpy`, so it runs on ROS 2 platforms only
> (ASTRALL SW01 today). **ROS 1 platforms (Go2, Dobot Lumo) are not supported**: the ROS 1
> launchers cannot start it, and no ROS 1 port is provided in this release.

`ros2_ws/scripts/remote_monitor.py` — a single-process, single-port web console for operating
an ONS2 robot from a browser anywhere: live status, on-demand camera, audio intercom, and
navigation control with an emergency stop.

---

## ⚠️ Network prerequisite (read this first)

The script only listens on the robot. **Reaching it from outside the LAN is your
responsibility** — the component does not include any tunnel or relay. You need **one** of:

| Option | What you need | Notes |
|---|---|---|
| **Same LAN** | Nothing extra | `http://<robot-ip>:18088` works directly. Browser microphone needs HTTPS unless you use `localhost`. |
| **4G / cellular** | A 4G module (or phone hotspot) on the robot + a reachable public address | Mobile carriers use shared NAT — you cannot dial in directly. Combine with a tunnel (next row). |
| **NAT traversal / tunnel** | A tunnel service such as SakuraFrp, frp, or natfrp, **plus a VPS if you self-host** | Map one public port to `127.0.0.1:18088`. This is exactly why the server speaks HTTP and HTTPS on a **single port**. |
| **Your own public server** | A VPS with a public IPv4 and a reverse proxy | Point the proxy at the robot over an existing tunnel/VPN. |

**Why a single port matters:** the server auto-detects TLS on the same port, so your tunnel
only has to expose **one** port instead of two. HTTPS is also a hard requirement for browser
microphone access on any non-`localhost` origin.

> There is no cloud account, no relay, and no default credentials in this component.
> Treat it as a service you deploy behind whatever connectivity you already trust.

---

## What it does

| Module | Class | Function |
|---|---|---|
| Safety stall monitor | `StallDetector` | Alerts when net displacement stays below a radius inside a sliding window — distinguishes "stopped", "creeping / blocked" and "odometry dropped out" |
| On-demand camera | `OnDemandImageGate` | Subscribes to `/odin1/image/compressed` **only while the page requests frames**, then downscales/recompresses for constrained links |
| Remote intercom | `AudioIntercom` | Browser microphone or text-to-speech, played through the robot's onboard speaker |
| Navigation & teleop | `NavigationAndTeleopController` | Emergency-stop latch, pause / resume / cancel / skip waypoint, manual takeover with watchdog |
| State hub | `MonitorStateHub` | Aggregates the above for the HTTP API and the page |
| Server | `DualProtocolHTTPServer` | HTTP **and** HTTPS on one port |

Design consequences worth knowing:

- **Zero bandwidth by default** — the image topic is *not* subscribed until the page asks for
  it. An idle console costs the cellular link essentially nothing.
- **Low-frequency polling** — the page fetches `GET /api/status`; no continuous video stream
  unless you turn the camera on.
- **Single process, single port** — default `0.0.0.0:18088`.

---

## Quick start

```bash
# ROS 2 environment (the script needs rclpy)
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash        # or your overlay

# Start the console
python3 ros2_ws/scripts/remote_monitor.py --port 18088
```

Then open `http://<robot-ip>:18088` from the same LAN, or expose it through your tunnel and
open the public URL.

Topics default to the ONS2 contract; override them if your setup differs:

| Flag | Default | Meaning |
|---|---|---|
| `--host` | `0.0.0.0` | Bind address |
| `--port` | `18088` | Bind port (HTTP + HTTPS together) |
| `--odom-topic` | `/odin1/odometry` | Odometry used by the stall monitor |
| `--cmd-vel-topic` | `/cmd_vel` | Motion command topic |
| `--image-topic` | `/odin1/image/compressed` | Compressed image for the on-demand camera |
| `--stall-window-sec` | `30` | Sliding window length for stall detection |
| `--stall-radius-m` | `0.20` | Net displacement radius under which the robot counts as stalled |

---

## What you can do from the page

- **Watch status** — mission state, current waypoint, velocity, stall alarm.
- **Turn the camera on/off** — frames are only pulled while enabled (this is the cellular
  bandwidth switch).
- **Talk to people around the robot** — record from the browser microphone, or type a phrase
  and let TTS speak it on the robot's speaker.
- **Control the mission** — pause / resume / cancel / skip waypoint.
- **Take over manually** — teleop, guarded by a watchdog that stops the robot if the page
  stops sending commands.
- **Emergency stop** — a latch that must be explicitly released.

---

## Dependencies

**Required**: ROS 2 (Jazzy or equivalent) with `rclpy`, `geometry_msgs`, `nav_msgs`,
`sensor_msgs`, `std_msgs`, `std_srvs`; Python 3 standard library; `openssl` for the
self-signed certificate (without it the server runs HTTP only).

**Optional** — the component degrades gracefully rather than failing:

| Missing | Effect |
|---|---|
| `nav_interfaces` (`MissionStatus`) | Cannot read `/waypoint_manager/status`; monitoring falls back to `/cmd_vel` motion only |
| `ffmpeg` | Microphone audio is not transcoded; TTS volume gain unavailable |
| `edge-tts` | TTS falls back to `espeak-ng`, then to a beep (not speech) |
| `Pillow` | Frames are sent unscaled (~490 KB each) — not advisable on cellular |
| `aplay` / `paplay` / `pw-play` / `ffplay` | No audio output at all |
| `amixer` | Mute is not lifted automatically before playback |

---

## Security notes

- **No authentication is built in.** Anyone who can reach the port can drive the robot.
  Put it behind your VPN/tunnel's own access control, or add a reverse proxy with auth.
- **HTTPS is self-signed** (generated on first start). Browsers will warn; that is expected
  for a NAT-traversal deployment.
- The emergency stop is a software latch — it is **not** a substitute for a physical e-stop.
- Bind to `0.0.0.0` only on networks you control.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Page loads but no microphone button / mic does not work | Served over plain HTTP on a non-`localhost` origin — browsers require HTTPS for microphone access. Use the HTTPS URL. |
| Camera panel stays empty | The gate is off, `/odin1/image/compressed` is not published, or the driver's `sendrgbcompressed` is disabled |
| No audio from the robot | No supported player installed, `amixer` left it muted, or the audio device index changed after replug |
| Stall alarm fires while the robot is moving | Odometry topic not matching (`--odom-topic`), or the window/radius is too tight for your speed |
| Cannot connect from outside the LAN | No tunnel configured, or the tunnel maps a different local port than `--port` |
