# dobot_ros_control

ROS 1 velocity and state bridge for **Dobot Lumo** (`/cmd_vel` -> JSON-RPC `SetVel`).

- **Odometry & obstacle perception** are provided by Odin1 (`/odin1/odometry` + TF + `/odin1/cloud_slam`); this package only forwards velocity commands and polls chassis FSM state.
- **Transport**: JSON-RPC over TCP (default `192.168.5.2:51234` via `scripts/dobot_vel_bridge.py` and `launch/dobot_control_py.launch`).

## ROS Interfaces

| Direction | Topic | Type | Description |
|---|---|---|---|
| Sub (In) | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` -> `vx`, `angular.z` -> `vyaw` (`linear.y` ignored; Lumo does not strafe) |
| Sub (In) | `/estop` | `std_msgs/Bool` | `true` -> zero velocity and enter `PAUSED` |
| Sub (In) | `/dobot/reset` | `std_msgs/Bool` | `true` -> manual reset (`DAMPED -> PREPARE -> WALK`) |
| Sub (In) | `/dobot/action` | `std_msgs/String` | Trigger built-in upper-body action by name |
| Pub (Out) | `/base/state` | `std_msgs/String` / `diagnostic_msgs/DiagnosticArray` | Bridge state, error code, FSM ID, and active velocity |

## FSM & Mode Ownership

```text
STANDING (300) <--allow_walk_switch--> WALK (304) --> RUNNING (SetVel stream)
```

- By default (`dry_run: true`, `allow_walk_switch: false`), the bridge runs in read-only mode without sending velocity or FSM commands.
- When `dry_run: false` and `allow_walk_switch: true`, the bridge only transitions between balanced states (`300` standing <-> `304` walking) when a fresh non-zero `/cmd_vel` arrives, and never switches into damping (`1`) or prepare (`100`).

## Usage

```bash
# 1) Read-only check (default dry_run=true)
roslaunch dobot_ros_control dobot_control_py.launch

# 2) Active velocity forwarding (with safety gantry and operator supervision)
roslaunch dobot_ros_control dobot_control_py.launch dry_run:=false allow_walk_switch:=true
```
