# Running ONS2 in the Isaac Sim (MT-Real2Sim)

This is the single source of truth for running ONS2 against the Isaac simulator.
The simulator repository is
[MT-Real2Sim-Tutorial](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial); integration
assets live in its `integrations/ons2/` directory.

> Status: ROS 2 (Jazzy) is verified end-to-end (3- and 5-waypoint missions `COMPLETED`).
> ROS 1 (Noetic) bridge support is implemented in the simulator; the Noetic-side launch and
> the NeuPAN Python 3.10 sidecar are not yet validated.

## 1. Scope: real robot vs simulation

Simulation support is additive and gated; it does not change how ONS2 runs on a robot.

| | Real robot | Simulation (Isaac Sim) |
|---|---|---|
| Entry point | `scripts/ons2_sw01.sh` (ROS 2), `scripts/ons2_go2.sh` / `scripts/ons2_dobot_lumo.sh` (ROS 1) | `scripts/ons2_sim_ros2.sh` inside the simulator's ROS 2 container |
| Profile | `robots/<robot>/robot.yaml` | `robots/go2_sim/robot.yaml`, reusing `robots/go2/neupan/` |
| Clock | wall clock, `use_sim_time=false` | `use_sim_time:=true` on every node, `/clock` from the bridge |
| Sensors / TF | Odin1 driver publishes `/odin1/*` and TF | `sim_bridge_ros2` publishes `/odin1/*` + `/clock` and subscribes `/cmd_vel`; same TF chain `map→odom→imu→odin1_base_link→base_link` |
| Map | site triplet via `--map=NAME` | `office_0923_sim.pgm` / `.yaml` from the simulator's `integrations/ons2/maps/`, passed via `--map` |
| Chassis | SW01 `[c]` enable gate | not used; `/cmd_vel` drives the simulated robot directly |

The shared ROS 2 launch files only gained a `use_sim_time` argument that defaults to `false`,
and the control console script only gained an opt-in `--use-sim-time` flag;
the simulation profile and `scripts/ons2_sim_ros2.sh` are separate files. No planner,
controller or navigation-tuning code changed, so a real robot behaves exactly as before
unless the simulation flags are used.

### 1.1 What to change for simulation / for the real robot

| Target | Change |
|---|---|
| Simulation | Use `scripts/ons2_sim_ros2.sh` with the simulator map (`--map`), keep `use_sim_time:=true` everywhere and confirm `/clock`; send waypoints with the simulator's `send_waypoints.sh` or the Foxglove `/waypoint_pick` console (§6, §6.1). |
| Real robot | Nothing. Keep `scripts/ons2_*.sh` and `robots/<robot>/...`; do not pass `use_sim_time:=true` (it defaults to `false`). |

Startup is in §5; the symptom table is in §8.

### 1.2 Branch status: `feature/sim-integration`

Six commits / 10 files against the `upstream/main` merge base (+539/−12): the `go2_sim` profile, the
`use_sim_time` argument in the `ons2_bringup` / `mission_manager` / `atomic_move` /
`neupan_ros` launch files, `scripts/ons2_sim_ros2.sh`, the opt-in `--use-sim-time` flag in
`ros2_ws/scripts/test_nav_control_ros2.py`, and these two guides plus the README links. The
branch is 2 commits behind the current `upstream/main` (map-tutorial docs update,
`.gitignore` chore); merge or review that gap before a release. One uncommitted local fix
(`Buffer(..., node=self.node)` in the same console script) makes `--use-sim-time` apply to TF
lookups as well; it is not part of the six commits yet.

## 2. What runs where

```
Isaac Sim (TCP:9090, msgpack)
├─ ROS 2 Jazzy container: sim_bridge_ros2 → /odin1/* + /clock, subscribes /cmd_vel
│   ONS2 ROS 2: go2_sim profile, map_server, sliding_map, map_planner, NeuPAN, mission_manager
└─ ROS 1 Noetic container: sim_bridge (frame_layout=odin), ONS2 ROS 1 (not yet validated)
```

The TF chain is identical to the real Go2:
`map→odom→imu→odin1_base_link→base_link`. See
[simulation-interface.md](https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial/blob/v2/docs/simulation-interface.md)
(or `docs/simulation-interface.md` in a local simulator checkout).

## 3. Prerequisites

* Simulator checkout of `MT-Real2Sim-Tutorial` branch `v2` (or the integration branch) with
  the Isaac image `r2s2r-isaac:5.1.0`.
* A scene directory containing `office_0923.usdz` (not redistributed; set `DATASET_DIR`).
* Docker + NVIDIA container toolkit.
* ONS2 checkout (this repository). The simulator expects it next to itself
  (`../Odin-Nav-Stack2` by default, override with `--ons2-root`).

## 4. One-time build (ROS 2)

```bash
cd MT-Real2Sim-Tutorial/docker
docker compose build ros2

ONS2_ROOT=/abs/path/to/Odin-Nav-Stack2 docker compose up -d ros2
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ons2/ros2_ws &&
  colcon build --symlink-install --packages-select \
    nav_interfaces atomic_move map_planner sliding_map neupan_ros mission_manager \
    ons2_bringup sim_bridge_ros2'
```

NeuPAN runs from source via `PYTHONPATH=/ons2/NeuPAN` (the pip project has no wheel); the
runtime image already contains its dependencies (torch CPU, cvxpylayers, ecos, gctl,
matplotlib, numpy 1.26, scipy 1.12).

**NeuPAN config layout.** Each robot keeps `config.yaml` + `planner.yaml` under
`robots/<robot>/neupan/`. Official profiles: `robots/go2` (Unitree Go2),
`robots/sw01` (Xuanji SW01, analytical DUNE), and `robots/_template` (user-customized
copy). The simulation profile `robots/go2_sim` **reuses** Go2's files through
`local_planner.backends.neupan.config: ../go2/neupan/config.yaml`; do not add a fourth
copy under `go2_sim`.

## 5. Start the simulation

```bash
cd MT-Real2Sim-Tutorial
DATASET_DIR=/abs/path/to/dataset \
  ./start.sh --robot go2 --runtime ros2 --stack ons2 --map office_0923_sim
```

This starts Isaac (headless office scene), the ROS 2 bridge, and the ONS2 stack. Logs:

* bridge: `docker exec isaac_nav_ros2 tail -f /tmp/bridge.log`
* stack: `docker exec isaac_nav_ros2 tail -f /tmp/ons2_sim.log`

Manual equivalent (when Isaac is already running):

```bash
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash &&
  ros2 launch sim_bridge_ros2 bridge.launch.py \
    bridge_config:=/ons2/ros2_ws/src/sim_bridge_ros2/config/go2_sim.yaml'
docker exec isaac_nav_ros2 bash -lc \
  'bash /ons2/scripts/ons2_sim_ros2.sh --map /integrations/ons2/maps/office_0923_sim.yaml'
```

`scripts/ons2_sim_ros2.sh` is the only entry point that passes `use_sim_time:=true`; it
starts `map_server`, the `go2_sim` bringup, `atomic_move`, `mission_manager`, the waypoint
bridge and, when tmux is installed, the control console. If the stack stays silent or does
not move, check §8 before changing any parameter.

## 6. Send waypoints

The real-robot interface is unchanged: `waypoint_goal_bridge` → `/waypoint_manager/start_mission`.

```bash
cd MT-Real2Sim-Tutorial
bash integrations/ons2/waypoints/send_waypoints.sh --runtime ros2 --scene office_0923_sim
```

Monitor:

```bash
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash &&
  ros2 topic echo /waypoint_manager/status'
```

Verified preset missions: a 3-waypoint mission from the spawn area
(`office_0923_sim.yaml`) and a 5-waypoint return route (`office_0923_return5.yaml`).

## 6.1 Interactive console (same script as the real robot)

The simulator runs the official ROS 2 control console
`ros2_ws/scripts/test_nav_control_ros2.py` (the same script `scripts/ons2_sw01.sh`
starts on a real robot) inside a tmux session:

```bash
docker exec -it isaac_nav_ros2 tmux attach -t ons2_sim
```

* Pick points in Foxglove with the **Pose** tool on topic `/waypoint_pick`,
  frame `map` (single- and multi-waypoint collection, pause/resume/cancel/return-home
  are handled by the console).
* The console runs with `--allow-motion`; the simulator additionally passes
  `--use-sim-time` so TF lookups use `/clock` (the public-main script is otherwise
  unchanged).
* Chassis enable `[c]` is SW01-specific and not used in the simulator; motion is
  delivered directly through `/cmd_vel`.

Official startup scripts on public `main`: ROS 1 Go2 `scripts/ons2_go2.sh` →
`ros_ws/scripts/test_nav_control_rospy.py`; ROS 2 `scripts/ons2_sw01.sh` →
`ros2_ws/scripts/test_nav_control_ros2.py`. The old `test_nav_control.py` is superseded
by these two runtime-specific scripts.

## 7. Map notes

* `office_0923_sim.pgm` is derived from an internal partial v3 map (kept as
  `office_0923_v3partial.pgm`). Unknown cells were filled as free; walls (0) and doors
  (128) remain blocked.
* The planner treats every non-free cell as blocked and **relocates goals** that have less
  than ~0.55 m clearance (up to `goal_relocate_max_radius_m=3.5 m`). Pick waypoints with
  ≥1.1 m clearance and validate with `/map_planner/plan` before a mission.
* The 2D map must match the USDZ scene frame. Do not reuse real-site maps with a different
  scene.

## 8. Known issues and troubleshooting

| Symptom | Cause / fix |
|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` around `source /opt/ros/jazzy/setup.bash`; the shipped script disables `set -u` while sourcing. |
| NeuPAN dies with `planner.yaml not found` | Each robot's `neupan/` needs `config.yaml` + `planner.yaml`; `robots/go2_sim` reuses `robots/go2/neupan/` via the profile's `config:` key. |
| `msgpack` / `numpy._core` errors | Image must include `msgpack`, numpy 1.26.4, scipy 1.12.0 (see Dockerfile.ros2). |
| Goals get relocated | Increase waypoint clearance (≥1.1 m) or adjust `goal_relocate_*` parameters. |
| Navigation frozen | All nodes need `use_sim_time:=true` and `/clock` must be published by the bridge. |
| GUI mode does not move | Click the Isaac **Play** button, or run headless (office scene config is headless). |
| No `/odin1/*` topics | Only one TCP client is allowed; keep a single bridge connected, restart the bridge after launching Isaac. |

## 9. ROS 1 runtime

The simulator's ROS 1 `sim_bridge` supports `frame_layout: odin` and the same topics.
Running the Noetic ONS2 stack additionally needs a NeuPAN Python 3.10 sidecar container
(Noetic is Python 3.8) and a Noetic map path argument; this path is tracked as follow-up
work and has not been validated yet.
