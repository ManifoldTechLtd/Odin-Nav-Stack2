#!/usr/bin/env bash
# ONS2 simulation bringup (ROS2 runtime).
# No Odin device, no relocalization gates and no chassis bridge: the simulator
# bridge owns /odin1/* and consumes /cmd_vel.
set -uo pipefail

MAP_YAML="/integrations/ons2/maps/office_0923_sim.yaml"
WITH_ROSBRIDGE=0
WITH_FOXGLOVE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP_YAML="$2"; shift 2 ;;
    --rosbridge) WITH_ROSBRIDGE=1; shift ;;
    --foxglove) WITH_FOXGLOVE=1; shift ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ONS2_SIM_LOG_DIR:-/tmp/ons2_sim_logs}"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
export PYTHONPATH="${REPO_ROOT}/NeuPAN:${REPO_ROOT}/scripts/lib:${PYTHONPATH:-}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
cd "${REPO_ROOT}/NeuPAN"

PROFILE="${REPO_ROOT}/robots/go2_sim/robot.yaml"

log() { echo "[ons2_sim] $*"; }

log "map_server: ${MAP_YAML}"
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="${MAP_YAML}" -p topic_name:=map -p frame_id:=map \
  -p use_sim_time:=true >"${LOG_DIR}/map_server.log" 2>&1 &
MAP_SERVER_PID=$!
sleep 3
for _ in $(seq 1 30); do
  ros2 lifecycle set /map_server configure >/dev/null 2>&1 \
    && ros2 lifecycle set /map_server activate >/dev/null 2>&1 && break
  sleep 2
done
log "map_server up (pid ${MAP_SERVER_PID})"

log "bringup (go2_sim)"
ros2 launch ons2_bringup bringup.launch.py \
  profile:="${PROFILE}" \
  cloud_filter:=off \
  navigation_mode:=replan_periodic \
  use_sim_time:=true >"${LOG_DIR}/bringup.log" 2>&1 &

for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -qx '/scan' \
     && ros2 topic list 2>/dev/null | grep -qx '/scan_360'; then
    break
  fi
  sleep 2
done
log "scan chain: $(ros2 topic list 2>/dev/null | grep -E '^/(scan|scan_360|sliding_map)$' | tr '\n' ' ')"

ros2 launch atomic_move atomic_move.launch.py use_sim_time:=true \
  >"${LOG_DIR}/atomic_move.log" 2>&1 &

ros2 launch mission_manager mission_manager.launch.py \
  navigation_mode:=replan_periodic local_planner:=neupan use_sim_time:=true \
  >"${LOG_DIR}/mission.log" 2>&1 &
sleep 5

log "waypoint bridge"
python3 "${REPO_ROOT}/ros2_ws/scripts/waypoint_goal_bridge.py" \
  --ros-args -p use_sim_time:=true >"${LOG_DIR}/waypoint.log" 2>&1 &

if [[ "${WITH_ROSBRIDGE}" -eq 1 ]]; then
  ros2 run rosbridge_server rosbridge_websocket --ros-args \
    -p port:=9090 -p max_message_size:=100000000 >"${LOG_DIR}/rosbridge.log" 2>&1 &
fi
if pgrep -f "foxglove_bridge" >/dev/null 2>&1; then
  log "foxglove_bridge already running (ws://0.0.0.0:8765)"
elif ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:=8765 -p send_buffer_limit:=200000000 >"${LOG_DIR}/foxglove.log" 2>&1 &
  log "foxglove_bridge started (ws://0.0.0.0:8765)"
else
  log "foxglove_bridge package missing; start it on the host with ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}"
fi

# Interactive console identical to scripts/ons2_sw01.sh:
# Foxglove Pose tool publishes /waypoint_pick, the console runs single/multi
# waypoint missions and pause/resume/cancel/return-home transitions.
if command -v tmux >/dev/null 2>&1; then
  cat > /tmp/ons2_control.sh <<EOF
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source ${REPO_ROOT}/ros2_ws/install/setup.bash
export PYTHONPATH=${REPO_ROOT}/NeuPAN:${REPO_ROOT}/scripts/lib:\${PYTHONPATH:-}
exec python3 ${REPO_ROOT}/ros2_ws/scripts/test_nav_control_ros2.py --allow-motion --use-sim-time
EOF
  chmod +x /tmp/ons2_control.sh
  tmux kill-session -t ons2_sim 2>/dev/null || true
  tmux new-session -d -s ons2_sim -n control -x 220 -y 50 /tmp/ons2_control.sh
  log "control console started; attach with: docker exec -it isaac_nav_ros2 tmux attach -t ons2_sim"
else
  log "tmux not installed; interactive control console skipped"
fi

log "ready; logs in ${LOG_DIR}"
wait
