#!/usr/bin/env bash
# Stop the ONS2 SW01 tmux stack and clean up residual processes.

set -uo pipefail

SESSION="ons2_sw01"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS_ROOT="${REPO_ROOT}/ros2_ws"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "${ROS2_WS_ROOT}/install/setup.bash" 2>/dev/null || true
set -u

log() { echo -e "[ons2_sw01_stop] $*"; }

log "1. Stopping SW01 bridge with SIGINT (zero velocity)"
pkill -INT -f "sw01_cmd_vel_bridge" 2>/dev/null || true
sleep 3

log "2. Closing tmux session ($SESSION)"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION" && log "   Closed"
else
  log "   Session not found (continuing)"
fi

log "3. Force killing residual processes"
for pat in neupan_action_node neupan_node mission_manager atomic_move \
           map_planner sliding_map pointcloud_to_laserscan scan_temporal_filter \
           nav2_map_server waypoint_goal_bridge test_nav_control_ros2 \
           rosbridge_server rosbridge_websocket foxglove_bridge \
           odin_ros_driver host_sdk_sample mock_odin_driver; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    pkill -9 -f "$pat" 2>/dev/null && log "   Killed: $pat"
  fi
done
pkill -9 -f "ros2 launch ${REPO_ROOT}" 2>/dev/null || true
sleep 2

log "4. Final verification"
LEFT=$(pgrep -a -f "sw01_cmd_vel_bridge|neupan_action_node|neupan_node|mission_manager|atomic_move|map_planner|sliding_map|pointcloud_to_laserscan|scan_temporal_filter|nav2_map_server|waypoint_goal_bridge|test_nav_control_ros2|rosbridge_server|rosbridge_websocket|foxglove_bridge|odin_ros_driver|host_sdk_sample|mock_odin_driver" 2>/dev/null \
       | grep -vE "ons2_sw01_stop|$$" || true)
if [[ -z "$LEFT" ]]; then
  log "   [OK] All processes cleaned up"
else
  log "   [WARN] Remaining processes:"; echo "$LEFT" | head -6
fi
log "Done. WARNING: robot remains in its current motion mode; use the remote to switch to DAMPING/FIXEDDOWN before handling."
