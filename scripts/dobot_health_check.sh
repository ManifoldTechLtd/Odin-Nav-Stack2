#!/usr/bin/env bash
# Dobot Lumo navigation stack health check.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"

WATCH=0
[[ "${1:-}" == "--watch" ]] && WATCH=1
ons2_run_ros1_health_check "Dobot Lumo Navigation" "${WATCH}" \
  /map_server /pc_to_scan /scan_temporal_filter /sliding_map_node \
  /dobot_vel_bridge /mission_manager /atomic_move /waypoint_goal_bridge
