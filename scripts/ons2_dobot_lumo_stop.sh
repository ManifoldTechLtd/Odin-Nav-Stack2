#!/usr/bin/env bash
# Stop the ONS2 Dobot Lumo tmux stack and clean up residual processes.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"

ons2_stop_ros1_stack "ons2_dobot_lumo_stop" "ons2_dobot_lumo" "dobot_vel_bridge" \
  roslaunch rosmaster odin1_full.launch odin_ros_driver_rev1 \
  host_sdk_sample odin_load_map dobot_vel_bridge map_planner map_server \
  pc_to_scan sliding_map_node scan_temporal_filter mission_manager atomic_move \
  neupan_upstream_action_node neupan_node waypoint_goal_bridge \
  rosbridge foxglove_bridge test_nav_control_rospy dobot_health_check \
  mock_odin_driver
