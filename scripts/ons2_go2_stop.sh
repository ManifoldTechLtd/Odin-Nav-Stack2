#!/usr/bin/env bash
# Stop the ONS2 Go2 tmux stack and clean up stale ROS master registrations.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"

ons2_stop_ros1_stack "ons2_go2_stop" "ons2_go2" "" \
  roslaunch rosmaster host_sdk_sample image_overlay cloud_reprojection \
  map_planner map_server pc_to_scan sliding_map_node unitree_vel_controller \
  mission_manager neupan_upstream atomic_move foxglove_bridge \
  waypoint_goal_bridge rosbridge test_nav_control_rospy \
  go2_health_check health_check odin_slam_temporal_filter \
  sru_nav_node sru_cmd_vel_relay pcd2depth mock_odin_driver
