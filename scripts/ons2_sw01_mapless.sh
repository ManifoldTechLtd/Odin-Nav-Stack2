#!/usr/bin/env bash
# ONS2 SW01 mapless navigation: Odin SLAM mode + empty map + static map->odom identity TF.
# Same bringup as ons2_sw01.sh; only the Odin mode, navigation map and Gate 2 differ.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ONS2_ROBOT="sw01"
SESSION="ons2_sw01"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
mkdir -p "${LOG_DIR}"

ROBOT_PROFILE="${REPO_ROOT}/robots/sw01/robot.yaml"
NEUPAN_CONFIG="$(python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --neupan-config "${ROBOT_PROFILE}")"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"
ons2_parse_common_args "$@"
ons2_validate_common
ons2_source_ros2
[[ -z "${MAP_NAME}" ]] || ons2_fail "--map is not used by mapless navigation (navigation map is maps/empty)"

ODIN_WAIT_SEC="${ODIN_WAIT_SEC:-180}"
MAP_FILE_NAME="empty"
MAP_YAML="${REPO_ROOT}/ros2_ws/src/map_planner/maps/${MAP_FILE_NAME}.yaml"
MAP_PGM="${REPO_ROOT}/ros2_ws/src/map_planner/maps/${MAP_FILE_NAME}.pgm"
ODIN_CONTROL_CONFIG="${REPO_ROOT}/ros2_ws/src/odin_ros_driver/config/control_command.yaml"
[[ -f "${MAP_YAML}" && -f "${MAP_PGM}" ]] || ons2_fail "empty map missing: ${MAP_YAML}"
ons2_prepare_odin_reloc_config "${ODIN_CONTROL_CONFIG}" "" 1

ons2_require_tmux
ons2_new_session "${SESSION}" odin

ENV_PREFIX="source /opt/ros/jazzy/setup.bash && source ${REPO_ROOT}/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-98} && export PYTHONPATH=${REPO_ROOT}/scripts/lib:${PYTHONPATH:-}"

if [[ "${MOCK_ODIN}" -eq 1 ]]; then
  ODIN_PROC_PATTERN="mock_odin_driver"
  tmux send-keys -t "${SESSION}:odin" \
    "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros2_ws/scripts/mock_odin_driver.py --motion static 2>&1 | tee ${LOG_DIR}/mock_odin.log" C-m
  ons2_log "w1 --mock-odin: starting mock Odin node (provides odom->map identity TF)"
  sleep 3
  pgrep -f "${ODIN_PROC_PATTERN}" >/dev/null || ons2_fail "mock odin failed to start"
else
  ODIN_LAUNCH="${REPO_ROOT}/ros2_ws/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py"
  [[ -f "${ODIN_LAUNCH}" ]] || ons2_fail "Odin launch not found: ${ODIN_LAUNCH}"
  ODIN_PROC_PATTERN="odin_ros_driver|host_sdk_sample"
  tmux send-keys -t "${SESSION}:odin" \
    "${ENV_PREFIX} && ros2 launch ${ODIN_LAUNCH} 2>&1 | tee ${LOG_DIR}/odin.log" C-m
  ons2_log "w1 starting odin driver (SLAM mode)..."
  sleep 8
  pgrep -f "${ODIN_PROC_PATTERN}" >/dev/null || ons2_fail "odin driver process not found"
fi

ons2_wait_window "${SESSION}" odin "Device ready and streams activated" "${ODIN_WAIT_SEC}" "Gate 1: Odin driver ready" "${ODIN_PROC_PATTERN}"
ons2_log "  Gate 2 skipped: mapless navigation has no relocalization"

if [[ "${MOCK_ODIN}" -ne 1 ]]; then
  ons2_start_window "${SESSION}" map_odom_tf "${ENV_PREFIX} && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id odom 2>&1 | tee ${LOG_DIR}/map_odom_tf.log"
  ons2_log "w2 static map->odom identity TF (replaces relocalization)"
fi

ons2_start_window "${SESSION}" map_server "${ENV_PREFIX} && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=${MAP_YAML} -p topic_name:=map -p frame_id:=map & sleep 3; until ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate; do echo 'Waiting for map_server lifecycle service...'; sleep 2; done; wait"
ons2_log "w3 starting map_server (empty map)..."

ons2_start_window "${SESSION}" sw01 "${ENV_PREFIX} && ros2 launch ons2_bringup bringup.launch.py profile:=${ROBOT_PROFILE} global_planner:=${GLOBAL_PLANNER} local_planner:=${LOCAL_PLANNER} neupan_config:=${NEUPAN_CONFIG} navigation_mode:=${NAVIGATION_MODE} periodic_replan_sec:=${PERIODIC_REPLAN_SEC} 2>&1 | tee ${LOG_DIR}/sw01.log"
ons2_log "w4 starting SW01 navigation stack (global_planner=${GLOBAL_PLANNER}, local_planner=${LOCAL_PLANNER}, mode=${LOCAL_PLANNER_MODE})..."
sleep 15
ons2_wait_for "ros2 topic list" '/scan$' 30 "pc_to_scan"
ons2_wait_for "ros2 topic list" '/scan_360$' 30 "sliding_map"
ons2_wait_for "ros2 node list" '/sw01_cmd_vel_bridge$' 30 "sw01 bridge"

if [[ "${LOCAL_PLANNER}" = "neupan" ]]; then
  MISSION_WAIT="until ros2 action list 2>/dev/null | grep -qx '/neupan/nav_path'; do sleep 1; done; "
else
  MISSION_WAIT=""
fi
ons2_start_window "${SESSION}" mission "${ENV_PREFIX} && ${MISSION_WAIT}ros2 launch mission_manager mission_manager.launch.py navigation_mode:=${NAVIGATION_MODE} periodic_replan_sec:=${PERIODIC_REPLAN_SEC} local_planner:=${LOCAL_PLANNER} local_planner_mode:=${LOCAL_PLANNER_MODE} ${MISSION_IFACE_ARGS} 2>&1 | tee ${LOG_DIR}/mission.log"
ons2_log "w5 starting mission_manager..."

ons2_start_window "${SESSION}" atomic "${ENV_PREFIX} && ros2 launch atomic_move atomic_move.launch.py 2>&1 | tee ${LOG_DIR}/atomic.log"
ons2_start_window "${SESSION}" waypoint "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros2_ws/scripts/waypoint_goal_bridge.py 2>&1 | tee ${LOG_DIR}/waypoint.log"
ons2_start_window "${SESSION}" rosbridge "${ENV_PREFIX} && ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p max_message_size:=100000000 2>&1 | tee ${LOG_DIR}/rosbridge.log"
ons2_start_window "${SESSION}" foxglove "${ENV_PREFIX} && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p send_buffer_limit:=200000000 2>&1 | tee ${LOG_DIR}/foxglove.log"
ons2_start_window "${SESSION}" health "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros2_ws/scripts/node_health_monitor.py 2>&1 | tee ${LOG_DIR}/health.log"
ons2_start_window "${SESSION}" control "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros2_ws/scripts/test_nav_control_ros2.py --allow-motion"

ons2_log "=================================================="
ons2_log "SW01 mapless navigation started"
ons2_log "  Local planner : ${LOCAL_PLANNER}"
ons2_log "  Navigation    : ${NAVIGATION_MODE}"
ons2_log "  Map           : ${MAP_FILE_NAME} (blank free space, ~100 m x 100 m centred on the start pose)"
ons2_log "  Attach        : tmux attach -t ${SESSION}"
ons2_log "  Stop          : bash ${SCRIPT_DIR}/ons2_sw01_stop.sh"
ons2_log "=================================================="
