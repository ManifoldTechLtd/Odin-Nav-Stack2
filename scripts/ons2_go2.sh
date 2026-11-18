#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ONS2_ROBOT="go2"
SESSION="ons2_go2"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
mkdir -p "${LOG_DIR}"

NEUPAN_CONFIG="${REPO_ROOT}/robots/go2/neupan/config.yaml"
ROBOT_PROFILE="${REPO_ROOT}/robots/go2/robot.yaml"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"
ons2_parse_common_args "$@"
ons2_validate_common
BRINGUP_ARGS="$(ons2_ros1_bringup_args "${ROBOT_PROFILE}")"

RELOC_MAP_SWITCHED=0
ODIN_WAIT_SEC="${ODIN_WAIT_SEC:-180}"
RELOC_WAIT_SEC="${RELOC_WAIT_SEC:-180}"
MAP_FILE_NAME="${MAP_NAME:-map_opt}"
ODIN_CFG="${REPO_ROOT}/ros_ws/src/odin_ros_driver/config/control_command.yaml"

if [[ -n "${MAP_NAME}" ]]; then
  MAP_YAML="${REPO_ROOT}/ros_ws/src/map_planner/maps/${MAP_NAME}.yaml"
  MAP_PGM="${REPO_ROOT}/ros_ws/src/map_planner/maps/${MAP_NAME}.pgm"
  [[ -f "${MAP_YAML}" ]] || ons2_fail "--map=${MAP_NAME} missing navigation map: ${MAP_YAML}"
  [[ -f "${MAP_PGM}" ]] || ons2_fail "--map=${MAP_NAME} missing map image: ${MAP_PGM}"
fi
ons2_prepare_odin_reloc_config "${ODIN_CFG}" "${REPO_ROOT}/customer_file/${MAP_FILE_NAME}.bin"

ons2_source_ros1
ons2_require_tmux
ons2_new_session "${SESSION}" roscore

ENV_PREFIX="source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/ros_ws/devel/setup.bash && export GO2_ROOT=${REPO_ROOT} && export ROS_MASTER_URI=http://localhost:11311"
ODIN_NICE="${ODIN_NICE:-0}"
if [[ "${ODIN_NICE}" != "0" ]]; then
  ODIN_CMD="nice -n ${ODIN_NICE} roslaunch odin_ros_driver odin1_ros1.launch"
else
  ODIN_CMD="roslaunch odin_ros_driver odin1_ros1.launch"
fi

ons2_log "w0 starting roscore..."
tmux send-keys -t "${SESSION}:roscore" \
  "source /opt/ros/noetic/setup.bash && roscore 2>&1 | tee ${LOG_DIR}/roscore.log" C-m
timeout 30 bash -c 'until timeout 3 rosparam get /run_id >/dev/null 2>&1; do sleep 1; done' \
  || ons2_fail "ROS Master /run_id not ready (30s timeout)"
rosparam set /global_planner "${GLOBAL_PLANNER}"
rosparam set /local_planner "${LOCAL_PLANNER}"
rosparam set /navigation_mode "${NAVIGATION_MODE}"
ons2_log "  [OK] roscore ready"

tmux new-window -t "${SESSION}" -n odin 'bash --norc'
if [[ "${MOCK_ODIN}" -eq 1 ]]; then
  ODIN_PROC_PATTERN="mock_odin_driver"
  tmux send-keys -t "${SESSION}:odin" \
    "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros_ws/scripts/mock_odin_driver.py --hz 10 2>&1 | tee ${LOG_DIR}/mock_odin.log" C-m
  ons2_log "w1 --mock-odin: starting mock Odin node"
  sleep 5
  pgrep -f "${ODIN_PROC_PATTERN}" >/dev/null || ons2_fail "mock odin failed to start"
  ons2_log "  [OK] mock Odin running (gates skipped)"
else
  ODIN_PROC_PATTERN="odin_ros_driver|host_sdk_sample"
  tmux send-keys -t "${SESSION}:odin" \
    "${ENV_PREFIX} && (rospack find odin_ros_driver >/dev/null 2>&1 || { echo '[ons2_go2][FAIL] odin_ros_driver not found'; exit 1; }) && ${ODIN_CMD} 2>&1 | tee ${LOG_DIR}/odin.log" C-m
  ons2_log "w1 starting odin driver..."
  sleep 20
  pgrep -f "${ODIN_PROC_PATTERN}" >/dev/null || ons2_fail "odin driver process not found"
  if (( RELOC_MAP_SWITCHED )); then
    ons2_wait_window "${SESSION}" odin "Device ready and streams activated" "" "Gate 1: Odin driver ready (map switched, no timeout)" "${ODIN_PROC_PATTERN}"
  else
    ons2_wait_window "${SESSION}" odin "Device ready and streams activated" "${ODIN_WAIT_SEC}" "Gate 1: Odin driver ready" "${ODIN_PROC_PATTERN}"
  fi
  if [[ "${SKIP_RELOC}" -eq 1 ]]; then
    ons2_log "  --skip-reloc: skipping Gate 2"
  else
    ons2_wait_window "${SESSION}" odin "relocalization success" "${RELOC_WAIT_SEC}" "Gate 2: Relocalization succeeded" "${ODIN_PROC_PATTERN}"
  fi
fi

tmux new-window -t "${SESSION}" -n go2 'bash --norc'
tmux send-keys -t "${SESSION}:go2" \
  "${ENV_PREFIX} && roslaunch ons2_bringup go2.launch map_file_name:=${MAP_FILE_NAME} navigation_mode:=${NAVIGATION_MODE} global_planner:=${GLOBAL_PLANNER} local_planner:=${LOCAL_PLANNER} neupan_config:=${NEUPAN_CONFIG} ${BRINGUP_ARGS} 2>&1 | tee ${LOG_DIR}/go2.log" C-m
ons2_log "w2 starting Go2 navigation stack (global_planner=${GLOBAL_PLANNER}, local_planner=${LOCAL_PLANNER}, mode=${LOCAL_PLANNER_MODE})..."
sleep 25
ons2_wait_for "rosnode list" '/vel_to_sdk$' 30 "unitree chassis bridge vel_to_sdk"
ons2_wait_for "rostopic list" '/scan$' 30 "pc_to_scan"
ons2_wait_for "rostopic list" '/scan_360$' 30 "sliding_map"
if [[ "${GLOBAL_PLANNER}" = "map_planner" ]]; then
  ons2_wait_for "rosservice list" '/map_planner/plan$' 30 "map_planner plan service"
fi

if [[ "${LOCAL_PLANNER}" = "neupan" ]]; then
  MISSION_WAIT="until rostopic list 2>/dev/null | grep -qx '/neupan/nav_path/status'; do sleep 1; done; "
else
  MISSION_WAIT=""
fi
tmux new-window -t "${SESSION}" -n mission 'bash --norc'
tmux send-keys -t "${SESSION}:mission" \
  "${ENV_PREFIX} && ${MISSION_WAIT}roslaunch mission_manager mission_manager.launch enable_atomic_move:=true enable_alignment:=true route_segment_enable:=true route_segment_length_m:=10.0 route_segment_overlap_m:=3.0 navigation_mode:=${NAVIGATION_MODE} periodic_replan_sec:=${PERIODIC_REPLAN_SEC} local_planner:=${LOCAL_PLANNER} local_planner_mode:=${LOCAL_PLANNER_MODE} ${MISSION_IFACE_ARGS} 2>&1 | tee ${LOG_DIR}/mission.log" C-m
ons2_log "w4 starting mission_manager..."
sleep 12

ons2_start_window "${SESSION}" waypoint "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros_ws/scripts/waypoint_goal_bridge.py 2>&1 | tee ${LOG_DIR}/waypoint.log"
ons2_start_window "${SESSION}" rosbridge "${ENV_PREFIX} && roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 max_message_size:=100000000 2>&1 | tee ${LOG_DIR}/rosbridge.log"
ons2_start_window "${SESSION}" foxglove "${ENV_PREFIX} && roslaunch foxglove_bridge foxglove_bridge.launch port:=8765 send_buffer_limit:=200000000 2>&1 | tee ${LOG_DIR}/foxglove.log"
ons2_start_window "${SESSION}" health "${ENV_PREFIX} && bash ${SCRIPT_DIR}/go2_health_check.sh --watch 2>&1 | tee ${LOG_DIR}/health.log"
ons2_start_window "${SESSION}" control "${ENV_PREFIX} && export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-} && /usr/bin/python3 ${REPO_ROOT}/ros_ws/scripts/test_nav_control_rospy.py --allow-motion"

ons2_log "=================================================="
ons2_log "Go2 stack started"
ons2_log "  Local planner : ${LOCAL_PLANNER}"
ons2_log "  Navigation    : ${NAVIGATION_MODE}"
ons2_log "  Map           : ${MAP_FILE_NAME}"
ons2_log "  Attach        : tmux attach -t ${SESSION}"
ons2_log "  Stop          : bash ${SCRIPT_DIR}/ons2_go2_stop.sh"
ons2_log "=================================================="
