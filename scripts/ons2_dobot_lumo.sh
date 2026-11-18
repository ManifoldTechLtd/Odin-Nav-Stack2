#!/usr/bin/env bash
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ONS2_ROBOT="dobot_lumo"
SESSION="ons2_dobot_lumo"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
mkdir -p "${LOG_DIR}"

NEUPAN_CONFIG="${REPO_ROOT}/robots/dobot_lumo/neupan/config.yaml"
ROBOT_PROFILE="${REPO_ROOT}/robots/dobot_lumo/robot.yaml"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"
ons2_parse_common_args "$@"
ons2_validate_common
BRINGUP_ARGS="$(ons2_ros1_bringup_args "${ROBOT_PROFILE}")"

O1L_TOPIC_PREFIX="${O1L_TOPIC_PREFIX:-odin1}"
O1L_IP="${O1L_IP:-192.168.1.251}"
ROBOT_IP="${ROBOT_IP:-192.168.5.2}"
ODIN_WAIT_SEC="${ODIN_WAIT_SEC:-180}"
RELOC_WAIT_SEC="${RELOC_WAIT_SEC:-180}"
GAP_SEC="${GAP_SEC:-7}"
MAP_FILE_NAME="${MAP_NAME:-map_opt}"
MAP_YAML="${REPO_ROOT}/ros_ws/src/map_planner/maps/${MAP_FILE_NAME}.yaml"
MAP_PGM="${REPO_ROOT}/ros_ws/src/map_planner/maps/${MAP_FILE_NAME}.pgm"
RELOC_BIN="${REPO_ROOT}/customer_file/${MAP_FILE_NAME}.bin"
NEUPAN_PYTHON="${NEUPAN_PYTHON:-${HOME}/miniforge3/envs/neupan311/bin/python}"
NEUPAN_TASKSET="${NEUPAN_TASKSET-taskset -c 0,1}"

[[ -f "${MAP_YAML}" ]] || ons2_fail "--map=${MAP_FILE_NAME} missing navigation map: ${MAP_YAML}"
[[ -f "${MAP_PGM}" ]] || ons2_fail "--map=${MAP_FILE_NAME} missing map image: ${MAP_PGM}"
if [[ "${MOCK_ODIN}" -ne 1 && "${SKIP_RELOC}" -ne 1 ]]; then
  [[ -f "${RELOC_BIN}" ]] || ons2_fail "--map=${MAP_FILE_NAME} missing relocalization map: ${RELOC_BIN}"
fi
if [[ "${LOCAL_PLANNER}" = "neupan" ]]; then
  command -v "${NEUPAN_PYTHON}" >/dev/null 2>&1 \
    || ons2_fail "NeuPAN Python not found: ${NEUPAN_PYTHON}"
fi

ons2_log "map: ${MAP_YAML}"
ons2_log "     ${MAP_PGM}"
if [[ "${MOCK_ODIN}" -eq 1 || "${SKIP_RELOC}" -eq 1 ]]; then
  ons2_log "     relocalization: skipped"
else
  ons2_log "     ${RELOC_BIN}"
fi
ping -c1 -W2 "${ROBOT_IP}" >/dev/null 2>&1 \
  || ons2_log "WARN: robot ${ROBOT_IP} unreachable"
ping -c1 -W2 "${O1L_IP}" >/dev/null 2>&1 \
  || ons2_log "WARN: O1L ${O1L_IP} unreachable"

ons2_source_ros1
ons2_require_tmux
ons2_new_session "${SESSION}" roscore

ENV_PREFIX="source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/ros_ws/devel/setup.bash && export GO2_ROOT=${REPO_ROOT} && export ROS_MASTER_URI=http://localhost:11311 && export O1L_TOPIC_PREFIX=${O1L_TOPIC_PREFIX} && export O1L_IP=${O1L_IP} && export ROBOT_IP=${ROBOT_IP}"

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
  ODIN_PROC_PATTERN="odin1_full.launch|odin_ros_driver_rev1"
  if [[ -n "${ODIN_ALGO_LIB_PATH:-}" ]]; then
    printf -v ODIN_ENV 'env ODIN_ALGO_LIB_PATH=%q' "${ODIN_ALGO_LIB_PATH}"
  else
    ODIN_ENV="env -u ODIN_ALGO_LIB_PATH"
  fi
  tmux send-keys -t "${SESSION}:odin" \
    "${ENV_PREFIX} && ${ODIN_ENV} O1L_TOPIC_PREFIX=${O1L_TOPIC_PREFIX} roslaunch dobot_ros_control odin1_full.launch topic_prefix:=${O1L_TOPIC_PREFIX} start_rviz:=false 2>&1 | tee ${LOG_DIR}/odin.log" C-m
  ons2_log "w1 starting O1L driver..."
  sleep 10
  pgrep -f "${ODIN_PROC_PATTERN}" >/dev/null || ons2_fail "O1L driver process not found"
  ons2_wait_window "${SESSION}" odin "DeviceProcessor initialized\|New device discovered" \
    "${ODIN_WAIT_SEC}" "Gate 1: O1L device discovered" "${ODIN_PROC_PATTERN}"
fi

if [[ "${MOCK_ODIN}" -eq 1 || "${SKIP_RELOC}" -eq 1 ]]; then
  ons2_log "  --mock-odin/--skip-reloc: skipping map load and Gate 2"
else
  LOAD_STATUS="${LOG_DIR}/load_map.exit"
  rm -f "${LOAD_STATUS}"
  : > "${LOG_DIR}/load_map.log"
  printf -v LOAD_CMD 'python3 -u %q --map-bin %q --prefix %q --timeout %q' \
    "${REPO_ROOT}/ros_ws/scripts/odin_load_map.py" "${RELOC_BIN}" "${O1L_TOPIC_PREFIX}" "${RELOC_WAIT_SEC}"
  tmux new-window -t "${SESSION}" -n load_map 'bash --norc'
  tmux send-keys -t "${SESSION}:load_map" \
    "${ENV_PREFIX} && ${LOAD_CMD} 2>&1 | tee ${LOG_DIR}/load_map.log; rc=\${PIPESTATUS[0]}; printf '%s\n' \"\$rc\" > ${LOAD_STATUS}" C-m
  ons2_log "w2 loading relocalization map..."
  lm_ok=0
  t0=${SECONDS}
  while (( SECONDS - t0 < RELOC_WAIT_SEC )); do
    if [[ -s "${LOAD_STATUS}" ]]; then
      load_rc="$(cat "${LOAD_STATUS}")"
      if [[ "${load_rc}" != "0" ]]; then
        tail -n 15 "${LOG_DIR}/load_map.log" >&2 || true
        ons2_fail "load_map failed (exit ${load_rc})"
      fi
      lm_ok=1
      break
    fi
    sleep 2
  done
  (( lm_ok )) || ons2_fail "load_map did not complete (check ${LOG_DIR}/load_map.log)"
  ons2_log "  [OK] map loaded"
  sleep 8
  python3 "${REPO_ROOT}/ros_ws/scripts/launch_gate.py" --wait-tf odom map --timeout "${RELOC_WAIT_SEC}" \
    || ons2_fail "Gate 2: relocalization timed out"
  ons2_log "  [OK] Gate 2: relocalization succeeded"
fi

NEUPAN_LAUNCH_PREFIX="${NEUPAN_TASKSET:+${NEUPAN_TASKSET} }${NEUPAN_PYTHON}"
printf -v NEUPAN_PREFIX_ARG 'neupan_launch_prefix:=%s' "${NEUPAN_LAUNCH_PREFIX}"
tmux new-window -t "${SESSION}" -n dobot 'bash --norc'
tmux send-keys -t "${SESSION}:dobot" \
  "${ENV_PREFIX} && export PYTHONPATH=${REPO_ROOT}/NeuPAN:\${PYTHONPATH:-} && roslaunch ons2_bringup dobot.launch map_file_name:=${MAP_FILE_NAME} navigation_mode:=${NAVIGATION_MODE} global_planner:=${GLOBAL_PLANNER} local_planner:=${LOCAL_PLANNER} neupan_config:=${NEUPAN_CONFIG} $(printf '%q' "${NEUPAN_PREFIX_ARG}") ${BRINGUP_ARGS} 2>&1 | tee ${LOG_DIR}/dobot.log" C-m
ons2_log "w3 starting Dobot navigation stack (global_planner=${GLOBAL_PLANNER}, local_planner=${LOCAL_PLANNER}, mode=${LOCAL_PLANNER_MODE})..."
sleep 15
ons2_wait_for "rosnode list" '/dobot_vel_bridge$' 30 "Dobot chassis bridge"
ons2_wait_for "rostopic list" '/scan$' 30 "pc_to_scan / scan filter"
ons2_wait_for "rostopic list" '/scan_360$' 30 "sliding_map"
if [[ "${SKIP_RELOC}" -eq 0 || "${MOCK_ODIN}" -eq 1 ]]; then
  python3 "${REPO_ROOT}/ros_ws/scripts/launch_gate.py" --wait-tf map base_link --timeout 30 \
    || ons2_fail "map->base_link TF unavailable"
  ons2_log "  [OK] map->base_link TF"
fi

python3 "${REPO_ROOT}/ros_ws/src/dobot_ros_control/scripts/check_o1l_inputs.py" \
  --prefix "${O1L_TOPIC_PREFIX}" --timeout 30 \
  || ons2_fail "navigation input gate failed"
ons2_log "  [OK] navigation input gate"
if [[ "${LOCAL_PLANNER}" = "neupan" ]]; then
  ons2_wait_for "rostopic list" '/neupan/nav_path/status$' 60 "NeuPAN action status"
  MISSION_WAIT="until rostopic list 2>/dev/null | grep -qx '/neupan/nav_path/status'; do sleep 1; done; "
else
  MISSION_WAIT=""
fi

tmux new-window -t "${SESSION}" -n mission 'bash --norc'
tmux send-keys -t "${SESSION}:mission" \
  "${ENV_PREFIX} && ${MISSION_WAIT}roslaunch mission_manager mission_manager.launch enable_atomic_move:=true enable_alignment:=true route_segment_enable:=true route_segment_length_m:=10.0 route_segment_overlap_m:=3.0 navigation_mode:=${NAVIGATION_MODE} periodic_replan_sec:=${PERIODIC_REPLAN_SEC} local_planner:=${LOCAL_PLANNER} local_planner_mode:=${LOCAL_PLANNER_MODE} ${MISSION_IFACE_ARGS} 2>&1 | tee ${LOG_DIR}/mission.log" C-m
ons2_log "w4 starting mission_manager..."
sleep "${GAP_SEC}"

ons2_start_window "${SESSION}" waypoint "${ENV_PREFIX} && python3 ${REPO_ROOT}/ros_ws/scripts/waypoint_goal_bridge.py 2>&1 | tee ${LOG_DIR}/waypoint.log"
ons2_start_window "${SESSION}" rosbridge "${ENV_PREFIX} && roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 max_message_size:=100000000 2>&1 | tee ${LOG_DIR}/rosbridge.log"
ons2_start_window "${SESSION}" foxglove "${ENV_PREFIX} && roslaunch foxglove_bridge foxglove_bridge.launch port:=8765 send_buffer_limit:=200000000 2>&1 | tee ${LOG_DIR}/foxglove.log"
ons2_start_window "${SESSION}" health "${ENV_PREFIX} && bash ${SCRIPT_DIR}/dobot_health_check.sh --watch 2>&1 | tee ${LOG_DIR}/health.log"
ons2_start_window "${SESSION}" control "${ENV_PREFIX} && export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-} && /usr/bin/python3 ${REPO_ROOT}/ros_ws/scripts/test_nav_control_rospy.py --allow-motion"

ons2_log "=================================================="
ons2_log "Dobot Lumo stack started"
ons2_log "  Local planner : ${LOCAL_PLANNER}"
ons2_log "  Navigation    : ${NAVIGATION_MODE}"
ons2_log "  Map           : ${MAP_FILE_NAME}"
ons2_log "  Attach        : tmux attach -t ${SESSION}"
ons2_log "  Stop          : bash ${SCRIPT_DIR}/ons2_dobot_lumo_stop.sh"
ons2_log "=================================================="
