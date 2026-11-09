#!/usr/bin/env bash
# Shared helpers for ONS2 robot entry scripts.

ons2_log() { echo -e "[ons2_${ONS2_ROBOT}] $*"; }
ons2_fail() { echo -e "[ons2_${ONS2_ROBOT}][FAIL] $*" >&2; exit 1; }

ons2_parse_common_args() {
  MAP_NAME=""
  NAVIGATION_MODE="replan_periodic"
  PERIODIC_REPLAN_SEC="2.0"
  LOCAL_PLANNER="$(python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --local-planner "${ROBOT_PROFILE}")"
  GLOBAL_PLANNER="$(python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --global-planner "${ROBOT_PROFILE}")"
  MOCK_ODIN=0
  SKIP_RELOC=0
  for arg in "$@"; do
    case "$arg" in
      --map=*) MAP_NAME="${arg#--map=}" ;;
      --navigation-mode=*) NAVIGATION_MODE="${arg#--navigation-mode=}" ;;
      --periodic-replan-sec=*) PERIODIC_REPLAN_SEC="${arg#--periodic-replan-sec=}" ;;
      --local-planner=*) LOCAL_PLANNER="${arg#--local-planner=}" ;;
      --global-planner=*) GLOBAL_PLANNER="${arg#--global-planner=}" ;;
      --neupan-config=*) NEUPAN_CONFIG="${arg#--neupan-config=}" ;;
      --mock-odin) MOCK_ODIN=1 ;;
      --skip-reloc) SKIP_RELOC=1 ;;
      *) ons2_fail "Unknown argument: $arg" ;;
    esac
  done
  LOCAL_PLANNER_MODE="$(python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --local-planner-mode "${ROBOT_PROFILE}" "${LOCAL_PLANNER}")"
  MISSION_IFACE_ARGS="$(python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --mission-args "${ROBOT_PROFILE}" "${LOCAL_PLANNER}" "${GLOBAL_PLANNER}")"
}

ons2_validate_common() {
  case "${NAVIGATION_MODE}" in
    escape|replan_periodic|neupan_only) ;;
    *) ons2_fail "Invalid --navigation-mode=${NAVIGATION_MODE}" ;;
  esac
  if [[ "${LOCAL_PLANNER}" = "neupan" ]]; then
    [[ -f "${NEUPAN_CONFIG}" ]] || ons2_fail "NeuPAN config not found: ${NEUPAN_CONFIG}"
  fi
}

ons2_ros1_bringup_args() {
  local profile="$1"
  [[ -f "${profile}" ]] || ons2_fail "Robot profile not found: ${profile}"
  python3 "${REPO_ROOT}/scripts/lib/robot_profile.py" --ros1-bringup-args "${profile}" "${LOCAL_PLANNER}" "${GLOBAL_PLANNER}" \
    || ons2_fail "Failed to parse bringup args from profile: ${profile}"
}

ons2_source_ros1() {
  [[ -f /opt/ros/noetic/setup.bash ]] || ons2_fail "ROS1 Noetic not found"
  [[ -f "${REPO_ROOT}/ros_ws/devel/setup.bash" ]] || ons2_fail "ROS1 workspace not built"
  set +u
  source /opt/ros/noetic/setup.bash || ons2_fail "failed to source ROS1"
  source "${REPO_ROOT}/ros_ws/devel/setup.bash" || ons2_fail "failed to source ROS1 workspace"
  set -u
}

ons2_source_ros2() {
  [[ -f /opt/ros/jazzy/setup.bash ]] || ons2_fail "ROS2 Jazzy not found"
  [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]] || ons2_fail "ROS2 workspace not built"
  set +u
  source /opt/ros/jazzy/setup.bash || ons2_fail "failed to source ROS2"
  source "${REPO_ROOT}/ros2_ws/install/setup.bash" || ons2_fail "failed to source ROS2 workspace"
  set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
}

ons2_require_tmux() {
  command -v tmux >/dev/null 2>&1 || ons2_fail "tmux is not installed"
}

ons2_new_session() {
  local session="$1" first_window="$2"
  tmux has-session -t "${session}" 2>/dev/null && ons2_fail "tmux session ${session} already exists; stop it first"
  # Force bash --norc so `source .../setup.bash` works regardless of login shell (e.g. zsh) and user rc files.
  tmux new-session -d -s "${session}" -n "${first_window}" 'bash --norc'
  tmux set-option -t "${session}" remain-on-exit on
}

ons2_start_window() {
  tmux new-window -t "$1" -n "$2" 'bash --norc'
  tmux send-keys -t "$1:$2" "$3" C-m
}

ons2_wait_for() {
  local cmd="$1" pat="$2" timeout="$3" desc="$4" t0=${SECONDS}
  while true; do
    if eval "${cmd}" 2>/dev/null | grep -q "${pat}"; then
      ons2_log "  [OK] ${desc}"
      return 0
    fi
    (( SECONDS - t0 >= timeout )) && ons2_fail "${desc} timed out (${timeout}s)"
    sleep 1
  done
}

ons2_wait_window() {
  local session="$1" window="$2" pattern="$3" timeout="$4" desc="$5" proc_pattern="$6" t0=${SECONDS}
  while true; do
    if tmux capture-pane -t "${session}:${window}" -p -S - -J 2>/dev/null | grep -q "${pattern}"; then
      ons2_log "  [OK] ${desc}"
      return 0
    fi
    if [[ -n "${proc_pattern}" ]] && ! pgrep -f "${proc_pattern}" >/dev/null 2>&1; then
      ons2_fail "${desc} failed: process exited"
    fi
    if [[ -n "${timeout}" ]] && (( SECONDS - t0 >= timeout )); then
      tmux capture-pane -t "${session}:${window}" -p -S - 2>/dev/null | tail -12 >&2 || true
      ons2_fail "${desc} timed out (${timeout}s)"
    fi
    sleep 2
  done
}

ons2_prepare_odin_reloc_config() {
  local cfg="$1" reloc_bin="$2"
  RELOC_MAP_SWITCHED=0
  [[ "${MOCK_ODIN}" -eq 1 ]] && return 0
  [[ -f "${cfg}" ]] || ons2_fail "Driver config not found: ${cfg}"
  [[ -f "${cfg}.orig" ]] || cp "${cfg}" "${cfg}.orig"
  sed -i -E "s|^[[:space:]]*use_host_ros_time:[[:space:]]*[0-2].*|  use_host_ros_time: 1|" "${cfg}"
  sed -i -E "s|^([[:space:]]*custom_map_mode:)[[:space:]]*[0-2].*|\1 2|" "${cfg}"
  if [[ -n "${MAP_NAME}" || -f "${reloc_bin}" ]]; then
    [[ -f "${reloc_bin}" ]] || ons2_fail "--map=${MAP_FILE_NAME} missing relocalization map: ${reloc_bin}"
    local old_bin
    old_bin="$(grep -oP '^[[:space:]]*relocalization_map_abs_path:[[:space:]]*"\K[^"]+' "${cfg}" 2>/dev/null | head -1 || true)"
    [[ "${old_bin}" = "${reloc_bin}" ]] || RELOC_MAP_SWITCHED=1
    sed -i "s|^[[:space:]]*relocalization_map_abs_path:.*|  relocalization_map_abs_path: \"${reloc_bin}\"|" "${cfg}"
    grep -Eq "^[[:space:]]*relocalization_map_abs_path:[[:space:]]*\"${reloc_bin}\"" "${cfg}" \
      || ons2_fail "Failed to update relocalization_map_abs_path in ${cfg}"
  fi
}

ons2_stop_ros1_stack() {
  local tag="$1" session="$2" sigint_pat="$3"
  shift 3
  local kill_pats=("$@" "ons2_safety_gate_ros1.py") n pat regex left
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
  set +u; source /opt/ros/noetic/setup.bash 2>/dev/null || true; set -u

  if [[ -n "${sigint_pat}" ]]; then
    pkill -INT -f "${sigint_pat}" 2>/dev/null || true
    sleep 2
  fi
  tmux kill-session -t "${session}" 2>/dev/null || true
  for n in $(timeout 6 rosnode list 2>/dev/null | grep -v rosout); do
    rosnode kill "$n" >/dev/null 2>&1 || true
  done
  sleep 2
  for pat in "${kill_pats[@]}"; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 1
  echo y | timeout 15 rosnode cleanup >/dev/null 2>&1 || true

  regex="$(IFS='|'; echo "${kill_pats[*]}")"
  left="$(pgrep -a -f "${regex}" 2>/dev/null | grep -vE "${tag}|$$" || true)"
  if [[ -z "${left}" ]]; then
    echo "[${tag}] [OK] Stack stopped and cleaned up"
  else
    echo "[${tag}] [WARN] Remaining processes:"; echo "${left}" | head -6
  fi
}

ons2_run_ros1_health_check() {
  local title="$1" watch="${2:-0}"
  shift 2
  local req_nodes=("$@")
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
  set +u
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  [[ -f "${REPO_ROOT}/ros_ws/devel/setup.bash" ]] && source "${REPO_ROOT}/ros_ws/devel/setup.bash" 2>/dev/null || true
  set -u
  export PYTHONPATH="/opt/ros/noetic/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"

  declare -A topic_hz=( [/odin1/odometry]=5 [/scan]=3 [/scan_360]=3 )

  _check_once() {
    echo "============ ${title} Health @ $(date '+%H:%M:%S') ============"
    timeout 5 rostopic list >/dev/null 2>&1 || { echo "  [FAIL] roscore offline"; return; }

    local planner gp lp_mode nodes missing=() n t min hz
    planner=$(rosparam get /mission_manager/local_planner 2>/dev/null || rosparam get /local_planner 2>/dev/null || echo neupan)
    gp=$(rosparam get /global_planner 2>/dev/null || echo map_planner)
    lp_mode=$(rosparam get /mission_manager/local_planner_mode 2>/dev/null || echo "")
    [[ -z "$lp_mode" ]] && { [[ "$planner" == "sru" || "$planner" == "goal_pose" ]] && lp_mode="goal_pose" || lp_mode="action"; }

    nodes=$(timeout 6 rosnode list 2>/dev/null)
    for n in "${req_nodes[@]}"; do echo "$nodes" | grep -qx "$n" || missing+=("$n"); done
    [[ "$gp" == "map_planner" ]] && { echo "$nodes" | grep -qx "/map_planner" || missing+=("/map_planner"); }
    [[ "$planner" == "neupan" ]] && { echo "$nodes" | grep -Eq '^/(neupan_node_|neupan_action_compat)' || missing+=("/neupan_action_compat"); }
    (( ${#missing[@]} == 0 )) && echo "  [OK] Nodes online (global=${gp}, local=${planner}, mode=${lp_mode})" \
                              || echo "  [FAIL] Missing nodes: ${missing[*]}"

    for t in "${!topic_hz[@]}"; do
      min=${topic_hz[$t]}
      hz=$(timeout 4 rostopic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
      [[ -z "$hz" ]] && echo "  [FAIL] $t no data" || { hz=${hz%%.*}; (( hz >= min )) && echo "  [OK] $t ${hz}Hz" || echo "  [WARN] $t ${hz}Hz (<${min}Hz)"; }
    done

    python3 "${REPO_ROOT}/ros_ws/scripts/launch_gate.py" --wait-tf map base_link --timeout 3 >/dev/null 2>&1 \
      && echo "  [OK] TF map->base_link" || echo "  [FAIL] TF map->base_link missing"

    if [[ "$lp_mode" == "action" ]]; then
      local act_name
      act_name=$(rosparam get /mission_manager/nav_path_action 2>/dev/null || echo /neupan/nav_path)
      rostopic list 2>/dev/null | grep -qx "${act_name%/}/status" \
        && echo "  [OK] Action server ${act_name} (${planner})" \
        || echo "  [FAIL] Action server ${act_name} unreachable (${planner})"
    fi
    echo "========================================================"
  }

  if (( watch )); then
    local gate_pid=""
    python3 "${REPO_ROOT}/ros_ws/scripts/ons2_safety_gate_ros1.py" >/dev/null 2>&1 &
    gate_pid=$!
    trap '[[ -n "${gate_pid:-}" ]] && kill "${gate_pid}" 2>/dev/null || true' EXIT INT TERM
    while true; do
      clear
      _check_once
      if [[ -n "${gate_pid:-}" ]] && ! kill -0 "${gate_pid}" 2>/dev/null; then
        python3 "${REPO_ROOT}/ros_ws/scripts/ons2_safety_gate_ros1.py" >/dev/null 2>&1 &
        gate_pid=$!
      fi
      sleep 5
    done
  else
    _check_once
  fi
}
