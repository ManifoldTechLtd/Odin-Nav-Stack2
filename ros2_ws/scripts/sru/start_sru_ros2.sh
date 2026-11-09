#!/usr/bin/env bash
# Launch the SRU end-to-end local planner backend (ROS 2 Jazzy).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]] && source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

SRU_ROS2_PKG="${SRU_ROS2_PKG:-sru_nav_go2_ros2}"
SRU_ROS2_LAUNCH="${SRU_ROS2_LAUNCH:-sru_nav_go2.launch.py}"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
mkdir -p "${LOG_DIR}"

SRU_SETUP_FILE=""
if [[ -z "${SRU_WS_ROS2:-}" ]]; then
  for cand in "${HOME}/code/sru_ros2" "${HOME}/sru_nav_ros2" "${HOME}/sru_nav/ros2_ws" "${HOME}/sru_nav"; do
    setup_file="$(ls -d "${cand}"/install/setup.bash "${cand}"/install_ros2/setup.bash 2>/dev/null | head -1 || true)"
    [[ -n "${setup_file}" ]] && { SRU_WS_ROS2="${cand}"; SRU_SETUP_FILE="${setup_file}"; break; }
  done
fi
[[ -n "${SRU_WS_ROS2:-}" ]] || { echo "[FAIL] SRU ROS 2 workspace not found; set SRU_WS_ROS2=/path/to/ws" >&2; exit 1; }
[[ -n "${SRU_SETUP_FILE}" ]] || SRU_SETUP_FILE="$(ls -d "${SRU_WS_ROS2}"/install/setup.bash "${SRU_WS_ROS2}"/install_ros2/setup.bash 2>/dev/null | head -1 || true)"
[[ -n "${SRU_SETUP_FILE}" ]] || { echo "[FAIL] setup.bash not found under ${SRU_WS_ROS2}" >&2; exit 1; }

set +u; source "${SRU_SETUP_FILE}"; set -u
echo "[start_sru_ros2] Workspace: ${SRU_WS_ROS2}" >&2

export PYTHONNOUSERSITE=1
SRU_VENV="${SRU_VENV:-${SRU_WS_ROS2}/venv}"
if [[ -d "${SRU_VENV}/lib" ]]; then
  VENV_SP="$(ls -d "${SRU_VENV}/lib"/python3*/site-packages 2>/dev/null | head -1 || true)"
  [[ -n "${VENV_SP}" ]] && export PYTHONPATH="${VENV_SP}${PYTHONPATH:+:${PYTHONPATH}}"
fi

SRU_MODELS_DIR="${SRU_MODELS_DIR:-${SRU_WS_ROS2}/Deployment/models}"
SRU_VAE_MODEL="${SRU_VAE_MODEL:-${SRU_MODELS_DIR}/vae_encoder.onnx}"
case "${SRU_POLICY_MODEL:-}" in
  */*) : ;;
  '')  SRU_POLICY_MODEL="${SRU_MODELS_DIR}/nav_policy_sw01.onnx" ;;
  *)   SRU_POLICY_MODEL="${SRU_MODELS_DIR}/${SRU_POLICY_MODEL}" ;;
esac
SRU_PARAMS_FILE="${SRU_PARAMS_FILE:-${SRU_WS_ROS2}/Deployment_ros2/config/sru_nav_sw01.yaml}"
for f in "${SRU_VAE_MODEL}" "${SRU_POLICY_MODEL}" "${SRU_PARAMS_FILE}"; do
  [[ -f "${f}" ]] || { echo "[FAIL] Required file not found: ${f}" >&2; exit 1; }
done

SRU_GOAL_TOPIC="${SRU_GOAL_TOPIC:-/sru/goal_pose}"
SRU_CMD_VEL_TOPIC="${SRU_CMD_VEL_TOPIC:-/sru/cmd_vel}"
SRU_LOG="${LOG_DIR}/sru_nav_ros2_$(date +%Y%m%d_%H%M%S).log"

ros2 launch "${SRU_ROS2_PKG}" "${SRU_ROS2_LAUNCH}" \
  params_file:="${SRU_PARAMS_FILE}" \
  launch_joy:=false \
  launch_static_tf:=false \
  require_joystick:=false \
  goal_topic:="${SRU_GOAL_TOPIC}" \
  cmd_vel_topic:="${SRU_CMD_VEL_TOPIC}" \
  vae_model_path:="${SRU_VAE_MODEL}" \
  policy_model_path:="${SRU_POLICY_MODEL}" \
  "$@" 2>&1 | tee -a "${SRU_LOG}"
