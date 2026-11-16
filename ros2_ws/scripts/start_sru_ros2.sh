#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# SRU local planner startup (ROS2 / Jazzy), called when local_planner=sru.
#
# SRU is an end-to-end visual navigation backend (core in <repo>/sru):
# depth + odom + goal -> /sru/cmd_vel (5 Hz, body frame). No global planning,
# no action feedback. This script sources the overlays, checks dependencies
# and the depth input, then launches the node (static TF off; output reaches
# the chassis only through the gated sru_cmd_vel_relay.py, started separately).
#
# Depth images come from pcd2depth_ros2_node inside the odin driver launch
# (control_command.yaml, senddepth: 1) -- no extra depth node needed.
#
# Python isolation (required): ~/.local holds numpy 2.x needed by other
# stacks, but Jazzy's cv2/cv_bridge are built against numpy 1.x -- loading
# user-site breaks import cv2. Hence PYTHONNOUSERSITE=1, with onnxruntime
# injected from a venv (see sru/README.md).
#
# Env overrides (defaults):
#   SRU_WS_ROS2     external SRU workspace (in-repo overlay preferred)
#   SRU_VENV        onnxruntime venv (<SRU_WS_ROS2>/venv)
#   SRU_ROS2_PKG / SRU_ROS2_LAUNCH  (sru_nav / sru_nav.launch.py)
#   SRU_MODELS_DIR  (<repo>/sru/models, fallback <ws>/Deployment/models)
#   SRU_VAE_MODEL / SRU_POLICY_MODEL  (vae_encoder.onnx / nav_policy.onnx)
#   SRU_PARAMS_FILE (robots/$ONS2_ROBOT/sru/config.yaml, then package default)
#   SRU_LOG_DIR     (${GO2_LOG_DIR:-$HOME/sru_logs})
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # base_inspection/

# ---- 0) Environment: ROS2 Jazzy + in-repo overlay -------------------------
# ROS setup.bash references unset variables; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

SRU_ROS2_PKG="${SRU_ROS2_PKG:-sru_nav}"
SRU_ROS2_LAUNCH="${SRU_ROS2_LAUNCH:-sru_nav.launch.py}"
LOG_DIR="${SRU_LOG_DIR:-${GO2_LOG_DIR:-${HOME}/sru_logs}}"
mkdir -p "${LOG_DIR}"

# ---- 1) SRU workspace: in-repo overlay preferred, external fallback -------
# The in-repo package (ros2_ws/src/sru_nav) is already on the overlay from §0.
# External deployments may use install/ or install_ros2/; probe both.
SRU_SETUP_FILE=""
if [ -z "${SRU_WS_ROS2:-}" ]; then
  if ros2 pkg prefix "${SRU_ROS2_PKG}" >/dev/null 2>&1; then
    SRU_WS_ROS2="$(cd "${REPO_ROOT}/ros2_ws" && pwd)"
    SRU_SETUP_FILE="${SRU_WS_ROS2}/install/setup.bash"
    echo "[start_sru_ros2] SRU package from the in-repo overlay: ${SRU_SETUP_FILE}" >&2
  fi
fi
if [ -z "${SRU_WS_ROS2:-}" ]; then
  for cand in "${HOME}/code/sru_ros2" "${HOME}/sru_nav_ros2" "${HOME}/sru_nav/ros2_ws" "${HOME}/sru_nav"; do
    setup_file="$(ls -d "${cand}"/install/setup.bash "${cand}"/install_ros2/setup.bash 2>/dev/null | head -1 || true)"
    if [ -n "${setup_file}" ]; then
      SRU_WS_ROS2="${cand}"
      SRU_SETUP_FILE="${setup_file}"
      break
    fi
  done
fi
if [ -z "${SRU_WS_ROS2:-}" ]; then
  echo "ERROR: SRU ROS2 workspace not found (probed ~/code/sru_ros2, ~/sru_nav_ros2, ~/sru_nav/ros2_ws, ~/sru_nav)" >&2
  echo "       Deploy the SRU ROS2 package first, or set SRU_WS_ROS2=/path/to/ws" >&2
  exit 1
fi
if [ -z "${SRU_SETUP_FILE}" ]; then
  # ls returns non-zero on missing paths; || true keeps set -e + pipefail safe
  SRU_SETUP_FILE="$(ls -d "${SRU_WS_ROS2}"/install/setup.bash "${SRU_WS_ROS2}"/install_ros2/setup.bash 2>/dev/null | head -1 || true)"
fi
if [ -z "${SRU_SETUP_FILE}" ]; then
  echo "ERROR: no install/setup.bash or install_ros2/setup.bash under ${SRU_WS_ROS2}" >&2
  exit 1
fi
# shellcheck disable=SC1090
# colcon's setup.bash also references unset variables (COLCON_TRACE)
set +u
source "${SRU_SETUP_FILE}"
set -u
echo "[start_sru_ros2] workspace: ${SRU_WS_ROS2} (${SRU_SETUP_FILE})" >&2

if ! ros2 pkg prefix "${SRU_ROS2_PKG}" >/dev/null 2>&1; then
  echo "ERROR: package ${SRU_ROS2_PKG} not found in the workspace (set SRU_ROS2_PKG=...)" >&2
  exit 1
fi

# ---- 2) Python: block user-site, inject the venv's onnxruntime ------------
# Without PYTHONNOUSERSITE, ~/.local's numpy 2.x breaks Jazzy's cv2 import.
export PYTHONNOUSERSITE=1
SRU_VENV="${SRU_VENV:-${SRU_WS_ROS2}/venv}"
if [ -d "${SRU_VENV}/lib" ]; then
  VENV_SP="$(ls -d "${SRU_VENV}/lib"/python3*/site-packages 2>/dev/null | head -1)"
  if [ -n "${VENV_SP}" ]; then
    export PYTHONPATH="${VENV_SP}${PYTHONPATH:+:${PYTHONPATH}}"
    echo "[start_sru_ros2] venv injected: ${VENV_SP}" >&2
  fi
fi
if ! python3 -c "import onnxruntime, cv2, numpy" >/dev/null 2>&1; then
  echo "WARN: python3 cannot import onnxruntime/cv2/numpy together; create the venv:" >&2
  echo "      python3 -m venv --system-site-packages ${SRU_VENV} &&" >&2
  echo "      ${SRU_VENV}/bin/pip install 'numpy<2' onnxruntime" >&2
fi

# ---- 3) Depth input (pcd2depth_ros2_node inside the odin driver launch) ---
if ! timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "/odin1/depth_img_competetion"; then
  echo "WARN: /odin1/depth_img_competetion not found; check the odin driver and" >&2
  echo "      senddepth: 1 in config/control_command.yaml" >&2
fi

# ---- 4) Launch -------------------------------------------------------------
if ros2 node list 2>/dev/null | grep -Fxq "/sru_nav_node"; then
  echo "SRU node already running; stop it first" >&2
  exit 3
fi

SRU_LOG="${LOG_DIR}/sru_nav_ros2_$(date +%Y%m%d_%H%M%S).log"
echo "[start_sru_ros2] launching ${SRU_ROS2_PKG}/${SRU_ROS2_LAUNCH}, log: ${SRU_LOG}" >&2
echo "[start_sru_ros2] note: the cmd_vel gate relay is started separately (sru_cmd_vel_relay.py)" >&2

# ---- Models and params: explicit absolute paths (no auto-discovery) -------
# Models are not in git: prefer <repo>/sru/models, fall back to the external
# deployment directory.
if [ -z "${SRU_MODELS_DIR:-}" ]; then
  if [ -d "${REPO_ROOT}/sru/models" ]; then
    SRU_MODELS_DIR="${REPO_ROOT}/sru/models"
  else
    SRU_MODELS_DIR="${SRU_WS_ROS2}/Deployment/models"
  fi
fi
SRU_VAE_MODEL="${SRU_VAE_MODEL:-${SRU_MODELS_DIR}/vae_encoder.onnx}"
# SRU_POLICY_MODEL: empty = default; bare filename = under SRU_MODELS_DIR;
# anything with a path separator = used as-is.
case "${SRU_POLICY_MODEL:-}" in
  */*) : ;;
  '')  SRU_POLICY_MODEL="${SRU_MODELS_DIR}/nav_policy.onnx" ;;
  *)   SRU_POLICY_MODEL="${SRU_MODELS_DIR}/${SRU_POLICY_MODEL}" ;;
esac
if [ -z "${SRU_PARAMS_FILE:-}" ]; then
  # Per-robot override first (same file bringup passes via robot.yaml config:)
  ROBOT_CFG=""
  if [ -n "${ONS2_ROBOT:-}" ] && [ -f "${REPO_ROOT}/robots/${ONS2_ROBOT}/sru/config.yaml" ]; then
    ROBOT_CFG="${REPO_ROOT}/robots/${ONS2_ROBOT}/sru/config.yaml"
  fi
  for cand in ${ROBOT_CFG:+"${ROBOT_CFG}"} \
              "${REPO_ROOT}/ros2_ws/src/sru_nav/config/sru_nav_ros2.yaml" \
              "${SRU_WS_ROS2}/Deployment_ros2/config/sru_nav_ros2.yaml"; do
    if [ -f "${cand}" ]; then SRU_PARAMS_FILE="${cand}"; break; fi
  done
  SRU_PARAMS_FILE="${SRU_PARAMS_FILE:-${REPO_ROOT}/ros2_ws/src/sru_nav/config/sru_nav_ros2.yaml}"
fi
for f in "${SRU_VAE_MODEL}" "${SRU_POLICY_MODEL}" "${SRU_PARAMS_FILE}"; do
  if [ ! -f "${f}" ]; then
    echo "ERROR: missing required file: ${f}" >&2
    echo "       (models are not in git; copy from an existing deployment, or set SRU_MODELS_DIR/SRU_PARAMS_FILE)" >&2
    exit 1
  fi
done
echo "[start_sru_ros2] models: vae=$(basename "${SRU_VAE_MODEL}") policy=$(basename "${SRU_POLICY_MODEL}") | params: $(basename "${SRU_PARAMS_FILE}")" >&2

ros2 launch "${SRU_ROS2_PKG}" "${SRU_ROS2_LAUNCH}" \
  params_file:="${SRU_PARAMS_FILE}" \
  launch_static_tf:=false \
  vae_model_path:="${SRU_VAE_MODEL}" \
  policy_model_path:="${SRU_POLICY_MODEL}" \
  "$@" 2>&1 | tee -a "${SRU_LOG}"
