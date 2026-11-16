#!/usr/bin/env bash
# Launch the SRU local planner backend (ROS1 Noetic) from this repository.
#
# SRU is an end-to-end visual navigation backend: depth + odom + goal in,
# /sru/cmd_vel out (control_frequency Hz, body frame). No global planning,
# no action feedback. This script:
#   1) sources Noetic + this repo's ros_ws/devel overlay (package sru_nav)
#   2) optionally activates a conda env providing onnxruntime (SRU_ENV)
#   3) starts the depth projector if the depth topic is absent
#   4) roslaunches sru_nav/sru_nav.launch (gated /sru/* topic chain)
#
# The cmd_vel gate relay (sru_cmd_vel_relay.py) is started separately by the
# orchestration layer; /sru/cmd_vel_enable defaults to false.
#
# Models are NOT bundled with this repository. Resolution order:
#   SRU_VAE_MODEL / SRU_POLICY_MODEL (absolute paths) -> <repo>/sru/models/
#
# Environment overrides:
#   SRU_ENV                 conda env with onnxruntime (default sru_nav; skipped if absent)
#   SRU_MODELS_DIR          model directory (default <repo>/sru/models)
#   SRU_VAE_MODEL / SRU_POLICY_MODEL  explicit onnx paths (default names
#                           vae_encoder.onnx / nav_policy.onnx under SRU_MODELS_DIR)
#   ENABLE_DEPTH_PROJECTOR  start pcd2depth_node when depth topic missing (default true)
#   SRU_LOG_DIR             log directory (default ${GO2_LOG_DIR:-$HOME/sru_logs})
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # base_inspection/

# ---- 0) Environment: Noetic + this repo's catkin overlay --------------------
# ROS setup.bash references unbound variables; relax nounset while sourcing.
set +u
source /opt/ros/noetic/setup.bash
if [[ -f "${REPO_ROOT}/ros_ws/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/ros_ws/devel/setup.bash"
fi
set -u

SRU_PKG="${SRU_PKG:-sru_nav}"
SRU_ENV="${SRU_ENV:-sru_nav}"
ENABLE_DEPTH_PROJECTOR="${ENABLE_DEPTH_PROJECTOR:-true}"
LOG_DIR="${SRU_LOG_DIR:-${GO2_LOG_DIR:-${HOME}/sru_logs}}"
mkdir -p "${LOG_DIR}"

if ! rospack find "${SRU_PKG}" >/dev/null 2>&1; then
  echo "[FAIL] package ${SRU_PKG} not found. Build it first:" >&2
  echo "       cd ${REPO_ROOT}/ros_ws && catkin_make --pkg sru_nav && source devel/setup.bash" >&2
  exit 1
fi

# ---- 1) Python env: optional conda env for onnxruntime ----------------------
for conda_sh in "${HOME}"/{miniconda3,miniforge3,mambaforge,anaconda3}/etc/profile.d/conda.sh; do
  if [[ -f "${conda_sh}" && -d "${conda_sh%/etc/profile.d/conda.sh}/envs/${SRU_ENV}" ]]; then
    # shellcheck disable=SC1090
    source "${conda_sh}" && conda activate "${SRU_ENV}"
    break
  fi
done
export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
if ! python3 -c "import onnxruntime, cv2, numpy" >/dev/null 2>&1; then
  echo "[WARN] python3 cannot import onnxruntime/cv2/numpy together; SRU will fail to start." >&2
  echo "       Install deps: pip install -r ${REPO_ROOT}/sru/requirements.txt (or use SRU_ENV conda env)" >&2
fi

# ---- 2) Models: explicit absolute paths (never rely on auto-discovery) ------
if [[ -z "${SRU_MODELS_DIR:-}" ]]; then
  SRU_MODELS_DIR="${REPO_ROOT}/sru/models"
fi
SRU_VAE_MODEL="${SRU_VAE_MODEL:-${SRU_MODELS_DIR}/vae_encoder.onnx}"
SRU_POLICY_MODEL="${SRU_POLICY_MODEL:-${SRU_MODELS_DIR}/nav_policy.onnx}"
for f in "${SRU_VAE_MODEL}" "${SRU_POLICY_MODEL}"; do
  if [[ ! -f "${f}" ]]; then
    echo "[FAIL] missing model file: ${f}" >&2
    echo "       Models are not in git; copy them from an existing deployment or set" >&2
    echo "       SRU_MODELS_DIR / SRU_VAE_MODEL / SRU_POLICY_MODEL." >&2
    exit 1
  fi
done
export SRU_VAE_MODEL SRU_POLICY_MODEL

# ---- 3) Depth input: start the projector if the topic is absent -------------
if [[ "${ENABLE_DEPTH_PROJECTOR}" = "true" ]] && ! rostopic list 2>/dev/null | grep -Fxq "/odin1/depth_img_competetion"; then
  DRIVER_PKG="$(rospack find odin_ros_driver)"
  nohup rosrun odin_ros_driver pcd2depth_node __name:=pcd2depth_sru \
    _register_keys/senddepth:=1 _calib_file_path:="${DRIVER_PKG}/config/calib.yaml" \
    >>"${LOG_DIR}/sru_depth_projector.log" 2>&1 &
fi

# ---- 4) Launch ---------------------------------------------------------------
if rosnode list 2>/dev/null | grep -Fxq "/sru_nav_node"; then
  echo "[start_sru] SRU node already running; stop it first" >&2
  exit 3
fi
SRU_LOG="${LOG_DIR}/sru_nav_$(date +%Y%m%d_%H%M%S).log"
echo "[start_sru] Launching ${SRU_PKG}/sru_nav.launch, log: ${SRU_LOG}" >&2
echo "[start_sru] models: vae=$(basename "${SRU_VAE_MODEL}") policy=$(basename "${SRU_POLICY_MODEL}")" >&2
roslaunch "${SRU_PKG}" sru_nav.launch "$@" 2>&1 | tee -a "${SRU_LOG}"