#!/usr/bin/env bash
# Launch the SRU local planner backend (ROS 1 Noetic).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${GO2_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
set +u
source /opt/ros/noetic/setup.bash
[[ -f "${REPO_ROOT}/ros_ws/devel/setup.bash" ]] && source "${REPO_ROOT}/ros_ws/devel/setup.bash"
set -u

SRU_DIR="${SRU_DIR:-${HOME}/sru_nav}"
SRU_PKG="${SRU_PKG:-sru_nav_go2_ros1}"
SRU_LAUNCH="${SRU_LAUNCH:-sru_nav_go2.launch}"
SRU_ENV="${SRU_ENV:-sru_nav}"
ENABLE_DEPTH_PROJECTOR="${ENABLE_DEPTH_PROJECTOR:-true}"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
mkdir -p "${LOG_DIR}"

if [[ -z "${SRU_WS:-}" ]]; then
  for cand in "${SRU_DIR}/ws" "${SRU_DIR}/catkin_ws" "${SRU_DIR}"; do
    [[ -f "${cand}/devel/setup.bash" ]] && { SRU_WS="${cand}"; break; }
  done
fi
[[ -n "${SRU_WS:-}" && -f "${SRU_WS}/devel/setup.bash" ]] || {
  echo "[FAIL] SRU catkin workspace not found under ${SRU_DIR}" >&2
  exit 1
}
set +u; source "${SRU_WS}/devel/setup.bash"; set -u

for conda_sh in "${HOME}"/{miniconda3,miniforge3,mambaforge,anaconda3}/etc/profile.d/conda.sh; do
  if [[ -f "${conda_sh}" && -d "${conda_sh%/etc/profile.d/conda.sh}/envs/${SRU_ENV}" ]]; then
    # shellcheck disable=SC1090
    source "${conda_sh}" && conda activate "${SRU_ENV}"
    break
  fi
done

export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
for _ffi in /lib/aarch64-linux-gnu/libffi.so.7 /usr/lib/aarch64-linux-gnu/libffi.so.7 \
            /lib/x86_64-linux-gnu/libffi.so.7 /usr/lib/x86_64-linux-gnu/libffi.so.7; do
  [[ -f "${_ffi}" ]] && { export LD_PRELOAD="${_ffi}${LD_PRELOAD:+:${LD_PRELOAD}}"; break; }
done

if [[ "${ENABLE_DEPTH_PROJECTOR}" = "true" ]] && ! rostopic list 2>/dev/null | grep -Fxq "/odin1/depth_img_competetion"; then
  DRIVER_PKG="$(rospack find odin_ros_driver)"
  nohup rosrun odin_ros_driver pcd2depth_node __name:=pcd2depth_sru \
    _register_keys/senddepth:=1 _calib_file_path:="${DRIVER_PKG}/config/calib.yaml" \
    >>"${LOG_DIR}/sru_depth_projector.log" 2>&1 &
fi

SRU_LOG="${LOG_DIR}/sru_nav_$(date +%Y%m%d_%H%M%S).log"
echo "[start_sru] Launching ${SRU_PKG}/${SRU_LAUNCH}, log: ${SRU_LOG}" >&2
roslaunch "${SCRIPT_DIR}/sru_nav_wiring.launch" require_joystick:=false "$@" 2>&1 | tee -a "${SRU_LOG}"
