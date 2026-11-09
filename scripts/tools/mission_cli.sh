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

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_ROOT="${GO2_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
set +u
source /opt/ros/noetic/setup.bash
[[ -f "${GO2_ROOT}/ros_ws/devel/setup.bash" ]] && source "${GO2_ROOT}/ros_ws/devel/setup.bash"
set -u

usage() {
  cat <<'EOF'
Usage:
  mission_cli.sh status
  mission_cli.sh services
  mission_cli.sh start <x> <y> [yaw] [dwell_sec] [name]
  mission_cli.sh pause
  mission_cli.sh resume
  mission_cli.sh return_home
  mission_cli.sh cancel
  mission_cli.sh reset

Motion-affecting commands require:
  GO2_MOTION_CONFIRM=I_UNDERSTAND_GO2_MAY_MOVE
EOF
}

require_motion_confirmation() {
  if [[ "${GO2_MOTION_CONFIRM:-}" != "I_UNDERSTAND_GO2_MAY_MOVE" ]]; then
    echo "refusing motion command: set GO2_MOTION_CONFIRM=I_UNDERSTAND_GO2_MAY_MOVE" >&2
    exit 3
  fi
}

command_name="${1:-}"
case "${command_name}" in
  status)
    exec rostopic echo -n1 /waypoint_manager/status
    ;;
  services)
    exec rosservice list | grep '^/waypoint_manager/'
    ;;
  start)
    require_motion_confirmation
    if [[ $# -lt 3 || $# -gt 6 ]]; then
      usage >&2
      exit 2
    fi
    x="$2"
    y="$3"
    yaw="${4:-0.0}"
    dwell_sec="${5:-0.0}"
    name="${6:-debug_waypoint}"
    exec rosservice call /waypoint_manager/start_mission \
      "{waypoints: [{x: ${x}, y: ${y}, yaw: ${yaw}, dwell_sec: ${dwell_sec}, name: '${name}'}], loop: false, temporary: false}"
    ;;
  pause|resume|return_home|cancel|reset)
    require_motion_confirmation
    exec rosservice call "/waypoint_manager/${command_name}" "{}"
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
