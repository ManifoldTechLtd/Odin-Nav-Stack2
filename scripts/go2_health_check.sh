#!/usr/bin/env bash
# Go2 navigation stack health check.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/lib/ons2_lib.sh"

WATCH=0
[[ "${1:-}" == "--watch" ]] && WATCH=1
ons2_run_ros1_health_check "Go2 Navigation Stack" "${WATCH}" \
  /host_sdk_sample /map_server /pc_to_scan /sliding_map_node /vel_to_sdk /mission_manager
