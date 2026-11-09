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
#
# Sync ROS-agnostic Python core, configs, scripts, and unit tests from
# ros_ws/src/odin_brain to ros2_ws/src/odin_brain and run unit tests on both sides.
# Each workspace keeps its own ROS adapter files:
#   - ros_ws/src/odin_brain/src/odin_brain/{ros1_nav.py, node.py}
#   - ros2_ws/src/odin_brain/src/odin_brain/{ros2_nav.py, node.py}

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS1_PKG="${REPO_ROOT}/ros_ws/src/odin_brain"
ROS2_PKG="${REPO_ROOT}/ros2_ws/src/odin_brain"

echo "[1/4] Verifying core modules have zero ROS dependencies..."
ROS_ONLY_REGEX='ros1_nav\.py|ros2_nav\.py|node\.py|cli\.py|place_tool\.py'
LEAKS="$(grep -rnE 'import (rospy|rclpy|tf2_ros)|from (rospy|rclpy|nav_interfaces)' "${ROS1_PKG}/src/odin_brain/" | grep -vE "${ROS_ONLY_REGEX}" || true)"
if [[ -n "${LEAKS}" ]]; then
  echo "[FAIL] Disallowed ROS imports found in core modules:"
  echo "${LEAKS}"
  exit 1
fi
echo "   [OK] Zero ROS dependency check passed"

echo "[2/4] Syncing core code, config, scripts, and tests to ros2_ws/src/odin_brain..."
mkdir -p "${ROS2_PKG}/src/odin_brain" "${ROS2_PKG}/config" "${ROS2_PKG}/scripts" "${ROS2_PKG}/test"

rsync -a --delete \
  --exclude='ros1_nav.py' \
  --exclude='ros2_nav.py' \
  --exclude='node.py' \
  --exclude='__pycache__' \
  "${ROS1_PKG}/src/odin_brain/" "${ROS2_PKG}/src/odin_brain/"

rsync -a --delete "${ROS1_PKG}/config/" "${ROS2_PKG}/config/"
rsync -a --delete "${ROS1_PKG}/scripts/" "${ROS2_PKG}/scripts/"
rsync -a --delete --exclude='__pycache__' "${ROS1_PKG}/test/" "${ROS2_PKG}/test/"
echo "   [OK] Sync complete"

echo "[3/4] Running unit tests in ros_ws/src/odin_brain..."
PYTHONPATH="${ROS1_PKG}/src" python3 -m unittest discover -s "${ROS1_PKG}/test"

echo "[4/4] Running unit tests in ros2_ws/src/odin_brain..."
PYTHONPATH="${ROS2_PKG}/src" python3 -m unittest discover -s "${ROS2_PKG}/test"

echo "All sync and unit test checks passed."
