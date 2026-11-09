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
# CLI shortcut to control Odin Brain via Hermes Agent.
# Usage:
#   bash scripts/hermes_odin.sh                     # Interactive session
#   bash scripts/hermes_odin.sh "<command>"         # Single command

set -euo pipefail

export PATH="$HOME/.local/bin:$PATH"

# Sync latest plugin files from repository
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLUGIN_SRC="${REPO_ROOT}/ros_ws/src/odin_brain/hermes_plugin/odin-brain"
PLUGIN_DST="${HOME}/.hermes/plugins/odin-brain"

if [[ -d "${PLUGIN_SRC}" ]]; then
  mkdir -p "${PLUGIN_DST}"
  cp -r "${PLUGIN_SRC}/"* "${PLUGIN_DST}/"
fi

if [[ $# -gt 0 ]]; then
  exec hermes --cli -t odin_brain -z "$*"
else
  exec hermes --cli -t odin_brain
fi
