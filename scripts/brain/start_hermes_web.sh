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
# Launch Hermes Agent web dashboard and Odin Brain voice control services:
#   1. Local FunASR speech recognition service (port 18082)
#   2. Local Edge-TTS speech synthesis service (port 18081)
#   3. Sync and enable ~/.hermes/plugins/odin-brain plugin
#   4. Launch Hermes Web Dashboard (http://127.0.0.1:9119/odin-brain)

set -euo pipefail

export PATH="$HOME/.local/bin:$PATH"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "=== [1/5] Syncing odin-brain Hermes web plugin ==="
mkdir -p "$HOME/.hermes/plugins/odin-brain"
cp -r "${REPO_ROOT}/ros_ws/src/odin_brain/hermes_plugin/odin-brain/"* "$HOME/.hermes/plugins/odin-brain/"
hermes config set tools.tool_search.enabled off >/dev/null 2>&1 || true
hermes plugins enable odin-brain >/dev/null 2>&1 || true
echo "   [OK] Plugin ready: ~/.hermes/plugins/odin-brain"

echo "=== [2/5] Checking LLM proxy service (port 18080) ==="
if ss -tln | grep -q ':18080 '; then
  echo "   [OK] LLM proxy already running (18080)"
else
  nohup python3 "${SCRIPT_DIR}/llm_proxy.py" --port 18080 > /tmp/llm_proxy.log 2>&1 &
  sleep 1
  echo "   [OK] Started LLM proxy (18080)"
fi

echo "=== [3/5] Checking TTS service (port 18081) ==="
if ss -tln | grep -q ':18081 '; then
  echo "   [OK] TTS service already running (18081)"
else
  nohup python3 "${SCRIPT_DIR}/tts_server.py" --port 18081 > /tmp/tts_server.log 2>&1 &
  sleep 1
  echo "   [OK] Started TTS service (18081)"
fi

echo "=== [4/5] Checking local FunASR service (port 18082) ==="
if ss -tln | grep -q ':18082 '; then
  echo "   [OK] Local FunASR service already running (18082)"
else
  ASR_PY="${ASR_PYTHON:-${HOME}/miniconda3/bin/python}"
  [[ -x "${ASR_PY}" ]] || ASR_PY="python3"
  nohup "${ASR_PY}" "${SCRIPT_DIR}/asr_server.py" --port 18082 > /tmp/asr_server.log 2>&1 &
  echo "   Loading FunASR model..."
  for _ in {1..15}; do
    if curl -s http://127.0.0.1:18082/health | grep -q '"ok": true'; then
      break
    fi
    sleep 1
  done
  echo "   [OK] Local FunASR service ready (18082)"
fi

echo "=== [5/5] Starting Hermes Agent web dashboard (port 9119) ==="
if [ "${1:-}" = "--restart" ] && ss -tln 2>/dev/null | grep -q ':9119 '; then
  OLD_PID=$(ss -ltnp 2>/dev/null | grep 9119 | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
  if [ -n "${OLD_PID:-}" ]; then
    echo "   Restarting existing dashboard (PID $OLD_PID)"
    kill -TERM "$OLD_PID" 2>/dev/null || true
    for _ in {1..10}; do sleep 1; ss -tln 2>/dev/null | grep -q ':9119 ' || break; done
  fi
fi

if ss -tln 2>/dev/null | grep -q ':9119 '; then
  echo "   [OK] Hermes Dashboard already running (9119)"
  echo "   (Use 'bash $0 --restart' to reload plugin changes)"
else
  nohup hermes dashboard --skip-build --no-open > /tmp/hermes_dashboard.log 2>&1 &
  for _ in {1..15}; do
    if ss -tln | grep -q ':9119 '; then
      break
    fi
    sleep 1
  done
  echo "   [OK] Hermes Dashboard started (9119)"
fi

echo ""
echo "========================================================================"
echo "Startup complete! Open one of the following URLs in your browser:"
echo "  Odin Brain Voice Console : http://127.0.0.1:9119/odin-brain"
echo "  Hermes Chat              : http://127.0.0.1:9119/chat"
echo "========================================================================"
