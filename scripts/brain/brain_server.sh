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
# Start, stop, or query the persistent odin_brain HTTP server on Go2.
# Usage:
#   bash scripts/brain_server.sh start
#   bash scripts/brain_server.sh start --dry-run
#   bash scripts/brain_server.sh stop
#   bash scripts/brain_server.sh status
#   bash scripts/brain_server.sh restart

set -uo pipefail

GO2_ROOT="${GO2_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS="$GO2_ROOT/ros_ws"
LOG_DIR="$HOME/go2_logs"
LOG="$LOG_DIR/brain_server.log"
PID_FILE="$HOME/.cache/odin_brain_server.pid"
PORT="${ODIN_BRAIN_SERVER_PORT:-18090}"

mkdir -p "$LOG_DIR" "$(dirname "$PID_FILE")"

go2_server_nounset=0
case "$-" in *u*) go2_server_nounset=1; set +u;; esac
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
[ "$go2_server_nounset" = "1" ] && set -u
export PYTHONUNBUFFERED=1

is_running() {
  [ -f "$PID_FILE" ] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start() {
  if is_running; then
    echo "[brain_server] Already running (PID $(cat "$PID_FILE"))"
    return 0
  fi
  if [[ " $* " != *" --mock "* ]]; then
    if [ ! -d "$WS/devel" ]; then
      echo "[brain_server] Missing build directory $WS/devel (run: cd $WS && catkin_make)" >&2
      return 1
    fi
    export CMAKE_PREFIX_PATH="$WS/devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$WS/devel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export ROS_PACKAGE_PATH="$WS/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
  fi
  echo "[brain_server] Starting (port $PORT, log $LOG)"
  nohup python3 "$GO2_ROOT/ros_ws/src/odin_brain/scripts/brain_server.py" \
        --host 0.0.0.0 --port "$PORT" "$@" >> "$LOG" 2>&1 &
  echo $! > "$PID_FILE"
  for _ in $(seq 1 30); do
    sleep 1
    if curl -s --max-time 2 "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
      echo "[brain_server] [OK] Ready (PID $(cat "$PID_FILE"))"
      curl -s --max-time 3 "http://127.0.0.1:$PORT/health" | head -c 300
      echo
      return 0
    fi
  done
  echo "[brain_server] [WARN] Not ready within 30s; check log: tail -40 $LOG" >&2
  return 1
}

stop() {
  if is_running; then
    local pid
    pid="$(cat "$PID_FILE")"
    echo "[brain_server] Stopping PID $pid"
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 10); do
      sleep 0.5
      kill -0 "$pid" 2>/dev/null || break
    done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  else
    echo "[brain_server] Not running"
  fi
  rm -f "$PID_FILE"
}

status() {
  if is_running; then
    echo "[brain_server] Running (PID $(cat "$PID_FILE"), port $PORT)"
    curl -s --max-time 3 "http://127.0.0.1:$PORT/health" || echo "  (health endpoint not responding)"
    echo
  else
    echo "[brain_server] Not running"
  fi
}

cmd="${1:-start}"
shift || true
case "$cmd" in
  start)   start "$@" ;;
  stop)    stop ;;
  restart) stop; sleep 1; start "$@" ;;
  status)  status ;;
  *) echo "Usage: bash $0 {start|stop|restart|status} [--dry-run|--mock|--no-llm|--no-tts]" >&2; exit 2 ;;
esac
