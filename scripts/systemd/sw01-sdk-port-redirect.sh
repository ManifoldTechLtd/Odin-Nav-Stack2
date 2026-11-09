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
# Redirect SW01 SDK UDP replies from port 3601 to 3600 for ASTRALL SDK 1.0.7 compatibility
# with SW01 firmware >= 1.1.7.
#
# Usage:
#   sw01-sdk-port-redirect.sh add      # Idempotently add iptables nat REDIRECT rule
#   sw01-sdk-port-redirect.sh del      # Remove rule
#   sw01-sdk-port-redirect.sh status   # Check current rule

set -euo pipefail

ACTION="${1:-}"
RULE_IF="${SW01_SDK_IFACE:-}"
IF_ARG=()
[ -n "${RULE_IF}" ] && IF_ARG=(-i "${RULE_IF}")
SUFFIX="${RULE_IF:+ (iface=${RULE_IF})}"

case "${ACTION}" in
  add)
    if iptables -t nat -C PREROUTING "${IF_ARG[@]}" -p udp --dport 3601 -j REDIRECT --to-ports 3600 2>/dev/null; then
      echo "[sdk-port] Rule already exists${SUFFIX}"
    else
      iptables -t nat -A PREROUTING "${IF_ARG[@]}" -p udp --dport 3601 -j REDIRECT --to-ports 3600
      echo "[sdk-port] Added UDP 3601->3600 REDIRECT${SUFFIX}"
    fi
    ;;
  del)
    while iptables -t nat -C PREROUTING "${IF_ARG[@]}" -p udp --dport 3601 -j REDIRECT --to-ports 3600 2>/dev/null; do
      iptables -t nat -D PREROUTING "${IF_ARG[@]}" -p udp --dport 3601 -j REDIRECT --to-ports 3600
    done
    echo "[sdk-port] Removed 3601->3600 REDIRECT${SUFFIX}"
    ;;
  status)
    if iptables -t nat -L PREROUTING -n 2>/dev/null | grep -q "dpt:3601"; then
      iptables -t nat -L PREROUTING -n --line-numbers | grep "dpt:3601"
    else
      echo "[sdk-port] No 3601 redirect rule found"
    fi
    ;;
  *)
    echo "Usage: $0 {add|del|status}" >&2
    exit 2
    ;;
esac
