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
# Health status of a single node (element of SystemHealth.msg).
#
# status enum (aligned with scripts/system_health_monitor.py):
#   OK           Healthy: fresh valid messages within threshold
#   NO_DATA      Never received data (node not started)
#   STALE        Offline: no new messages within threshold
#   CONTENT_BAD  Messages received but content invalid
#   SERVICE_DOWN Service unreachable (map_planner / atomic_move)
#   TF_LOST      Relocalization lost: map -> base_link lookup failed
#   UNKNOWN      Not yet evaluated (initial startup)
string name      # Node name, e.g. "odin", "neupan", "bridge"
string status    # Enum value above
string detail    # Human-readable detail
string cause     # Probable cause
string action    # Recommended recovery action
