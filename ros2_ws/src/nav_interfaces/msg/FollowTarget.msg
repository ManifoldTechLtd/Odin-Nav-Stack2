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
# Dynamic obstacle masking circle around a tracked follow target.
# Published by perception (<= 2Hz) alongside dynamic_goal; NeuPAN excludes points
# inside this circle from obstacle avoidance and suspends stuck_escape while valid=true.
std_msgs/Header header          # frame_id is odom; stamp is the source image timestamp
float64 x                       # Target center x in header.frame_id
float64 y                       # Target center y in header.frame_id
float32 radius                  # Masking circle radius (m), default ~0.4
bool valid                      # false = target lost, stop obstacle masking immediately
