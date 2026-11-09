#!/usr/bin/env python3
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

"""Unit tests for the path-block grace state machine (`note_path_block`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from neupan.util.stuck_escape import StuckEscapePolicy

BASE_CFG = {
    "enable": False,
    "path_block_grace_enable": True,
    "path_block_dist_m": 0.45,
    "path_block_frames": 5,
    "path_block_clear_frames": 10,
}


def new_policy(**over):
    return StuckEscapePolicy(dict(BASE_CFG, **over))


def feed(p, seq):
    """Feed a sequence of boolean blocked states and return the per-frame active flags."""
    return [p.note_path_block(b) for b in seq]


fails = 0

# T1: disabled switch never activates
p = new_policy(path_block_grace_enable=False)
r = feed(p, [True] * 20)
assert not any(r), "T1 failed: activated when enable=false"
print("T1 enable=false stays inactive PASS")

# T2: activates on 5 consecutive blocked frames (not 4)
p = new_policy()
r = feed(p, [True] * 4)
assert not any(r), "T2 failed: activated on 4 frames"
r += feed(p, [True])
assert r[-1] is True and p.pb_active, "T2 failed: did not activate on 5th frame"
print("T2 activates on 5 consecutive frames PASS")

# T3: interruption resets counter (3 + 1 false + 3 does not activate)
p = new_policy()
r = feed(p, [True] * 3 + [False] + [True] * 3)
assert not p.pb_active, "T3 failed: counter not reset on interruption"
print("T3 interruption resets counter PASS")

# T4: deactivates after 10 consecutive clear frames (holds on 9)
p = new_policy()
feed(p, [True] * 5)
assert p.pb_active
r = feed(p, [False] * 9)
assert p.pb_active, "T4 failed: deactivated early on 9 frames"
r += feed(p, [False])
assert not p.pb_active, "T4 failed: did not deactivate on 10th frame"
print("T4 deactivates after 10 clear frames PASS")

# T5: can re-activate after deactivation
p = new_policy()
feed(p, [True] * 5 + [False] * 10)
assert not p.pb_active
r = feed(p, [True] * 5)
assert r[-1] is True, "T5 failed: could not re-activate"
print("T5 re-activates after deactivation PASS")

# T6: shares grace scale factors with escape grace
p = new_policy()
assert abs(p.grace_q_s_scale - 0.5) < 1e-9 and abs(p.grace_q_theta_scale - 0.25) < 1e-9
print("T6 shares escape grace scales (q_s*0.5 / q_theta*0.25) PASS")

if __name__ == "__main__":
    print("\ntest_path_block_grace: 6/6 PASS")
