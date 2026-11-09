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

"""Diagnostic script for wide_corridor bypass waypoint tracking."""
import sys, os, tempfile
_HERE = os.path.dirname(os.path.abspath(__file__))
_NEUPAN_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, _NEUPAN_ROOT)
import yaml
from neupan import neupan
import irsim
import numpy as np

cfg = yaml.safe_load(open(os.path.join(_HERE, "wide_corridor", "planner.yaml")))
cfg["ipath"]["waypoints"] = [[5, -0.4, 0], [14, 0, 0]]
import os as _os
_ck = os.path.join(_NEUPAN_ROOT, "example", "dune_train", "model", "sw01", "model_5000.pth")
if not _os.path.exists(_ck):
    _ck = _ck.replace("model_5000.pth", "model_500.pth")
cfg["pan"]["dune_checkpoint"] = _ck
tf = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
yaml.safe_dump(cfg, tf); tf.close()

env = irsim.make(os.path.join(_HERE, "wide_corridor", "env.yaml"),
                 save_ani=False, full=False, display=False)
planner = neupan.init_from_yaml(tf.name)

for k in range(600):
    st = env.get_robot_state(); scan = env.get_lidar_scan()
    pts = planner.scan_to_point(st, scan)
    action, info = planner(st, pts)
    if k % 20 == 0 or info["arrive"] or info["stop"]:
        x, y, th = st.flatten()[:3]; v, w = action.flatten()
        try:
            md = planner.min_distance
            md = float(md.detach().cpu()) if hasattr(md, "detach") else float(md)
        except Exception:
            md = float("nan")
        print(f"step={k:3d} pos=({x:5.2f},{y:+.3f},{th:+.2f}) cmd=(v={v:+.3f},w={w:+.3f}) min_dist={md:.3f} arr={info['arrive']} stop={info['stop']}")
    if info["arrive"] or info["stop"]:
        break
    env.step(action)
env.end()
os.unlink(tf.name)
print("DONE")
