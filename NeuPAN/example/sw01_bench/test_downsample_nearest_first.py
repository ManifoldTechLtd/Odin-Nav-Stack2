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

"""Obstacle downsampling must keep the points closest to the robot."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from neupan.util import downsample_nearest_first  # noqa: E402


def _scene():
    far = np.stack([np.linspace(3.0, 4.0, 200), np.full(200, 2.0)])
    near = np.array([[0.6, 0.7], [0.05, -0.05]])
    return np.hstack([far[:, :100], near, far[:, 100:]])


def test_keeps_near_points_when_far_points_dominate():
    idx = downsample_nearest_first(_scene(), 50, (0.0, 0.0))
    assert len(idx) == 50 and len(set(idx.tolist())) == 50
    assert {100, 101} <= set(idx.tolist())


def test_keeps_far_coverage():
    idx = downsample_nearest_first(_scene(), 50, (0.0, 0.0))
    assert int(np.sum(idx > 101)) >= 10


def test_returns_all_points_under_the_limit():
    pts = _scene()[:, :30]
    np.testing.assert_array_equal(downsample_nearest_first(pts, 50, (0.0, 0.0)), np.arange(30))


def test_accepts_torch_tensors():
    torch = pytest.importorskip("torch")
    idx = downsample_nearest_first(torch.tensor(_scene()), 50, torch.tensor([0.0, 0.0]))
    assert {100, 101} <= set(idx.tolist())


def test_pan_point_flow_keeps_the_nearest_obstacle():
    torch = pytest.importorskip("torch")
    from neupan import neupan

    planner = neupan.init_from_yaml(str(ROOT.parent / "robots/sw01/neupan/planner.yaml"))
    pan = planner.pan
    nom_s = torch.zeros((3, pan.T + 1))
    obs = torch.tensor(_scene(), dtype=torch.float32)
    _, _, obs_list = pan.generate_point_flow(nom_s, obs)
    kept = obs_list[0]
    assert kept.shape[1] == pan.dune_max_num
    dist = torch.linalg.norm(kept, dim=0)
    assert float(dist.min()) < 0.8, "nearest obstacle point was dropped by downsampling"
