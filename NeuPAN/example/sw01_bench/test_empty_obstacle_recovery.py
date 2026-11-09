"""Real optimizer regression for stale collision recovery when obstacle points clear."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from neupan import neupan


@pytest.mark.parametrize("empty_points", [None, np.empty((2, 0))], ids=["none", "zero_columns"])
def test_empty_observation_clears_collision_and_new_obstacle_still_stops(empty_points):
    torch.set_num_threads(1)
    planner = neupan.init_from_yaml(
        str(ROOT.parent / "robots/sw01/neupan/planner.yaml"))
    planner.set_initial_path([
        np.array([[x], [0.0], [0.0], [1.0]]) for x in np.arange(0.0, 5.01, 0.1)
    ])
    state = np.zeros((3, 1))
    collision = np.array([[0.48], [0.0]])
    action, info = planner(state, collision)
    assert info["stop"]
    np.testing.assert_array_equal(action, 0.0)

    for _ in range(3):
        action, info = planner(state, empty_points)
        assert not info["stop"], "A fresh empty observation retained the previous collision"
        assert math.isinf(float(planner.min_distance))
        assert float(action[0, 0]) > 0.05
        assert planner.pan.dune_points is None
        assert planner.pan.nrmp_points is None

    action, info = planner(state, collision)
    assert info["stop"], "Recovery must not disable checking subsequent obstacles"
    np.testing.assert_array_equal(action, 0.0)
