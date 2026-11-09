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

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'low_flicker_filter.py'
GROUND = np.array([[0.6, 0.0, -0.6]], dtype=np.float32)
LOW = np.array([[0.6, 0.0, -0.5]], dtype=np.float32)


def make_core(**options):
    assert SCRIPT.exists(), 'A-class low flicker kernel is not implemented'
    spec = importlib.util.spec_from_file_location('low_flicker_filter', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    params = dict(enabled=True, plane_normal=[0.0, 0.0, 1.0],
                  plane_offset_m=0.6)
    params.update(options)
    return module.LowFlickerFilter(**params)


def test_ground_to_single_flicker_then_ground_has_no_reverse_deletion():
    core = make_core()
    assert core.process(GROUND, 100_000_000)[0].tolist() == [True]
    keep, stats = core.process(LOW, 200_000_000)
    assert keep.tolist() == [False]
    assert stats['candidate_points'] == stats['removed_points'] == 1
    keep, stats = core.process(GROUND, 300_000_000)
    assert keep.tolist() == [True]
    assert stats['removed_points'] == 0


@pytest.mark.parametrize('min_hits', [2, 3, 255])
def test_persistent_obstacle_is_confirmed_and_counter_does_not_overflow(min_hits):
    core = make_core(min_hits=min_hits)
    core.process(GROUND)
    for hit in range(1, min_hits + 260):
        keep, _ = core.process(LOW)
        assert bool(keep[0]) == (hit >= min_hits)


def test_initial_and_unobserved_obstacles_are_not_delayed():
    core = make_core()
    assert core.process(LOW)[0].tolist() == [True]
    assert core.process(LOW)[0].tolist() == [True]
    assert core.process(np.zeros((1, 3)))[0].tolist() == [True]
    assert core.process(LOW)[0].tolist() == [True]


def test_outside_band_or_roi_and_invalid_records_are_preserved():
    core = make_core()
    points = np.array([
        [1.3, 0, -0.5], [-0.2, 0, -0.5], [0.6, 0, -0.2],
        [0.6, 0, -0.58], [0, 0, 0], [np.nan, 0, 0],
        [np.inf, 0, 0], [0.6, 0.0, -0.5],
    ])
    ground = points.copy()
    ground[:, 2] = -0.6
    core.process(ground)
    keep, stats = core.process(points)
    assert keep.tolist() == [True] * 7 + [False]
    assert stats['candidate_points'] == 1


def test_leaving_band_resets_confirmation():
    core = make_core(min_hits=3)
    core.process(GROUND)
    assert not core.process(LOW)[0][0]
    core.process(GROUND)
    assert not core.process(LOW)[0][0]
    assert not core.process(LOW)[0][0]
    assert core.process(LOW)[0][0]


@pytest.mark.parametrize('stamp,reason', [
    (900_000_000, 'frame_gap'), (100_000_000, 'non_monotonic_stamp'),
    (50_000_000, 'non_monotonic_stamp'),
])
def test_discontinuous_time_resets_and_preserves_first_obstacle(stamp, reason):
    core = make_core()
    core.process(GROUND, 100_000_000)
    keep, stats = core.process(LOW, stamp)
    assert keep.tolist() == [True]
    assert stats['reset_reason'] == reason
    assert stats['warmup']


def test_point_count_change_and_explicit_reset_clear_history():
    core = make_core()
    core.process(np.repeat(GROUND, 2, axis=0))
    keep, stats = core.process(LOW)
    assert keep.tolist() == [True]
    assert stats['reset_reason'] == 'point_count_change'
    core.process(GROUND)
    core.reset()
    assert core.process(LOW)[0].tolist() == [True]
    assert core.process(np.empty((0, 3)))[0].size == 0


def test_disabled_filter_is_passthrough_and_does_not_need_calibration():
    core = make_core(enabled=False, plane_normal=[0, 0, 0],
                     plane_offset_m=float('nan'))
    core.process(GROUND)
    keep, stats = core.process(LOW)
    assert keep.tolist() == [True]
    assert stats['removed_points'] == 0
    core.enabled = True
    # Enabling without constructing a calibrated core is not a supported API.
    core = make_core()
    assert core.process(LOW)[0].tolist() == [True]


def test_scaled_plane_normal_and_offset_define_the_same_metric_height():
    core = make_core(plane_normal=[0, 0, 2], plane_offset_m=1.2)
    core.process(GROUND)
    assert core.process(LOW)[0].tolist() == [False]


@pytest.mark.parametrize('options', [
    {'plane_normal': [0, 0, 0]}, {'plane_normal': [0, 0, -1]},
    {'plane_normal': [0, 1]}, {'plane_normal': [0, np.nan, 1]},
    {'plane_offset_m': np.nan}, {'plane_offset_m': np.inf},
    {'radius_m': 0}, {'radius_m': np.nan}, {'height_min_m': 0.02},
    {'height_max_m': 0.01}, {'min_hits': 1}, {'min_hits': 256},
    {'min_hits': 2.5}, {'ground_tolerance_m': -0.1},
    {'max_frame_gap_sec': -1}, {'max_frame_gap_sec': np.inf},
])
def test_invalid_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        make_core(**options)


def test_invalid_xyz_shape_is_rejected():
    core = make_core()
    with pytest.raises(ValueError):
        core.process(np.zeros((3, 2)))
