#!/usr/bin/python3
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

"""Unit tests for SlamTransientFilter (near-field transient noise filter on cloud_slam)."""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'slam_transient_filter.py'
SPEC = importlib.util.spec_from_file_location('slam_transient_filter', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
SlamTransientFilter = MODULE.SlamTransientFilter

I4 = np.eye(4)
GROUND = np.array([[x, y, -0.5] for x in (0.5, 1.5, 3.0) for y in (-0.6, 0.0, 0.6)], dtype=np.float64)
SPLASH = np.array([[0.30, 0.00, 0.00],
                   [0.32, 0.02, -0.02],
                   [0.34, -0.02, -0.04]], dtype=np.float64)


def make(**kw):
    return SlamTransientFilter(**kw)


def run(filter_, frame, stamp_ns):
    keep, stats = filter_.process(frame, I4, stamp_ns)
    return keep, stats


def person_column(nx=2, ny=7, nz=19, x0=0.80, y0=-0.15, z0=-0.35):
    """Approximate pedestrian as a vertical dense point column exceeding max_cluster_voxels."""
    xs = x0 + 0.05 * np.arange(nx)
    ys = y0 + 0.05 * np.arange(ny)
    zs = z0 + 0.05 * np.arange(nz)
    return np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=np.float64)


def test_splash_without_prior_return_is_removed():
    f = make()
    run(f, GROUND, 100_000_000)
    frame = np.vstack([GROUND, SPLASH])
    keep, stats = run(f, frame, 200_000_000)
    assert keep[len(GROUND):].tolist() == [False] * len(SPLASH)
    assert keep[:len(GROUND)].all()
    assert stats['removed_points'] == len(SPLASH)


def test_splash_in_front_of_far_wall_is_removed():
    f = make()
    wall = np.array([[3.0, y, 0.0] for y in (-0.3, 0.0, 0.3)], dtype=np.float64)
    run(f, wall, 100_000_000)
    keep, _ = run(f, np.vstack([wall, SPLASH]), 200_000_000)
    assert keep[len(wall):].tolist() == [False] * len(SPLASH)


def test_ground_recovery_is_never_removed():
    f = make()
    run(f, np.vstack([GROUND, SPLASH]), 100_000_000)
    keep, stats = run(f, GROUND, 200_000_000)
    assert keep.all()
    assert stats['roi_points'] == 0


def test_person_like_column_is_protected_by_large_cluster_guard():
    f = make()
    run(f, GROUND, 100_000_000)
    person = person_column()
    keep, stats = run(f, np.vstack([GROUND, person]), 200_000_000)
    assert keep[len(GROUND):].all()
    assert stats['sudden_near_points'] > 0


def test_guard_is_what_protects_person_not_the_judgement():
    f = make(max_cluster_voxels=10 ** 9)
    run(f, GROUND, 100_000_000)
    person = person_column()
    keep, _ = run(f, np.vstack([GROUND, person]), 200_000_000)
    kept_person = keep[len(GROUND):]
    in_band = (person[:, 2] >= -0.35) & (person[:, 2] <= 0.35)
    assert not kept_person[in_band].any()
    assert kept_person[~in_band].all()


def test_persistent_near_obstacle_kept_by_voxel_support():
    f = make()
    box = np.array([[0.35, 0.0, 0.0], [0.36, 0.05, 0.0]], dtype=np.float64)
    run(f, np.vstack([GROUND, box]), 100_000_000)
    keep, _ = run(f, np.vstack([GROUND, box]), 200_000_000)
    assert keep[len(GROUND):].all()


def test_range_increase_kept():
    f = make()
    near = np.array([[0.20, 0.0, 0.0]], dtype=np.float64)
    run(f, np.vstack([GROUND, near]), 100_000_000)
    far = np.array([[0.90, 0.0, 0.0]], dtype=np.float64)
    keep, _ = run(f, np.vstack([GROUND, far]), 200_000_000)
    assert keep[len(GROUND):].all()


def test_outside_roi_and_height_band_untouched():
    f = make()
    run(f, GROUND, 100_000_000)
    outside = np.array([[1.30, 0.00, 0.00],
                        [-0.40, 0.00, 0.00],
                        [0.30, 0.00, 0.55]],
                       dtype=np.float64)
    keep, _ = run(f, np.vstack([GROUND, outside]), 200_000_000)
    assert keep[len(GROUND):].all()


def test_large_frame_gap_resets_history():
    f = make(max_frame_gap_sec=0.30)
    run(f, GROUND, 100_000_000)
    keep, stats = run(f, np.vstack([GROUND, SPLASH]), 900_000_000)
    assert stats['warmup'] is True
    assert keep.all()


def test_new_direction_outside_previous_fov_is_kept():
    f = make(fov_az_deg=(-10.0, 10.0), fov_el_deg=(-5.0, 5.0))
    run(f, GROUND, 100_000_000)
    lateral = np.array([[0.30, 0.40, 0.00]], dtype=np.float64)
    keep, _ = run(f, np.vstack([GROUND, lateral]), 200_000_000)
    assert keep[-1]


def test_disabled_style_empty_history_first_frame_passthrough():
    f = make()
    keep, stats = run(f, np.vstack([GROUND, SPLASH]), 100_000_000)
    assert stats['warmup'] is True
    assert keep.all()


def test_same_voxel_across_roi_boundary_is_not_clipped():
    f = make()
    p_prev = np.array([[0.7199, 0.7499, 0.029]], dtype=np.float64)
    p_curr = np.array([[0.6901, 0.7201, 0.0001]], dtype=np.float64)
    bg_wall = np.array([[1.3802, 1.4402, 0.0002]], dtype=np.float64)
    run(f, np.vstack([GROUND, p_prev, bg_wall]), 100_000_000)
    keep, stats = run(f, np.vstack([GROUND, p_curr, bg_wall]), 200_000_000)
    assert keep[-2]
    assert stats['matched_voxel_points'] == 1
    assert stats['removed_points'] == 0


def test_pose_and_parameter_validation_and_enabled_contract():
    f = make()
    run(f, GROUND, 100_000_000)
    keep, stats = f.process(SPLASH, None, enabled=False)
    assert keep.all()
    assert stats['warmup'] is True
    for bad in (np.zeros((4, 4)), np.diag([2.0, 1.0, 1.0, 1.0])):
        with pytest.raises(ValueError):
            f.process(SPLASH, bad, 100_000_000)
    with pytest.raises(ValueError):
        make(max_rotation_jump_deg=181.0)
