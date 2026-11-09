import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from slam_cloud_filter import SlamVoxelFilter


def pose(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    transform = np.eye(4)
    yaw = np.deg2rad(yaw_deg)
    transform[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0.0],
                         [np.sin(yaw), np.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]]
    transform[:3, 3] = [x, y, z]
    return transform


def run(core, points, transform=None, stamp_ns=1_000_000_000):
    return core.process(np.asarray(points, dtype=float), pose() if transform is None else transform,
                        stamp_ns=stamp_ns)


def test_first_frame_passes_through():
    core = SlamVoxelFilter()
    keep, stats = run(core, [[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]])
    assert keep.all()
    assert stats["warmup"] is True
    assert stats["reset_reason"] == "first_frame"


def test_constant_scene_reordered_and_changed_count():
    core = SlamVoxelFilter()
    first = [[0.5, 0.0, 0.0], [0.5, 0.1, 0.0], [0.4, -0.1, 0.0]]
    second = [[0.4, -0.1, 0.0], [0.5, 0.1, 0.0], [0.5, 0.0, 0.0], [0.3, 0.2, 0.0]]
    run(core, first, stamp_ns=1_000_000_000)
    keep, stats = run(core, second, stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["removed_points"] == 0


def test_one_frame_closer_return_is_deleted():
    core = SlamVoxelFilter()
    run(core, [[1.0, 0.0, 0.0]], stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.7, 0.0, 0.0]], stamp_ns=1_100_000_000)
    assert not keep[0]
    assert stats["removed_points"] == 1


def test_distance_increase_is_kept():
    core = SlamVoxelFilter()
    run(core, [[0.5, 0.0, 0.0]], stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.7, 0.0, 0.0]], stamp_ns=1_100_000_000)
    assert keep[0]
    assert stats["removed_points"] == 0


def test_persistent_obstacle_has_one_frame_delay():
    core = SlamVoxelFilter()
    run(core, [[1.0, 0.0, 0.0]], stamp_ns=1_000_000_000)
    first, stats = run(core, [[0.7, 0.0, 0.0]], stamp_ns=1_100_000_000)
    second, stats = run(core, [[0.7, 0.0, 0.0]], stamp_ns=1_200_000_000)
    assert not first[0]
    assert second[0]
    assert stats["removed_points"] == 0


def test_new_direction_without_previous_support_is_kept():
    core = SlamVoxelFilter()
    run(core, [[0.7, 0.0, 0.0]], stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.5, 0.5, 0.0]], stamp_ns=1_100_000_000)
    assert keep[0]
    assert stats["sudden_near_points"] == 0


def test_robot_translation_keeps_static_points():
    core = SlamVoxelFilter()
    points = [[0.7, 0.0, 0.0], [0.7, 0.1, 0.0]]
    run(core, points, pose(), stamp_ns=1_000_000_000)
    keep, stats = run(core, points, pose(x=0.1), stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["removed_points"] == 0


def test_robot_rotation_keeps_static_points():
    core = SlamVoxelFilter()
    points = [[0.7, 0.0, 0.0], [0.7, 0.1, 0.0]]
    run(core, points, pose(), stamp_ns=1_000_000_000)
    keep, stats = run(core, points, pose(yaw_deg=10.0), stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["removed_points"] == 0


def test_invalid_points_are_kept():
    core = SlamVoxelFilter()
    run(core, [[1.0, 0.0, 0.0], [np.nan, 0.0, 0.0], [0.0, 0.0, 0.0]],
        stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.7, 0.0, 0.0], [np.inf, 0.0, 0.0], [0.0, 0.0, 0.0]],
                      stamp_ns=1_100_000_000)
    assert not keep[0]
    assert keep[1] and keep[2]
    assert stats["valid_points"] == 1


def test_non_origin_roi_uses_sensor_view():
    core = SlamVoxelFilter()
    transform = pose(x=2.0, y=3.0)
    run(core, [[3.0, 3.0, 0.0]], transform, stamp_ns=1_000_000_000)
    keep, stats = run(core, [[2.7, 3.0, 0.0]], transform, stamp_ns=1_100_000_000)
    assert not keep[0]
    assert stats["removed_points"] == 1


def test_non_monotonic_stamp_resets():
    core = SlamVoxelFilter()
    run(core, [[0.5, 0.0, 0.0]], stamp_ns=2_000_000_000)
    keep, stats = run(core, [[0.5, 0.0, 0.0]], stamp_ns=1_000_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "non_monotonic_stamp"


def test_frame_gap_resets():
    core = SlamVoxelFilter()
    run(core, [[0.5, 0.0, 0.0]], stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.5, 0.0, 0.0]], stamp_ns=1_310_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "frame_gap"


def test_pose_jump_resets():
    core = SlamVoxelFilter()
    run(core, [[0.5, 0.0, 0.0]], pose(), stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.5, 0.0, 0.0]], pose(x=0.6), stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "pose_jump"


def test_pose_rotation_jump_resets():
    core = SlamVoxelFilter()
    run(core, [[0.5, 0.0, 0.0]], pose(), stamp_ns=1_000_000_000)
    keep, stats = run(core, [[0.5, 0.0, 0.0]], pose(yaw_deg=31.0), stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "pose_jump"


def test_disabled_passes_through():
    core = SlamVoxelFilter()
    keep, stats = core.process(np.asarray([[0.5, 0.0, 0.0]], dtype=float), pose(),
                               stamp_ns=1_000_000_000, enabled=False)
    assert keep.all()
    assert stats["reset_reason"] == "disabled"


def test_invalid_transform_resets():
    core = SlamVoxelFilter()
    keep, stats = run(core, [[0.5, 0.0, 0.0]], np.eye(3), stamp_ns=1_000_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "invalid_transform"
    bad = np.eye(4)
    bad[0, 0] = np.nan
    keep, stats = run(core, [[0.5, 0.0, 0.0]], bad, stamp_ns=1_100_000_000)
    assert keep.all()
    assert stats["reset_reason"] == "invalid_transform"


def test_bad_parameters_raise():
    for kwargs in ({"roi_radius_m": 0.0}, {"voxel_size_m": -1.0},
                   {"min_range_drop_m": 0.0}, {"angular_bin_deg": 0.0},
                   {"max_frame_gap_sec": 0.0}, {"pose_jump_m": 0.0},
                   {"pose_jump_deg": 0.0}):
        with pytest.raises(ValueError):
            SlamVoxelFilter(**kwargs)


def test_bad_xyz_shape_raises():
    core = SlamVoxelFilter()
    with pytest.raises(ValueError):
        core.process(np.zeros((3, 2)), pose())
