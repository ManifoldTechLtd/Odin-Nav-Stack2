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
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


SCRIPT = Path(__file__).parents[1] / 'scripts/slam_cloud_filter.py'


def load_module():
    assert SCRIPT.exists(), 'slam_cloud_filter.py is missing'
    spec = importlib.util.spec_from_file_location('slam_cloud_filter_unit', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_filter(**options):
    module = load_module()
    assert hasattr(module, 'PersistVoxelFilter'), 'PersistVoxelFilter is not implemented'
    return module.PersistVoxelFilter(**options)


def test_voxel_key_matches_feishu_bit_packing():
    module = load_module()
    assert hasattr(module, 'PersistVoxelFilter'), 'PersistVoxelFilter is not implemented'
    pts = np.array([
        [0.001, -0.001, 0.025],
        [0.040, 0.060, -0.040],
    ], dtype=np.float32)
    keys = module.PersistVoxelFilter.voxel_keys(pts, 0.02)
    q = np.floor(pts / 0.02).astype(np.int64)
    bias = 1 << 20
    expected = ((q[:, 0] + bias) << 42) | ((q[:, 1] + bias) << 21) | (q[:, 2] + bias)
    np.testing.assert_array_equal(keys, expected)
    assert keys.dtype == np.int64


def test_counts_frames_not_points_within_same_frame():
    flt = make_filter(res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    # 4 points in the same 2cm voxel in a single frame must count as 1 frame hit, not 4.
    cluster_same_voxel = np.array([
        [1.001, 0.001, 0.101],
        [1.005, 0.002, 0.102],
        [1.009, 0.003, 0.103],
        [1.012, 0.004, 0.104],
    ], dtype=np.float32)
    filtered, single_shot, near, stats = flt.classify(cluster_same_voxel, odom_xy=(0.0, 0.0))
    assert not filtered.any()
    assert single_shot.all()
    assert not near.any()
    assert stats['tracked_points'] == 4
    assert stats['single_shot_points'] == 4
    assert stats['persistent_points'] == 0

    # Second frame hits the same voxel -> count reaches 2 >= persist, all 4 points become persistent.
    filtered, single_shot, near, stats = flt.classify(cluster_same_voxel, odom_xy=(0.0, 0.0))
    assert filtered.all()
    assert not single_shot.any()
    assert not near.any()
    assert stats['persistent_points'] == 4
    assert stats['single_shot_points'] == 0


def test_fifo_window_evicts_old_frame_sets():
    flt = make_filter(res=0.02, window=3, persist=2, near_radius=0.5, active_radius=2.0)
    target = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    other = np.array([[1.5, 0.0, 0.0]], dtype=np.float32)

    # Frame 1: target seen once -> single_shot
    filtered, single_shot, _, _ = flt.classify(target, odom_xy=(0.0, 0.0))
    assert not filtered[0] and single_shot[0]

    # Frames 2, 3, 4: only 'other' is seen; frame 1 falls out of the 3-frame window
    for _ in range(3):
        flt.classify(other, odom_xy=(0.0, 0.0))

    # Frame 5: target seen again, but frame 1 was evicted -> count is 1 again (single_shot)
    filtered, single_shot, _, _ = flt.classify(target, odom_xy=(0.0, 0.0))
    assert not filtered[0] and single_shot[0]


def test_near_blind_zone_is_excluded_from_tracking_and_filtered():
    flt = make_filter(res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    # Point at distance 0.3m from odom (10.0, 20.0) is inside near_radius=0.5m
    pt = np.array([[10.3, 20.0, 0.0]], dtype=np.float32)
    for _ in range(4):
        filtered, single_shot, near, stats = flt.classify(pt, odom_xy=(10.0, 20.0))
        assert near.tolist() == [True]
        assert filtered.tolist() == [False]
        assert single_shot.tolist() == [False]
        assert stats['near_points'] == 1
        assert stats['tracked_points'] == 0

    # When the robot moves so the same world voxel is now at distance 1.0m (tracked),
    # it has 0 prior history because near points never entered the frame set.
    filtered, single_shot, near, _ = flt.classify(pt, odom_xy=(9.3, 20.0))
    assert near.tolist() == [False]
    assert single_shot.tolist() == [True]
    assert filtered.tolist() == [False]


def test_far_points_passthrough_immediately_without_tracking():
    flt = make_filter(res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    far_pt = np.array([[2.5, 0.0, 0.0]], dtype=np.float32)
    filtered, single_shot, near, stats = flt.classify(far_pt, odom_xy=(0.0, 0.0))
    assert filtered.tolist() == [True]
    assert single_shot.tolist() == [False]
    assert near.tolist() == [False]
    assert stats['far_points'] == 1
    assert stats['tracked_points'] == 0


def test_zero_or_negative_radii_and_missing_odom_behavior():
    # near_radius=0 disables near branch; active_radius=0 tracks even far points
    flt = make_filter(res=0.02, window=5, persist=2, near_radius=0.0, active_radius=0.0)
    pts = np.array([[0.1, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=np.float32)
    filtered, single_shot, near, stats = flt.classify(pts, odom_xy=(0.0, 0.0))
    assert not near.any()
    assert single_shot.all()
    assert not filtered.any()
    assert stats['far_points'] == 0
    assert stats['tracked_points'] == 2

    # Missing odom (odom_xy=None) disables near/active radius and tracks all finite points
    flt_no_odom = make_filter(res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    filtered, single_shot, near, stats = flt_no_odom.classify(pts, odom_xy=None)
    assert not near.any()
    assert single_shot.all()
    assert not filtered.any()
    assert not stats['has_odom']
    assert stats['tracked_points'] == 2


def test_non_finite_points_are_dropped_from_all_three_outputs():
    flt = make_filter(res=0.02, window=5, persist=2, near_radius=0.5, active_radius=2.0)
    pts = np.array([
        [np.nan, 0.0, 0.0],
        [1.0, np.inf, 0.0],
        [1.0, 0.0, -np.inf],
        [0.2, 0.0, 0.0],   # near
        [1.0, 0.0, 0.0],   # tracked single-shot
        [2.5, 0.0, 0.0],   # far passthrough
    ], dtype=np.float32)
    filtered, single_shot, near, stats = flt.classify(pts, odom_xy=(0.0, 0.0))
    assert not (filtered[:3] | single_shot[:3] | near[:3]).any()
    assert near.tolist() == [False, False, False, True, False, False]
    assert single_shot.tolist() == [False, False, False, False, True, False]
    assert filtered.tolist() == [False, False, False, False, False, True]
    assert stats['input_points'] == 6
    assert stats['valid_points'] == 3


def test_parse_and_pack_16byte_xyz_rgb_clouds():
    module = load_module()
    header = Header(frame_id='odom')
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='confidence', offset=16, datatype=PointField.UINT16, count=1),
    ]
    rgb_bits = 0x112233
    rgb_f32 = np.uint32(rgb_bits).view(np.float32)
    raw_bytes = (
        struct.pack('<fff f H', 1.0, 2.0, 3.0, float(rgb_f32), 100)
        + struct.pack('<fff f H', float('nan'), 0.0, 0.0, float(rgb_f32), 50)
    )
    msg = PointCloud2(
        header=header, height=1, width=2, is_bigendian=False,
        point_step=18, row_step=36, is_dense=False, fields=fields, data=raw_bytes)

    xyz, rgb = module.parse_pointcloud_xyz_rgb(msg)
    assert xyz.shape == (2, 3)
    assert rgb.shape == (2,)
    assert int(rgb.view(np.uint32)[0]) == rgb_bits

    out_filtered = module.pack_xyz_rgb_cloud(header, xyz[:1], rgb[:1])
    assert out_filtered.header == header
    assert out_filtered.point_step == 16
    assert out_filtered.row_step == 16
    assert out_filtered.width == 1 and out_filtered.height == 1
    assert [f.name for f in out_filtered.fields] == ['x', 'y', 'z', 'rgb']
    unpacked = np.frombuffer(out_filtered.data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    assert int(unpacked['rgb'][0]) == rgb_bits

    out_near = module.pack_xyz_rgb_cloud(header, xyz[:1], module.COLOR_NEAR_RGB_F32)
    unpacked_near = np.frombuffer(out_near.data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    assert int(unpacked_near['rgb'][0]) == 0xFFFF00

    out_single = module.pack_xyz_rgb_cloud(header, xyz[:1], module.COLOR_SINGLE_SHOT_RGB_F32)
    unpacked_single = np.frombuffer(out_single.data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    assert int(unpacked_single['rgb'][0]) == 0xFF0000


@pytest.mark.parametrize('options', [
    {'res': 0.0},
    {'res': -0.01},
    {'res': np.nan},
    {'window': 1},
    {'window': True},
    {'persist': 1},
    {'persist': 6, 'window': 5},
    {'persist': False},
    {'near_radius': np.nan},
    {'active_radius': np.inf},
    {'near_radius': 2.5, 'active_radius': 2.0},
])
def test_invalid_parameters_rejected(options):
    with pytest.raises(ValueError):
        make_filter(**options)
