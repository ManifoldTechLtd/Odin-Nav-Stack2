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
import struct
import sys
from pathlib import Path

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'odin_cloud_temporal_filter.py'
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location('odin_cloud_temporal_filter', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
TwoFrameVoxelFilter = MODULE.TwoFrameVoxelFilter
select_pointcloud_records = MODULE.select_pointcloud_records


def test_two_frame_voxel_match_only_filters_roi():
    core = TwoFrameVoxelFilter(roi_radius_m=0.8, voxel_size_m=0.05)
    first = np.array([
        [0.32, 0.01, -0.30],
        [0.55, 0.40, -0.30],      # Farther baseline for the same scan index
        [0.90, 0.00, -0.30],
        [0.00, 0.00, 0.00],
    ], dtype=np.float32)
    keep, stats = core.process(first, stamp_ns=100_000_000)
    assert keep.tolist() == [True, True, True, True]
    assert stats.warmup

    second = np.array([
        [0.32, 0.01, -0.30],      # Same voxel as previous frame, kept
        [0.20, 0.20, -0.20],      # New voxel with sudden range drop on same ray, removed
        [0.90, 0.00, -0.30],      # Outside ROI, untouched
        [0.00, 0.00, 0.00],       # Invalid zero point preserved
    ], dtype=np.float32)
    keep, stats = core.process(second, stamp_ns=200_000_000)
    assert keep.tolist() == [True, False, True, True]
    assert stats.roi_points == 2
    assert stats.removed_points == 1
    assert stats.sudden_near_points == 1

    # Once transient noise disappears and the ray returns to the far ground, it must not be removed.
    keep, stats = core.process(first, stamp_ns=300_000_000)
    assert keep.tolist() == [True, True, True, True]
    assert stats.removed_points == 0


def test_warmup_can_require_two_hits_and_time_gap_resets():
    core = TwoFrameVoxelFilter(
        roi_radius_m=0.8, voxel_size_m=0.05,
        warmup_passthrough=False, max_frame_gap_sec=0.3)
    point = np.array([[0.3, 0.0, -0.2]], dtype=np.float32)
    keep, _ = core.process(point, stamp_ns=100_000_000)
    assert not keep[0]
    keep, _ = core.process(point, stamp_ns=200_000_000)
    assert keep[0]
    keep, stats = core.process(point, stamp_ns=800_000_000)
    assert not keep[0]
    assert stats.reset_reason == 'frame_gap'


def test_compaction_preserves_custom_point_record_bytes():
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='confidence', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='offset_time', offset=14, datatype=PointField.FLOAT32, count=1),
    ]
    record_a = struct.pack('<fffBBf', 0.2, 0.1, -0.3, 120, 80, 0.01)
    record_b = struct.pack('<fffBBf', 1.2, 0.2, -0.4, 230, 150, 0.09)
    cloud = PointCloud2(
        height=1, width=2, fields=fields, is_bigendian=False,
        point_step=18, row_step=36, data=record_a + record_b, is_dense=True)

    output = select_pointcloud_records(cloud, np.array([False, True]))
    assert output.height == 1
    assert output.width == 1
    assert output.point_step == 18
    assert output.row_step == 18
    assert bytes(output.data) == record_b


def test_invalid_short_return_is_preserved_not_classified_as_a_near_obstacle():
    core = TwoFrameVoxelFilter()
    core.process(np.array([[0.6, 0, -0.3]]))
    keep, stats = core.process(np.array([[0.01, 0, 0]]))
    assert keep.tolist() == [True]
    assert stats.valid_points == stats.removed_points == 0
