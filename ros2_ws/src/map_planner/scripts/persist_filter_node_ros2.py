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

"""Standalone entry point for OdinSlamTemporalFilter with default node name `persist_filter`."""

from odin_slam_temporal_filter import (
    OdinSlamTemporalFilter as PersistFilter,
    main as _main,
    pack_cloud,
    unpack_cloud,
    voxel_keys,
)

__all__ = ['PersistFilter', 'pack_cloud', 'unpack_cloud', 'voxel_keys', 'main']


def main(args=None):
    _main(args=args, default_node_name='persist_filter')


if __name__ == '__main__':
    main()
