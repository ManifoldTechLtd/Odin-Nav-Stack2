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

"""Launch file for pointcloud_saver (ROS 2).

Defaults target_frame to 'odom' so SLAM map->odom loop-closure/relocalization jumps
do not tear accumulated point clouds during mapping.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    save_dir = LaunchConfiguration('save_dir')
    declare_save_dir = DeclareLaunchArgument(
        'save_dir', default_value=os.path.expanduser('~/maps'),
        description='PCD output directory')

    params = {
        'cloud_topic': '/odin1/cloud_slam',
        'target_frame': 'odom',
        'save_dir': save_dir,
        'max_points': 10000000,
        'online_voxel_size': 0.10,
        'save_rgb': True,
        'auto_publish': True,
        'pcd_binary': False,
        'max_stat_points': 150000,
    }

    node = Node(
        package='pointcloud_saver',
        executable='pointcloud_saver_node.py',
        name='pointcloud_saver',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([declare_save_dir, node])
