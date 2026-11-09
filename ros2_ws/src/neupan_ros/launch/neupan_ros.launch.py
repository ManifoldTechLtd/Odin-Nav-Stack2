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

"""Launch file for neupan_ros (ROS 2).

Environment variables:
  - PYTHONPATH must include <repo>/NeuPAN (neupan package and ros2_shim adapter)
  - NEUPAN_ROS_CONFIG selects the planner YAML config (e.g. robots/sw01/neupan/config.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    nav_mode = LaunchConfiguration('navigation_mode')
    periodic_sec = LaunchConfiguration('periodic_replan_sec')

    declare_config = DeclareLaunchArgument(
        'config_file', default_value=os.environ.get('NEUPAN_ROS_CONFIG', ''),
        description='Absolute path to robots/<robot>/neupan/config.yaml')
    declare_mode = DeclareLaunchArgument(
        'navigation_mode', default_value=os.environ.get('NAVIGATION_MODE', 'escape'),
        description='escape | replan_periodic | neupan_only')
    declare_periodic = DeclareLaunchArgument(
        'periodic_replan_sec', default_value=os.environ.get('PERIODIC_REPLAN_SEC', '2.0'))

    node = Node(
        package='neupan_ros',
        executable='neupan_action_node.py',
        name='neupan_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        additional_env={'NEUPAN_ROS_CONFIG': config_file},
        parameters=[{
            'navigation_mode': nav_mode,
            'periodic_replan_sec': periodic_sec,
        }],
    )

    return LaunchDescription([declare_config, declare_mode, declare_periodic, node])
