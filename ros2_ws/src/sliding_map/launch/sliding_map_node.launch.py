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

"""Launch file for sliding_map (ROS 2).

Loads parameters from config/sliding_map.yaml. Topic QoS is configured in the node:
  /sliding_map        -> transient_local
  /scan, /scan_360 -> reliable QoS(5)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sliding_map')
    params_file = os.path.join(pkg_share, 'config', 'sliding_map.yaml')

    return LaunchDescription([
        Node(
            package='sliding_map',
            executable='sliding_map_node',
            name='sliding_map_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
