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

"""Launch file for atomic_move (ROS 2).

Loads parameters from config/atomic_move.yaml and names the node `atomic_move`
so private services resolve to /atomic_move/move, /atomic_move/turn, and /atomic_move/align.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('atomic_move')
    params_file = os.path.join(pkg_share, 'config', 'atomic_move.yaml')

    return LaunchDescription([
        Node(
            package='atomic_move',
            executable='atomic_move_node',
            name='atomic_move',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[params_file],
        ),
    ])
