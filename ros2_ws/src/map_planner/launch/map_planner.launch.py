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

"""Launch file for map_planner (ROS 2).

Loads generic defaults from config/nav_params.yaml and merges them with
node-internal parameters. Static map /map is provided externally (transient_local).
Robot-specific parameters are injected by ons2_bringup from robots/<robot>/robot.yaml.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('map_planner')
    nav_params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')

    with open(nav_params_file, 'r', encoding='utf-8') as f:
        customer_params = yaml.safe_load(f) or {}
    a_keys = (
        'inflation_radius',
        'obstacle_cost_safe_distance',
        'line_of_sight_safe_distance',
        'smoothing_iterations',
        'smoothing_max_deviation_m',
    )
    a_params = {k: customer_params[k] for k in a_keys if k in customer_params}

    params = {
        'obstacle_threshold': 50,
        'sliding_map_max_age_sec': 2.0,
        'require_fresh_sliding_map': True,
        'path_max_start_offset_m': 0.20,
        'inflated_map_publish_period_sec': 1.0,
        'inflated_map_window_m': 15.0,
        'obstacle_cost_weight': 8.0,
        'turn_penalty_weight': 0.5,
        'use_global_memory': False,
        'global_memory_res': 0.20,
        'global_memory_inflation_radius': 0.35,
        'global_memory_exempt_radius_m': 0.0,
        'path_initial_heading_tol_deg': 45.0,
        'path_protect_initial_m': 1.0,
        'path_validate_enable': True,
        'path_min_clear_m': 0.10,
    }
    params.update(a_params)

    node = Node(
        package='map_planner',
        executable='map_planner_node',
        name='map_planner',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([node])
