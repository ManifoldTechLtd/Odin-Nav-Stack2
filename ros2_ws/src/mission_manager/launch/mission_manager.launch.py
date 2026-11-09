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

"""Launch file for mission_manager (ROS 2).

Reads generic defaults from map_planner/config/nav_params.yaml and allows
overrides via environment variables or launch arguments.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('map_planner')
    nav_params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    with open(nav_params_file, 'r', encoding='utf-8') as f:
        customer = yaml.safe_load(f) or {}

    nav_mode = LaunchConfiguration('navigation_mode')
    periodic_sec = LaunchConfiguration('periodic_replan_sec')
    local_planner = LaunchConfiguration('local_planner')
    local_planner_mode = LaunchConfiguration('local_planner_mode')
    plan_service = LaunchConfiguration('plan_service')
    nav_path_action = LaunchConfiguration('nav_path_action')
    stop_topic = LaunchConfiguration('stop_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    enable_topic = LaunchConfiguration('enable_topic')
    planner_cmd_vel_topic = LaunchConfiguration('planner_cmd_vel_topic')

    declare_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value=os.environ.get(
            'NAVIGATION_MODE', str(customer.get('navigation_mode', 'replan_periodic'))),
        description='replan_periodic (default) | escape | neupan_only')
    declare_periodic = DeclareLaunchArgument(
        'periodic_replan_sec',
        default_value=os.environ.get(
            'PERIODIC_REPLAN_SEC', str(customer.get('periodic_replan_sec', 2.0))))
    declare_local_planner = DeclareLaunchArgument(
        'local_planner',
        default_value=os.environ.get(
            'LOCAL_PLANNER', str(customer.get('local_planner', 'neupan'))),
        description='Local planner backend key in robot.yaml (default: neupan)')
    declare_local_planner_mode = DeclareLaunchArgument(
        'local_planner_mode',
        default_value=os.environ.get('LOCAL_PLANNER_MODE', ''),
        description='Interface contract: action (path tracking) | goal_pose (end-to-end PoseStamped) | empty (auto)')
    declare_plan_service = DeclareLaunchArgument('plan_service', default_value='/map_planner/plan')
    declare_nav_path_action = DeclareLaunchArgument('nav_path_action', default_value='/neupan/nav_path')
    declare_stop_topic = DeclareLaunchArgument('stop_topic', default_value='/neupan/stop')
    declare_goal_topic = DeclareLaunchArgument('goal_topic', default_value='/sru/goal_pose')
    declare_enable_topic = DeclareLaunchArgument('enable_topic', default_value='/sru/cmd_vel_enable')
    declare_planner_cmd_vel = DeclareLaunchArgument('planner_cmd_vel_topic', default_value='/sru/cmd_vel')

    params = {
        # Coordinate frames and service/action endpoints
        'map_frame': 'map',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'odom_topic': '/odin1/odometry',
        'plan_service': plan_service,
        'nav_path_action': nav_path_action,
        'stop_topic': stop_topic,
        'goal_topic': goal_topic,
        'enable_topic': enable_topic,
        'planner_cmd_vel_topic': planner_cmd_vel_topic,
        # Local planner backend and contract mode
        'local_planner': local_planner,
        'local_planner_mode': local_planner_mode,
        'sru_arrive_threshold_m': float(customer.get('sru_arrive_threshold_m', 0.85)),
        'sru_robot_frame': str(customer.get('sru_robot_frame', 'odin1_base_link')),
        # Mission and final alignment
        'enable_alignment': True,
        'periodic_replan_sec': periodic_sec,
        'initial_plan_timeout_sec': 15.0,
        # Route segmentation (replan_periodic mode)
        'route_segment_enable': True,
        'route_segment_length_m': 10.0,
        'route_segment_overlap_m': 3.0,
        'path_safety_scan_max_age_sec': 1.0,
        # Recovery contract (escape mode)
        'recovery_shadow_enable': True,
        'recovery_behavior_enable': True,
        # Watchdogs and handover
        'nav_watchdog_timeout_sec': 5.0,
        'nav_stall_timeout_sec': 15.0,
        'alignment_watchdog_timeout_sec': 25.0,
        'handover_settle_sec': 0.2,
        # Dynamic target following mode (disabled by default)
        'enable_dynamic_mode': False,
        'dynamic_replan_cooldown_sec': 0.5,
        'dynamic_goal_topic': '/waypoint_manager/dynamic_goal',
        # Stepped rotation search on target loss (disabled by default)
        'enable_follow_search': False,
        'follow_search_clearance_m': 0.60,
        'follow_search_step_deg': 90.0,
        'follow_search_step_pause_sec': 0.3,
        'follow_search_timeout_sec': 0.0,
        'follow_search_total_limit_deg': 0.0,
        'follow_search_turn_service': '/atomic_move/turn',
        'follow_search_scan_topic': '/scan_360',
        # Periodic replan install gate
        'install_gate_enable': False,
        'install_gate_shadow': False,
    }

    node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[params, {'navigation_mode': nav_mode}],
    )

    return LaunchDescription([
        declare_mode,
        declare_periodic,
        declare_local_planner,
        declare_local_planner_mode,
        declare_plan_service,
        declare_nav_path_action,
        declare_stop_topic,
        declare_goal_topic,
        declare_enable_topic,
        declare_planner_cmd_vel,
        node,
    ])
