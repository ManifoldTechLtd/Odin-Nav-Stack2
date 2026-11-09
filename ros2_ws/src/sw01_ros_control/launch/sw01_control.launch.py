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

"""Launch file for sw01_ros_control (ROS 2).

Prerequisites:
  1) SDK network interface configured (e.g. 10.18.0.200/24 reaching SW01 at 10.18.0.100, UDP 3600)
  2) SW01 in DAMPING or FIXEDDOWN state so the SDK link can be established
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    declare_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=os.environ.get('CMD_VEL_TOPIC', '/cmd_vel'),
        description='Subscribed velocity topic (default /cmd_vel)')

    params = {
        'cmd_vel_topic': cmd_vel_topic,
        'max_linear_vel': 1.0,
        'max_angular_vel': 1.0,
        'cmd_vel_timeout': 0.5,
        'cmd_vel_decay_window': 0.0,   # 0.0 = disabled
        'heartbeat_rate': 10.0,
        'control_rate': 50.0,
        'sdk_timeout': 10000,
        'auth_timeout': 2000,
        'move_timeout': 20,
        'heartbeat_timeout': 20,
        'request_control_authority': True,
        'require_move_mode': True,
        'auto_enter_move_mode': True,  # Automatically transitions robot into MOVE mode
        'publish_status': True,
    }

    node = Node(
        package='sw01_ros_control',
        executable='sw01_cmd_vel_bridge',
        name='sw01_cmd_vel_bridge',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[params],
    )

    return LaunchDescription([declare_topic, node])
