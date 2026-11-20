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

Every bridge parameter is a launch argument; ons2_bringup fills them from
robots/sw01/robot.yaml. Motion is enabled by default and takes effect whenever
the remote controller switches to MOVE mode and releases control authority.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name -> (type, default). Launch arguments are strings; they are converted to these types so the
# node's typed declare_parameter() calls accept them (e.g. "50" must become 50.0 for control_rate).
BRIDGE_PARAMETERS = {
    'cmd_vel_topic': (str, os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')),
    'max_linear_vel': (float, 1.0),
    'max_angular_vel': (float, 1.0),
    'cmd_vel_timeout': (float, 0.5),
    'cmd_vel_decay_window': (float, 0.0),   # 0.0 = disabled
    'heartbeat_rate': (float, 10.0),
    'control_rate': (float, 50.0),
    'sdk_timeout': (int, 10000),
    'auth_timeout': (int, 200),
    'move_timeout': (int, 20),
    'heartbeat_timeout': (int, 20),
    'request_control_authority': (bool, True),
    'require_move_mode': (bool, True),
    'auto_enter_move_mode': (bool, False),
    'publish_status': (bool, True),
    'require_enable': (bool, False),
    'blocking_error_mask': (int, 0),
}

_DESCRIPTIONS = {
    'cmd_vel_topic': 'Subscribed velocity topic (default /cmd_vel)',
    'auto_enter_move_mode': 'Allow DAMPING/FIXEDSTAND -> MOVE while enabled and commanded',
    'require_enable': 'Require ~/enable before motion (disabled again on authority/link loss or fault)',
}

_TRUE_WORDS = ('true', '1', 'yes', 'on')
_FALSE_WORDS = ('false', '0', 'no', 'off')


def _to_bool(name, text):
    word = text.lower()
    if word in _TRUE_WORDS:
        return True
    if word in _FALSE_WORDS:
        return False
    raise ValueError(f'{name}: expected a boolean, got {text!r}')


def _to_int(name, text):
    try:
        return int(text, 0)
    except ValueError:
        number = float(text)
        if not number.is_integer():
            raise ValueError(f'{name}: expected an integer, got {text!r}') from None
        return int(number)


def _convert(name, kind, value):
    text = str(value).strip()
    if kind is bool:
        return _to_bool(name, text)
    if kind is int:
        return _to_int(name, text)
    if kind is float:
        return float(text)
    return text


def typed_parameters(context):
    """Convert the bridge launch arguments to the parameter types the node declares."""
    return {
        name: _convert(name, kind, LaunchConfiguration(name).perform(context))
        for name, (kind, _) in BRIDGE_PARAMETERS.items()
    }


def _launch_bridge(context, *args, **kwargs):
    return [Node(
        package='sw01_ros_control',
        executable='sw01_cmd_vel_bridge',
        name='sw01_cmd_vel_bridge',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[typed_parameters(context)],
    )]


def _default_text(kind, default):
    if kind is bool:
        return 'true' if default else 'false'
    return str(default)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            name, default_value=_default_text(kind, default),
            description=_DESCRIPTIONS.get(name, f'sw01_cmd_vel_bridge parameter ({kind.__name__})'))
        for name, (kind, default) in BRIDGE_PARAMETERS.items()
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_bridge)])
