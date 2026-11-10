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

"""Bridge launch-argument merge rules of scripts/lib/robot_profile.py."""

import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'scripts' / 'lib'))

import robot_profile  # noqa: E402


def _profile(**bridge):
    return {'bridge': bridge}


def test_runtime_overrides_native_and_common():
    args = robot_profile.bridge_launch_arguments(_profile(
        common={'cmd_vel_topic': '/a', 'cmd_vel_timeout': 0.5},
        native={'cmd_vel_timeout': 0.4, 'auth_timeout': 300},
        ros2={'package': 'p', 'launch': 'l.launch.py', 'auth_timeout': 200, 'cmd_vel_topic': '/b'},
    ), 'ros2')
    assert args == {'cmd_vel_topic': '/b', 'cmd_vel_timeout': '0.4', 'auth_timeout': '200'}


def test_values_are_formatted_for_launch():
    args = robot_profile.bridge_launch_arguments(_profile(native={
        'require_enable': True, 'auto_enter_move_mode': False,
        'control_rate': 50.0, 'sdk_timeout': 10000,
    }), 'ros2')
    assert args == {'require_enable': 'true', 'auto_enter_move_mode': 'false',
                    'control_rate': '50.0', 'sdk_timeout': '10000'}


def test_nested_and_empty_values_are_skipped():
    args = robot_profile.bridge_launch_arguments(_profile(native={
        'connection': {'robot_ip': '192.168.5.2'}, 'ids': [1, 2], 'unset': None, 'rate': 1,
    }), 'ros2')
    assert args == {'rate': '1'}


def test_sw01_profile_sets_every_declared_bridge_argument():
    profile = robot_profile.load_profile(REPO / 'robots' / 'sw01' / 'robot.yaml')
    args = robot_profile.bridge_launch_arguments(profile, 'ros2')
    launch_file = REPO / 'ros2_ws' / 'src' / 'sw01_ros_control' / 'launch' / 'sw01_control.launch.py'
    spec = importlib.util.spec_from_file_location('sw01_control_launch', launch_file)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    assert set(args) == set(launch_module.BRIDGE_PARAMETERS)
    assert args['auto_enter_move_mode'] == 'false'
    assert args['require_enable'] == 'true'
    assert args['auth_timeout'] == '200'


def test_ros1_bringup_only_receives_the_bridge_topic():
    profile = robot_profile.load_profile(REPO / 'robots' / 'dobot_lumo' / 'robot.yaml')
    args = robot_profile.ros1_bringup_arguments(profile)
    assert args['bridge_cmd_vel_topic'] == '/cmd_vel'
    assert 'cmd_vel_decay_window' not in args
    assert 'dry_run' not in args
