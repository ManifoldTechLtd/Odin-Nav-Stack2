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

"""Typed launch arguments of sw01_control.launch.py."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'sw01_control.launch.py'


def _load():
    spec = importlib.util.spec_from_file_location('sw01_control_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared(module):
    return [entity for entity in module.generate_launch_description().entities
            if isinstance(entity, DeclareLaunchArgument)]


def _context(module, **overrides):
    context = LaunchContext()
    for name, value in overrides.items():
        context.launch_configurations[name] = value
    for argument in _declared(module):
        argument.visit(context)
    return context


def test_defaults_are_safe_and_typed():
    module = _load()
    params = module.typed_parameters(_context(module))
    assert params['auto_enter_move_mode'] is False
    assert params['require_enable'] is True
    assert params['auth_timeout'] == 200 and isinstance(params['auth_timeout'], int)
    assert params['control_rate'] == 50.0 and isinstance(params['control_rate'], float)
    assert params['blocking_error_mask'] == 0


def test_every_bridge_parameter_is_a_launch_argument():
    module = _load()
    assert {argument.name for argument in _declared(module)} == set(module.BRIDGE_PARAMETERS)


def test_string_arguments_are_converted_to_declared_types():
    module = _load()
    params = module.typed_parameters(_context(
        module, control_rate='50', auto_enter_move_mode='true', auth_timeout='200.0',
        blocking_error_mask='0x4', max_linear_vel='0.5'))
    assert params['control_rate'] == 50.0 and isinstance(params['control_rate'], float)
    assert params['auto_enter_move_mode'] is True
    assert params['auth_timeout'] == 200 and isinstance(params['auth_timeout'], int)
    assert params['blocking_error_mask'] == 4
    assert params['max_linear_vel'] == 0.5


@pytest.mark.parametrize('name,value', [('require_enable', 'maybe'), ('auth_timeout', '200.5')])
def test_invalid_values_are_rejected(name, value):
    module = _load()
    with pytest.raises(ValueError):
        module.typed_parameters(_context(module, **{name: value}))
