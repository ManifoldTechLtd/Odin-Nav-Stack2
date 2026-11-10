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

"""Tests for the chassis enable entry of the ROS 2 control console (needs a sourced ROS 2 env)."""

import os
import sys

import pytest

rclpy = pytest.importorskip("rclpy")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_nav_control_ros2 as console  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


class _FakeSetBoolClient:
    def __init__(self, success=True, message="ok"):
        self.requests = []
        self._success = success
        self._message = message

    def wait_for_service(self, timeout_sec=None):
        return True

    def call(self, request, timeout_sec=None):
        self.requests.append(request.data)
        response = SetBool.Response()
        response.success = self._success
        response.message = self._message
        return response


@pytest.fixture
def node():
    rclpy.init()
    test_node = rclpy.create_node("nav_control_chassis_test")
    yield test_node
    test_node.destroy_node()
    rclpy.shutdown()


def _console(node, allow_motion=True):
    nav = console.NavControlTest(node, "map", 0.25, allow_motion, False)
    fake = _FakeSetBoolClient()
    nav._clients[console.CHASSIS_ENABLE_SERVICE] = fake
    return nav, fake


def _answers(monkeypatch, *answers):
    replies = iter(answers)
    monkeypatch.setattr("builtins.input", lambda prompt="": next(replies))


def test_console_starts_without_rtk_helpers(node):
    nav, _ = _console(node)
    assert nav.rtk.status_lines()


def test_enable_requires_confirmation_and_calls_bridge(node, monkeypatch):
    nav, fake = _console(node)
    _answers(monkeypatch, "e", "y")
    nav.toggle_chassis_motion()
    assert fake.requests == [True]


def test_enable_declined_does_not_call_bridge(node, monkeypatch):
    nav, fake = _console(node)
    _answers(monkeypatch, "e", "n")
    nav.toggle_chassis_motion()
    assert fake.requests == []


def test_disable_calls_bridge_without_confirmation(node, monkeypatch):
    nav, fake = _console(node)
    _answers(monkeypatch, "d")
    nav.toggle_chassis_motion()
    assert fake.requests == [False]


def test_read_only_console_never_enables(node, monkeypatch):
    nav, fake = _console(node, allow_motion=False)
    _answers(monkeypatch, "e", "y")
    nav.toggle_chassis_motion()
    assert fake.requests == []
