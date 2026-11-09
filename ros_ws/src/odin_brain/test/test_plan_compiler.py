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

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from odin_brain.models import Plan, Step  # noqa: E402
from odin_brain.plan_compiler import compile_segments, describe_segments, is_pure_mission  # noqa: E402
from odin_brain.places import PlaceStore  # noqa: E402
from odin_brain.planner import RulePlanner  # noqa: E402


def _store(tmpdir):
    path = os.path.join(tmpdir, "places.json")
    data = {
        "schema_version": 1,
        "map_id": "map_opt",
        "places": [
            {"id": "p.desk", "name": "桌子", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
            {"id": "p.fridge", "name": "冰箱", "pose": {"x": 5.0, "y": 6.0, "yaw": 0.5}},
            {"id": "p.seat", "name": "工位A", "pose": {"x": 3.0, "y": 4.0, "yaw": 1.0}},
        ],
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False)
    return PlaceStore(path)


class TestMixedPlan(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.planner = RulePlanner(_store(self.tmpdir))

    def test_mixed_plan_parsing(self):
        text = "先去桌子，喊一声忠诚，然后再去冰箱，等待十秒钟，然后回到工位A，转180度，然后说介绍词"
        plan = self.planner.plan(text)
        kinds = [step.kind for step in plan.steps]
        self.assertEqual(kinds, ["goto", "speak", "goto", "goto", "turn", "speak"], kinds)
        self.assertEqual(plan.steps[0].place.name, "桌子")
        self.assertEqual(plan.steps[1].text, "忠诚")
        self.assertEqual(plan.steps[2].place.name, "冰箱")
        self.assertAlmostEqual(plan.steps[2].dwell_sec, 10.0)
        self.assertEqual(plan.steps[3].place.name, "工位A")
        self.assertAlmostEqual(plan.steps[4].angle_deg, 180.0)
        self.assertIn("硬核璀璨明珠", plan.steps[5].text)

    def test_turn_directions(self):
        left = self.planner.plan("左转90度").steps[0]
        self.assertEqual(left.kind, "turn")
        self.assertAlmostEqual(left.angle_deg, 90.0)

        right = self.planner.plan("右转45度").steps[0]
        self.assertAlmostEqual(right.angle_deg, -45.0)

        plain = self.planner.plan("转180度").steps[0]
        self.assertAlmostEqual(plain.angle_deg, 180.0)

    def test_move_rel_directions(self):
        cases = {
            "前进2米": ("forward", 2.0),
            "后退0.5米": ("backward", 0.5),
            "左移1米": ("left", 1.0),
            "往右3米": ("right", 3.0),
        }
        for text, (direction, distance) in cases.items():
            step = self.planner.plan(text).steps[0]
            self.assertEqual(step.kind, "move_rel", text)
            self.assertEqual(step.direction, direction, text)
            self.assertAlmostEqual(step.distance_m, distance, msg=text)

    def test_speak_action(self):
        step = self.planner.plan("说一句你好").steps[0]
        self.assertEqual(step.kind, "speak")
        self.assertEqual(step.text, "你好")

    def test_pure_goto_unchanged(self):
        plan = self.planner.plan("去桌子")
        self.assertEqual(len(plan.steps), 1)
        self.assertEqual(plan.steps[0].kind, "goto")


class TestCompiler(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.store = _store(self.tmpdir)

    def _place(self, name):
        return self.store.get(name)

    def test_consecutive_goto_merged(self):
        plan = Plan(steps=[
            Step(place=self._place("桌子")),
            Step(place=self._place("冰箱")),
        ])
        segments = compile_segments(plan)
        self.assertEqual(len(segments), 1)
        self.assertTrue(is_pure_mission(segments))
        self.assertEqual(len(segments[0]["waypoints"]), 2)

    def test_mixed_split_into_segments(self):
        plan = Plan(steps=[
            Step(place=self._place("桌子")),
            Step(kind="speak", text="忠诚"),
            Step(place=self._place("冰箱"), dwell_sec=10),
            Step(kind="turn", angle_deg=180),
        ])
        segments = compile_segments(plan)
        self.assertEqual([seg["type"] for seg in segments],
                         ["mission", "speak", "mission", "turn"])
        self.assertFalse(is_pure_mission(segments))
        self.assertAlmostEqual(segments[2]["waypoints"][0]["dwell_sec"], 10.0)

    def test_describe(self):
        plan = Plan(steps=[
            Step(place=self._place("桌子")),
            Step(kind="turn", angle_deg=90),
        ])
        text = describe_segments(compile_segments(plan))
        self.assertIn("桌子", text)
        self.assertIn("90", text)


if __name__ == "__main__":
    unittest.main()
