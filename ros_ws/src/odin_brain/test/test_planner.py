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

from odin_brain.places import PlaceStore  # noqa: E402
from odin_brain.planner import PlanError, RulePlanner, cn_to_number  # noqa: E402


class TestCnNumber(unittest.TestCase):
    def test_values(self):
        cases = {
            "五": 5, "十": 10, "十五": 15, "二十": 20, "二十五": 25,
            "两": 2, "3": 3, "3.5": 3.5,
        }
        for token, expected in cases.items():
            self.assertAlmostEqual(cn_to_number(token), expected)

    def test_invalid(self):
        self.assertIsNone(cn_to_number("abc"))


class TestRulePlanner(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.path = os.path.join(self.tmpdir, "places.json")
        data = {
            "schema_version": 1,
            "map_id": "map_opt",
            "places": [
                {"id": "p.toilet", "name": "厕所", "aliases": ["卫生间", "WC"],
                 "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
                {"id": "p.desk", "name": "小A工位", "aliases": ["小A"],
                 "pose": {"x": 3.0, "y": 4.0, "yaw": 0.5}, "dwell_default_sec": 2},
                {"id": "p.room", "name": "会议室", "aliases": [],
                 "pose": {"x": 5.0, "y": 6.0, "yaw": -0.7}},
            ],
        }
        with open(self.path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False)
        self.store = PlaceStore(self.path)
        self.planner = RulePlanner(self.store)

    def test_single_goto(self):
        plan = self.planner.plan("去厕所")
        self.assertEqual(len(plan.steps), 1)
        self.assertEqual(plan.steps[0].place.name, "厕所")
        self.assertAlmostEqual(plan.steps[0].dwell_sec, 0.0)

    def test_two_steps_with_dwell(self):
        plan = self.planner.plan("先去厕所待五秒，然后再去小A工位")
        self.assertEqual(len(plan.steps), 2)
        self.assertEqual(plan.steps[0].place.name, "厕所")
        self.assertAlmostEqual(plan.steps[0].dwell_sec, 5.0)
        self.assertEqual(plan.steps[1].place.name, "小A工位")
        self.assertAlmostEqual(plan.steps[1].dwell_sec, 2.0)

    def test_dwell_in_separate_segment(self):
        plan = self.planner.plan("去厕所，等3秒，再去小A工位")
        self.assertEqual(len(plan.steps), 2)
        self.assertAlmostEqual(plan.steps[0].dwell_sec, 3.0)

    def test_dwell_default_unit_seconds(self):
        plan = self.planner.plan("去厕所停留10")
        self.assertAlmostEqual(plan.steps[0].dwell_sec, 10.0)

    def test_alias_and_verb_prefix(self):
        plan = self.planner.plan("麻烦带我去卫生间")
        self.assertEqual(plan.steps[0].place.name, "厕所")

    def test_home_step(self):
        plan = self.planner.plan("回起点")
        self.assertEqual(len(plan.steps), 1)
        self.assertEqual(plan.steps[0].kind, "home")

    def test_goto_then_home(self):
        plan = self.planner.plan("去会议室，然后回来")
        self.assertEqual(len(plan.steps), 2)
        self.assertEqual(plan.steps[0].place.name, "会议室")
        self.assertEqual(plan.steps[1].kind, "home")

    def test_waypoints_payload(self):
        plan = self.planner.plan("去厕所待五秒")
        waypoints = plan.waypoints()
        self.assertEqual(len(waypoints), 1)
        self.assertAlmostEqual(waypoints[0]["x"], 1.0)
        self.assertAlmostEqual(waypoints[0]["y"], 2.0)
        self.assertAlmostEqual(waypoints[0]["dwell_sec"], 5.0)
        self.assertEqual(waypoints[0]["name"], "厕所")

    def test_unknown_place(self):
        with self.assertRaises(PlanError) as ctx:
            self.planner.plan("去食堂")
        self.assertIn("食堂", str(ctx.exception))
        self.assertTrue(ctx.exception.unknown)

    def test_empty_input(self):
        with self.assertRaises(PlanError):
            self.planner.plan("   ")

    def test_max_steps(self):
        planner = RulePlanner(self.store, max_steps=1)
        with self.assertRaises(PlanError):
            planner.plan("去厕所，再去会议室")


if __name__ == "__main__":
    unittest.main()
