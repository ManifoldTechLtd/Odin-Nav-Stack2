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

"""Unit tests for precheck validation, relative pose description, map_id verification, and health reporting."""

import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from odin_brain.engine import BrainEngine  # noqa: E402
from odin_brain.mock_nav import MockNav  # noqa: E402
from odin_brain.models import Place  # noqa: E402
from odin_brain.places import PlaceStore, describe_relative_pose  # noqa: E402
from odin_brain.planner import RulePlanner  # noqa: E402


def _make_store(tmpdir, map_id="map_opt", places=None):
    payload = {
        "schema_version": 1,
        "map_id": map_id,
        "frame_id": "map",
        "places": places or [
            {"id": "p.toilet", "name": "厕所", "aliases": ["卫生间"],
             "pose": {"x": 5.0, "y": 0.0, "yaw": 0.0}},
            {"id": "p.fridge", "name": "冰箱", "pose": {"x": 0.0, "y": 3.0, "yaw": 0.0}},
        ],
    }
    path = os.path.join(tmpdir, "places.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False)
    return PlaceStore(path)


def _make_engine(tmpdir, nav=None, map_id="map_opt", places=None):
    store = _make_store(tmpdir, map_id=map_id, places=places)
    nav = nav or MockNav(per_step_sec=0.01)
    engine = BrainEngine(store, RulePlanner(store), None, nav, dry_run=False, verbose=False)
    return engine, nav, store


class TestDescribeRelativePose(unittest.TestCase):
    def _place(self, x, y):
        return Place(id="p", name="厕所", x=x, y=y, yaw=0.0)

    def test_describe_relative_pose_directions(self):
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        self.assertIn("正前方", describe_relative_pose(pose, self._place(5.0, 0.0)))
        self.assertIn("左前方", describe_relative_pose(pose, self._place(3.0, 3.0)))
        self.assertIn("右前方", describe_relative_pose(pose, self._place(3.0, -3.0)))
        self.assertIn("正左方", describe_relative_pose(pose, self._place(0.0, 3.0)))
        self.assertIn("正右方", describe_relative_pose(pose, self._place(0.0, -3.0)))
        self.assertIn("正后方", describe_relative_pose(pose, self._place(-5.0, 0.0)))
        self.assertIn("身边", describe_relative_pose(pose, self._place(0.2, 0.1)))
        self.assertIn("5.0 米", describe_relative_pose(pose, self._place(5.0, 0.0)))

    def test_direction_respects_robot_heading(self):
        pose = {"x": 0.0, "y": 0.0, "yaw": math.pi / 2}
        text = describe_relative_pose(pose, self._place(5.0, 0.0))
        self.assertIn("右", text)


class TestA2ExplainThenNavigate(unittest.TestCase):
    def test_explain_then_navigate_a2(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir)
            nav._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            result = engine.handle("厕所在哪，带我去")
            self.assertTrue(result.ok, result.reply)
            self.assertIn("厕所", result.reply)
            self.assertIn("米处", result.reply)
            self.assertEqual(result.intent, "navigate")


class TestPrecheckBlocksOccupiedGoal(unittest.TestCase):
    def test_precheck_blocks_occupied_goal(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir)
            nav.occupied_boxes = [(4.0, -1.0, 6.0, 1.0)]
            result = engine.handle("去厕所")
            self.assertFalse(result.ok, result.reply)
            self.assertEqual(result.state, "PRECHECK_FAILED")
            self.assertIn("不可行", result.reply)
            self.assertIn("GOAL_OCCUPIED", result.reply)


class TestMoveRelStepback(unittest.TestCase):
    def test_move_rel_stepback_on_obstacle(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir)
            nav._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            nav.occupied_boxes = [(1.2, -0.5, 2.5, 0.5)]
            result = engine.handle("前进2米")
            self.assertTrue(result.ok, result.reply)
            self.assertAlmostEqual(float(nav._pose["x"]), 0.0)
            self.assertEqual(result.intent, "navigate")

    def test_move_rel_rejects_when_fully_blocked(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir)
            nav._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            nav.occupied_boxes = [(0.4, -0.5, 2.5, 0.5)]
            result = engine.handle("前进2米")
            self.assertFalse(result.ok, result.reply)
            self.assertEqual(result.state, "PRECHECK_FAILED")
            self.assertIn("有障碍物", result.reply)


class TestMapIdMismatch(unittest.TestCase):
    def test_map_id_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, map_id="map_opt")
            nav.runtime_map_id = "floor2"
            result = engine.handle("去厕所")
            self.assertFalse(result.ok, result.reply)
            self.assertIn("地图不匹配", result.reply)
            self.assertIn("floor2", result.reply)

    def test_map_id_match_passes(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, map_id="map_opt")
            nav.runtime_map_id = "map_opt"
            result = engine.handle("去厕所")
            self.assertTrue(result.ok, result.reply)


class TestHealthExplain(unittest.TestCase):
    def test_health_reason_in_query(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir)
            nav._state = "FAILED"
            nav.health_state = {
                "all_ok": False,
                "bad": [{"name": "neupan", "status": "ERROR",
                         "cause": "节点进程退出（rc=134）", "action": "已由看护脚本重启"}],
                "note": "1 个节点异常",
            }
            result = engine.handle("怎么停了")
            self.assertIn("neupan", result.reply)
            self.assertIn("原因", result.reply)
            self.assertIn("建议", result.reply)


class TestPlaceStoreHelpers(unittest.TestCase):
    def test_hotwords_dedup(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            store = _make_store(tmpdir)
            words = store.hotwords()
            self.assertIn("厕所", words)
            self.assertIn("卫生间", words)
            self.assertEqual(len(words), len(set(words)))

    def test_check_map_id_empty_runtime_skipped(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            store = _make_store(tmpdir, map_id="map_opt")
            self.assertEqual(store.check_map_id("")[0], True)
            self.assertEqual(store.check_map_id("map_opt")[0], True)
            ok, message = store.check_map_id("floor2")
            self.assertFalse(ok)
            self.assertIn("地图不匹配", message)


if __name__ == "__main__":
    unittest.main()
