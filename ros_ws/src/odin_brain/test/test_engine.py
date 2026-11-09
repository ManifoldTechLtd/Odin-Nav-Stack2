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

from odin_brain.engine import BrainEngine  # noqa: E402
from odin_brain.mock_nav import MockNav  # noqa: E402
from odin_brain.places import PlaceStore  # noqa: E402
from odin_brain.planner import RulePlanner  # noqa: E402


def _store(tmpdir):
    path = os.path.join(tmpdir, "places.json")
    data = {
        "schema_version": 1,
        "map_id": "map_opt",
        "places": [
            {"id": "p.toilet", "name": "厕所", "aliases": ["卫生间"],
             "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
            {"id": "p.desk", "name": "小A工位", "aliases": ["小A"],
             "pose": {"x": 3.0, "y": 4.0, "yaw": 0.0}},
        ],
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False)
    return PlaceStore(path)


class TestEngine(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.store = _store(self.tmpdir)
        self.nav = MockNav(per_step_sec=0.01)
        self.engine = BrainEngine(self.store, RulePlanner(self.store), None, self.nav,
                                  dry_run=False, verbose=False)

    def test_navigate_executes(self):
        result = self.engine.handle("去厕所")
        self.assertEqual(result.intent, "navigate")
        self.assertTrue(result.ok)
        self.assertEqual(result.state, "COMPLETED")
        self.assertEqual(result.reply, "")

    def test_navigate_dry_run(self):
        self.engine.dry_run = True
        self.engine.executor.dry_run = True
        result = self.engine.handle("去厕所")
        self.assertEqual(result.intent, "navigate")
        self.assertEqual(result.state, "DRY_RUN")

    def test_navigate_unknown_place(self):
        result = self.engine.handle("去食堂")
        self.assertEqual(result.intent, "navigate")
        self.assertFalse(result.ok)
        self.assertIn("食堂", result.reply)

    def test_query_reports_idle(self):
        result = self.engine.handle("还有多远")
        self.assertEqual(result.intent, "query")
        self.assertIn(result.state, ("IDLE", "UNKNOWN"))
        self.assertTrue(result.reply)

    def test_control_pause(self):
        self.nav.start_mission
        result = self.engine.handle("停一下")
        self.assertEqual(result.intent, "control")
        self.assertTrue(result.ok)
        self.assertEqual(self.nav.snapshot()["state"], "PAUSED")

    def test_control_return_home(self):
        result = self.engine.handle("回来")
        self.assertEqual(result.intent, "control")
        self.assertTrue(result.ok)

    def test_other_intent(self):
        result = self.engine.handle("帮我拿快递")
        self.assertEqual(result.intent, "other")
        self.assertFalse(result.ok)
        self.assertIn("做不了", result.reply)

    def test_chat_without_llm(self):
        result = self.engine.handle("你是谁")
        self.assertEqual(result.intent, "chat")
        self.assertTrue(result.reply)

    def test_session_records_history(self):
        self.engine.handle("你是谁")
        self.engine.handle("还有多远")
        self.assertGreaterEqual(self.engine.session.turns, 2)
        self.engine.handle("去厕所")
        self.assertEqual(self.engine.session.last_place, "厕所")

    def test_navigate_executes_no_confirm_reply(self):
        for text in ("去厕所", "去小A工位", "转180度"):
            result = self.engine.handle(text)
            self.assertTrue(result.ok, text)
            self.assertEqual(result.reply, "", text)


if __name__ == "__main__":
    unittest.main()
