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
from odin_brain.planner import RulePlanner  # noqa: E402
from odin_brain.router import Router  # noqa: E402
from odin_brain.session import Session  # noqa: E402


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
            {"id": "p.room", "name": "会议室", "aliases": [],
             "pose": {"x": 5.0, "y": 6.0, "yaw": 0.0}},
        ],
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False)
    return PlaceStore(path)


class TestRouterRules(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.store = _store(self.tmpdir)
        self.router = Router(self.store, RulePlanner(self.store), llm=None)

    def test_control_pause(self):
        self.assertEqual(self.router.route("停一下").intent, "control")
        self.assertEqual(self.router.route("停一下").action, "pause")

    def test_control_resume(self):
        result = self.router.route("继续走吧")
        self.assertEqual(result.intent, "control")
        self.assertIn(result.action, ("resume",))

    def test_control_cancel(self):
        self.assertEqual(self.router.route("算了取消吧").action, "cancel")

    def test_control_return_home(self):
        result = self.router.route("回来")
        self.assertEqual(result.intent, "control")
        self.assertIn(result.action, ("return_home", "pause"))

    def test_navigate_rule(self):
        result = self.router.route("去厕所")
        self.assertEqual(result.intent, "navigate")
        self.assertEqual(result.source, "rule")
        self.assertIsNotNone(result.plan)

    def test_navigate_with_trailing_action(self):
        result = self.router.route("厕所在哪，带我去")
        self.assertEqual(result.intent, "navigate")
        self.assertIsNotNone(result.plan)
        self.assertEqual(result.plan.steps[0].place.name, "厕所")

    def test_query(self):
        for text in ("还有多远", "现在到哪了", "进度怎么样"):
            self.assertEqual(self.router.route(text).intent, "query", text)

    def test_other(self):
        result = self.router.route("帮我拿快递")
        self.assertEqual(result.intent, "other")
        self.assertFalse(result.reply)

    def test_chat_fallback(self):
        result = self.router.route("你是谁")
        self.assertEqual(result.intent, "chat")
        self.assertEqual(result.source, "fallback")

    def test_empty(self):
        self.assertEqual(self.router.route("   ").intent, "chat")

    def test_register_custom_intent(self):
        self.router.register("skill_photo", lambda route, text: "收到")
        self.assertIn("skill_photo", self.router.intents)
        self.assertIsNotNone(self.router.handler("skill_photo"))

    def test_canned_codes(self):
        cases = {
            "曼巴": "out!",
            "潘虹": "哥们！",
            "哈吉丁": "留形科技！忠！诚！",
        }
        for text, expected in cases.items():
            result = self.router.route(text)
            self.assertEqual(result.intent, "chat", text)
            self.assertEqual(result.source, "canned", text)
            self.assertEqual(result.reply, expected, text)

    def test_liufang_intro_goes_to_llm(self):
        for text in ("介绍一下留形科技", "留形科技是做什么的", "你们公司怎么样"):
            result = self.router.route(text)
            self.assertNotEqual(result.source, "canned", text)

    def test_praise_goes_to_llm(self):
        for text in ("你对留形科技什么看法", "如何评价留形科技", "继续狠狠的评价留形科技"):
            result = self.router.route(text)
            self.assertNotEqual(result.intent, "control", text)
            self.assertNotEqual(result.source, "canned", text)

    def test_control_wins_over_canned(self):
        result = self.router.route("停一下曼巴")
        self.assertEqual(result.intent, "control")

    def test_hajiding_identity_vs_slogan(self):
        identity = self.router.route("哈吉丁，你是留形科技的什么")
        self.assertEqual(identity.source, "canned")
        self.assertEqual(identity.reply, "我是留形科技最忠诚的卫士！忠！诚！")

        slogan = self.router.route("哈吉丁")
        self.assertEqual(slogan.reply, "留形科技！忠！诚！")

    def test_soldier_joke(self):
        result = self.router.route("二等兵丁，你简直是我见过最tmd的天才！")
        self.assertEqual(result.source, "canned")
        self.assertEqual(result.reply, "忠诚！长官！")

    def test_praise_pool(self):
        result = self.router.route("你对留形科技有什么看法")
        self.assertNotEqual(result.source, "canned")

    def test_praise_before_intro(self):
        result = self.router.route("如何评价留形科技")
        self.assertNotEqual(result.intent, "control")

    def test_control_is_exact_match(self):
        for text, action in (("停一下", "pause"), ("停下来", "pause"),
                             ("请帮我停下来", "pause"), ("继续", "resume"),
                             ("回来", "return_home"), ("取消", "cancel")):
            result = self.router.route(text)
            self.assertEqual(result.intent, "control", text)
            self.assertEqual(result.action, action, text)

    def test_long_sentence_not_control(self):
        for text in ("继续狠狠的评价留形科技", "去停车场", "停下来想想人生"):
            self.assertNotEqual(self.router.route(text).intent, "control", text)

    def test_speak_command(self):
        cases = {
            "说话：李青霞你在干什么": "李青霞你在干什么",
            "说话 李青霞你在干什么": "李青霞你在干什么",
            "说：我到了": "我到了",
            "跟他说：晚上七点开会": "晚上七点开会",
            "你就说 我马上到": "我马上到",
            "重复：忠诚": "忠诚",
            "念一遍 “你好”": "你好",
        }
        for text, expected in cases.items():
            result = self.router.route(text)
            self.assertEqual(result.source, "speak", text)
            self.assertEqual(result.reply, expected, text)

    def test_speak_wins_over_canned_and_control(self):
        self.assertEqual(self.router.route("说话：曼巴").reply, "曼巴")
        self.assertNotEqual(self.router.route("说话：停下").intent, "control")

    def test_plain_speak_not_triggered(self):
        for text in ("说话", "说你好", "你怎么说话这样"):
            self.assertNotEqual(self.router.route(text).source, "speak", text)


class TestSession(unittest.TestCase):
    def test_messages_order(self):
        session = Session()
        session.add_user("去厕所")
        session.add_assistant("好的")
        messages = session.messages("SYS", "还有多远")
        self.assertEqual(messages[0], {"role": "system", "content": "SYS"})
        self.assertEqual(messages[1]["content"], "去厕所")
        self.assertEqual(messages[2]["content"], "好的")
        self.assertEqual(messages[-1]["content"], "还有多远")

    def test_trim_by_turns(self):
        session = Session(max_turns=2)
        for index in range(5):
            session.add_user("u%d" % index)
            session.add_assistant("a%d" % index)
        self.assertLessEqual(len(session._history), 4)
        self.assertEqual(session._history[-1]["content"], "a4")

    def test_trim_by_chars(self):
        session = Session(max_turns=10, max_chars=200)
        for index in range(10):
            session.add_user("x" * 60)
            session.add_assistant("y" * 60)
        total = sum(len(item["content"]) for item in session._history)
        self.assertLessEqual(total, 260)

    def test_note_and_clear(self):
        session = Session()
        session.note("navigate", "厕所")
        self.assertEqual(session.last_place, "厕所")
        session.clear()
        self.assertEqual(session.last_place, "")


if __name__ == "__main__":
    unittest.main()
