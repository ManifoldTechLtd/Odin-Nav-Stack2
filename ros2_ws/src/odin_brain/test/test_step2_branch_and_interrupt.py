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

"""Unit tests for conditional branches, fast/slow channels, temporary preemption, and event bus."""

import json
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from odin_brain.engine import BrainEngine  # noqa: E402
from odin_brain.event_bus import EventBus  # noqa: E402
from odin_brain.mock_nav import MockNav  # noqa: E402
from odin_brain.models import Plan, Step  # noqa: E402
from odin_brain.places import PlaceStore  # noqa: E402
from odin_brain.planner import RulePlanner  # noqa: E402

CASE_A = ("去桌子靠窗那边，到达之后说话：我带的东西到达请查收。"
          "如果你听到有人说\"好的\"或者\"我拿到了\"之后你就去冰箱，"
          "如果等待20秒没有人给你对应的反馈就直接去小A工位，说\"货已送达无人领取已返回。\"")


def _wait_until(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def _make_engine(tmpdir, per_step=0.3):
    path = os.path.join(tmpdir, "places.json")
    payload = {
        "schema_version": 1,
        "map_id": "map_opt",
        "places": [
            {"id": "p.desk", "name": "桌子", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
            {"id": "p.fridge", "name": "冰箱", "pose": {"x": 5.0, "y": 6.0, "yaw": 0.5}},
            {"id": "p.seat", "name": "小A工位", "pose": {"x": 3.0, "y": 4.0, "yaw": 1.0}},
            {"id": "p.room", "name": "会议室", "pose": {"x": 7.0, "y": 1.0, "yaw": 0.0}},
        ],
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False)
    store = PlaceStore(path)
    nav = MockNav(per_step_sec=per_step)
    engine = BrainEngine(store, RulePlanner(store), None, nav, dry_run=False, verbose=False)
    return engine, nav, store


class TestEventBus(unittest.TestCase):
    def test_wait_wakes_on_publish(self):
        bus = EventBus()
        threading.Timer(0.15, lambda: bus.publish("ASR", {"text": "好的"})).start()
        started = time.time()
        event = bus.wait(["ASR"], timeout_sec=2.0)
        self.assertIsNotNone(event)
        self.assertEqual(event.payload["text"], "好的")
        self.assertLess(time.time() - started, 1.0)

    def test_wait_returns_none_on_timeout(self):
        bus = EventBus()
        started = time.time()
        self.assertIsNone(bus.wait(["ASR"], timeout_sec=0.2))
        self.assertGreaterEqual(time.time() - started, 0.18)

    def test_interrupt_slot_and_listening_flag(self):
        bus = EventBus()
        bus.request_interrupt("cancel")
        self.assertTrue(bus.has_interrupt())
        action, payload = bus.pop_interrupt()
        self.assertEqual(action, "cancel")
        self.assertIsNone(payload)
        self.assertFalse(bus.has_interrupt())

        bus.set_listening(True)
        self.assertTrue(bus.is_listening())
        bus.set_listening(False)
        self.assertFalse(bus.is_listening())

    def test_clear_asr_keeps_other_events(self):
        bus = EventBus()
        bus.publish("ASR", {"text": "旧话"})
        bus.publish("MISSION_EVENT", {"event": "X"})
        bus.clear_asr()
        self.assertIsNone(bus.wait(["ASR"], timeout_sec=0.1))
        self.assertIsNotNone(bus.wait(["MISSION_EVENT"], timeout_sec=0.1))


class TestCaseABranch(unittest.TestCase):
    def test_case_a_match_branch(self):
        """Inject matching ASR response during ask_and_wait window -> navigate to fridge."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, per_step=0.05)

            plan = engine.planner.plan(CASE_A)
            kinds = [step.kind for step in plan.steps]
            self.assertEqual(kinds, ["goto", "ask_and_wait"], kinds)
            self.assertEqual(plan.steps[0].modifier, "靠窗")
            self.assertIn("请查收", plan.steps[1].text)
            self.assertEqual(plan.steps[1].expect_keywords, ["好的", "我拿到了"])

            engine.handle_async(CASE_A)
            self.assertTrue(_wait_until(engine.event_bus.is_listening, 10.0),
                            "Did not enter ask_and_wait listening window")
            engine.event_bus.publish("ASR", {"text": "好的，我拿到了"})
            self.assertTrue(_wait_until(lambda: not engine.busy, 15.0), "Task did not finish")

            self.assertEqual(nav.missions[-1], ["冰箱"])

    def test_case_a_timeout_branch(self):
        """Timeout without ASR response -> navigate to desk and speak timeout message."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, per_step=0.05)
            spoken = []
            plan = engine.planner.plan(CASE_A)
            plan.steps[-1].timeout_sec = 0.15

            result = engine.executor.run(plan, on_speak=spoken.append)
            self.assertTrue(result.ok, result.message)
            self.assertEqual(nav.missions[-1], ["小A工位"])
            self.assertTrue(any("货已送达" in text for text in spoken), spoken)
            self.assertTrue(any("请查收" in text for text in spoken), spoken)

    def test_ask_and_wait_empty_match_branch(self):
        """Keyword match with empty on_match branch completes immediately."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, store = _make_engine(tmpdir, per_step=0.05)
            fridge = store.get("冰箱")
            plan = Plan(steps=[Step(kind="ask_and_wait", text="到了吗",
                                    expect_keywords=["到了"], timeout_sec=2.0,
                                    on_match=[], on_timeout=[Step(place=fridge)])])
            threading.Timer(0.2, lambda: engine.event_bus.publish("ASR", {"text": "到了"})).start()
            result = engine.executor.run(plan)
            self.assertTrue(result.ok)
            self.assertEqual(result.state, "COMPLETED")
            self.assertEqual(nav.missions, [])


class TestFastChannel(unittest.TestCase):
    def test_mid_navigation_cancel_and_query(self):
        """Fast channel answers queries immediately and cancels active navigation within 0.5s."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, per_step=0.8)
            engine.handle_async("去冰箱")
            self.assertTrue(_wait_until(lambda: nav.snapshot().get("state") == "NAVIGATING", 5.0))

            started = time.time()
            query = engine.handle_fast("还有多远")
            query_ms = time.time() - started
            self.assertIsNotNone(query)
            self.assertLess(query_ms, 0.5, "Fast channel query timed out %.3fs" % query_ms)
            self.assertIn("第", query.reply)

            started = time.time()
            cancel = engine.handle_fast("取消")
            cancel_ms = time.time() - started
            self.assertIsNotNone(cancel)
            self.assertLess(cancel_ms, 0.5, "Fast channel cancel timed out %.3fs" % cancel_ms)

            self.assertTrue(_wait_until(lambda: nav.snapshot().get("state") == "CANCELLED", 3.0))
            self.assertTrue(_wait_until(lambda: not engine.busy, 5.0))

    def test_fast_channel_treats_text_as_asr(self):
        """Plain text on the fast channel is published to the event bus as an ASR event."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, _nav, _store = _make_engine(tmpdir)
            result = engine.handle_fast("我拿到了")
            self.assertIsNotNone(result)
            event = engine.event_bus.wait(["ASR"], timeout_sec=0.5)
            self.assertIsNotNone(event)
            self.assertEqual(event.payload["text"], "我拿到了")


class TestTemporaryPreempt(unittest.TestCase):
    def test_temporary_mission_preempt_and_restore(self):
        """Temporary mission preemption pauses the main mission, runs the temporary mission, and resumes the main mission."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, per_step=0.45)
            engine.handle_async("去冰箱")
            self.assertTrue(_wait_until(lambda: nav.snapshot().get("state") == "NAVIGATING", 5.0))
            main_id = nav.snapshot().get("mission_id")

            reply = engine.handle_fast("先去桌子")
            self.assertIsNotNone(reply)
            self.assertIn("桌子", reply.reply)

            self.assertTrue(_wait_until(lambda: len(nav.missions) >= 2, 8.0), "Temporary mission not dispatched")
            self.assertEqual(nav.missions[0], ["冰箱"])
            self.assertEqual(nav.missions[1], ["桌子"])

            self.assertTrue(_wait_until(lambda: not engine.busy, 25.0), "Main mission did not finish")
            self.assertEqual(nav.snapshot().get("state"), "COMPLETED")
            self.assertEqual(nav.snapshot().get("mission_id"), main_id)


class TestSpeakerHalfDuplex(unittest.TestCase):
    def test_is_speaking_false_when_disabled(self):
        from odin_brain.speech import Speaker
        os.environ.pop("ODIN_TTS_URL", None)
        speaker = Speaker(enabled=False)
        self.assertFalse(speaker.is_speaking)


class TestStaleMissionIdGracePeriod(unittest.TestCase):
    def test_wait_with_interrupts_ignores_previous_completed_mission(self):
        """Executor ignores stale COMPLETED status from a previous mission_id while waiting for a newly started mission."""
        with tempfile.TemporaryDirectory() as tmpdir:
            engine, nav, _store = _make_engine(tmpdir, per_step=0.15)
            r1 = engine.handle("去桌子")
            self.assertTrue(r1.ok)
            old_id = r1.mission_id

            orig_snapshot = nav.snapshot
            delay_until = [0.0]
            new_id_holder = [""]

            orig_start = nav.start_mission

            def delayed_start(plan, loop=False, temporary=False):
                ok, msg, mid = orig_start(plan, loop=loop, temporary=temporary)
                new_id_holder[0] = mid
                delay_until[0] = time.time() + 0.2
                return ok, msg, mid

            def delayed_snapshot():
                if time.time() < delay_until[0]:
                    return {
                        "ok": True, "state": "COMPLETED", "mission_id": old_id,
                        "idx": 0, "total": 1, "dist_m": 0.0, "progress": 100.0,
                    }
                return orig_snapshot()

            nav.start_mission = delayed_start
            nav.snapshot = delayed_snapshot

            started = time.time()
            r2 = engine.handle("去冰箱")
            elapsed = time.time() - started
            self.assertTrue(r2.ok)
            self.assertEqual(r2.mission_id, new_id_holder[0])
            self.assertGreaterEqual(elapsed, 0.18)


if __name__ == "__main__":
    unittest.main()
