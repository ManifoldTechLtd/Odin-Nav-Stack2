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

"""Command-line interface for odin_brain (interactive dialogue and navigation).

Usage:
    rosrun odin_brain brain
    rosrun odin_brain brain --dry-run
    rosrun odin_brain brain --mock
    rosrun odin_brain brain --once "去厕所"
"""

import argparse
import os
import sys
from typing import List, Optional

from .engine import BrainEngine
from .llm_client import build_client
from .places import PlaceStore
from .planner import RulePlanner
from .speech import Speaker
from .audio_in import AudioListener

DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base_link"


def default_places_path() -> str:
    """Resolve places.json path: $ODIN_BRAIN_PLACES -> ~/.config/odin_brain/places.json -> package config/places.json."""
    env = os.environ.get("ODIN_BRAIN_PLACES", "")
    if env:
        return os.path.expanduser(env)
    user_path = os.path.expanduser("~/.config/odin_brain/places.json")
    if os.path.isfile(user_path):
        return user_path
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "config", "places.json"))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="brain",
        description="Natural-language dialogue and waypoint navigation CLI",
    )
    parser.add_argument("--places", default="", help="Path to places.json")
    parser.add_argument("--dry-run", action="store_true", help="Parse and preview without dispatching missions")
    parser.add_argument("--mock", action="store_true", help="Use MockNav without ROS")
    parser.add_argument("--once", default="", help="Execute a single instruction and exit")
    parser.add_argument("--no-llm", action="store_true", help="Disable LLM and use rule-based parser only")
    parser.add_argument("--no-stream", action="store_true", help="Disable streaming output for chat")
    parser.add_argument("--loop", action="store_true", help="Loop mission execution (mission_manager loop=true)")
    parser.add_argument("--map-frame", default=DEFAULT_MAP_FRAME)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--quiet", action="store_true", help="Suppress per-waypoint progress logs")
    parser.add_argument("--no-tts", action="store_true", help="Disable TTS voice output")
    return parser


class BrainCLI(object):
    def __init__(self, args):
        self.args = args
        self.places_path = args.places or default_places_path()
        self.store = PlaceStore(self.places_path)
        self.planner = RulePlanner(self.store)
        self.llm = None if args.no_llm else build_client()
        self.dry_run = bool(args.dry_run)
        self.stream = not args.no_stream
        self.verbose = not args.quiet
        self.nav = None
        self.engine = None
        self.speaker = None
        self._rospy = None
        self._speaking = False
        self._printed_any = False
        self._reply_stream = sys.stdout

    def setup(self) -> None:
        print("[info] Places: %s (map_id=%s, %d places)" % (
            self.places_path, self.store.map_id or "-", len(self.store.places)))
        if not self.store.places:
            print("[warn] Place store is empty: record or add places with brain_places first")
        if self.llm is None:
            print("[info] Parser: rule-based (--no-llm)")
        elif self.llm.available:
            print("[info] Parser: LLM-first (%s), with rule fallback" % self.llm.config.describe())
        else:
            print("[info] Parser: rule-based (%s)" % self.llm.config.describe())

        if self.args.mock:
            from .mock_nav import MockNav
            self.nav = MockNav()
            print("[info] Nav backend: Mock")
        else:
            import rospy
            from .ros1_nav import Ros1Nav
            self._rospy = rospy
            if not rospy.core.is_initialized():
                rospy.init_node("odin_brain_cli", anonymous=True, disable_signals=True)
            self.nav = Ros1Nav(map_frame=self.args.map_frame, base_frame=self.args.base_frame)
            ready, message = self.nav.wait_ready(timeout_sec=8.0)
            if ready:
                print("[info] Nav backend: ROS1 (ready)")
            else:
                print("[warn] Nav backend: ROS1 (%s)" % message)
                print("[warn] Use --dry-run to preview, or ensure mission_manager is running.")
            print("[info] Motion switch: %s" % (
                "DRY-RUN (no dispatch)" if self.dry_run else "ENABLED"))

        self.engine = BrainEngine(self.store, self.planner, self.llm, self.nav,
                                  dry_run=self.dry_run, verbose=self.verbose,
                                  loop=self.args.loop)

        self.speaker = Speaker(enabled=not self.args.no_tts)
        print("[info] TTS: %s" % self.speaker.describe())
        if self.engine is not None:
            self.engine.on_speak = self.speaker.speak
            self.engine.executor.speaker = self.speaker

        self.mic = AudioListener(store=self.store, speaker=self.speaker,
                                 event_bus=self.engine.event_bus if self.engine else None,
                                 on_text=self._on_voice_text, on_wake=self._on_wake,
                                 verbose=self.verbose)
        if not self.args.once or ("如果" in self.args.once or "若" in self.args.once):
            self.mic.start()

    def _on_wake(self, word: str) -> None:
        """Respond when only the wake word was spoken."""
        reply = "我在，请说"
        self._say(reply)
        if self.speaker is not None:
            self.speaker.speak(reply)

    def _on_voice_text(self, text: str) -> None:
        """Execute a voice command recognized outside an ask_and_wait listening window."""
        print("[mic ] Voice command: %s" % text)
        if self.engine is None:
            return
        if self.engine.busy:
            fast = self.engine.handle_fast(text)
            if fast is not None and fast.reply:
                self._say(fast.reply)
                if self.speaker is not None:
                    self.speaker.speak(fast.reply)
            return
        self.engine.handle_async(text, on_token=self._on_token if self.stream else None,
                                 on_done=self._on_task_done)

    def teardown(self) -> None:
        if getattr(self, "mic", None) is not None:
            self.mic.stop()
        if self.engine is not None and self.engine.busy:
            self.engine._task_thread.join(timeout=30.0)
        if self.speaker is not None:
            self.speaker.wait_idle(timeout_sec=20.0)
            self.speaker.stop()
        if self._rospy is not None:
            try:
                self._rospy.signal_shutdown("cli exit")
            except Exception:
                pass

    def _say(self, text: str) -> None:
        if not text:
            return
        print("[odin] %s" % text, file=self._reply_stream, flush=True)

    def _on_token(self, delta: str) -> None:
        self._printed_any = True
        if not self._speaking:
            print("[odin] ", end="", file=self._reply_stream, flush=True)
            self._speaking = True
        print(delta, end="", file=self._reply_stream, flush=True)

    def _end_stream(self) -> None:
        if self._speaking:
            print("")
            self._speaking = False

    def handle_once(self, text: str) -> int:
        """Execute a single instruction synchronously (--once mode)."""
        if self.speaker is not None:
            self.speaker.stop()
        result = self.engine.handle(
            text, on_token=self._on_token if self.stream else None)
        self._on_task_done(result)
        return 0 if result.ok else 2

    def handle(self, text: str) -> int:
        if self.engine is not None and self.engine.busy:
            fast = self.engine.handle_fast(text)
            if fast is not None:
                if fast.reply:
                    self._say(fast.reply)
                    if self.speaker is not None:
                        self.speaker.speak(fast.reply)
                return 0 if fast.ok else 1
            return 0

        if self.speaker is not None:
            self.speaker.stop()
        self.engine.handle_async(text, on_token=self._on_token if self.stream else None,
                                 on_done=self._on_task_done)
        return 0

    def _on_task_done(self, result) -> None:
        """Finalize background task completion (print, speak, and extend wake window)."""
        printed = getattr(self, "_printed_any", False)
        self._printed_any = False
        self._end_stream()
        if result.reply and not printed:
            self._say(result.reply)
        if self.speaker is not None and result.reply:
            self.speaker.speak(result.reply)
        if self.verbose and result.intent in ("navigate", "control", "query", "other"):
            print("[route] intent=%s source=%s state=%s" % (
                result.intent, result.source, result.state or "-"))
        if self.mic is not None and hasattr(self.mic, "extend_wake_window"):
            self.mic.extend_wake_window()

    def loop(self) -> int:
        self.print_help()
        while True:
            try:
                line = input("odin> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("")
                return 0
            if not line:
                continue
            if line.startswith("/"):
                if self.handle_command(line):
                    return 0
                continue
            self.handle(line)

    def handle_command(self, line: str) -> bool:
        parts = line.split()
        command = parts[0].lower()
        if command in ("/quit", "/exit", "/q"):
            return True
        if command == "/help":
            self.print_help()
        elif command == "/places":
            self.print_places()
        elif command == "/status":
            print(self.nav.snapshot() if self.nav else "(no navigation backend)")
        elif command == "/history":
            text = self.engine.session.history_text()
            print(text or "(no conversation history)")
        elif command == "/dry":
            self.dry_run = not self.dry_run
            if self.engine:
                self.engine.dry_run = self.dry_run
                self.engine.executor.dry_run = self.dry_run
            print("[info] dry-run = %s" % self.dry_run)
        elif command == "/tts":
            if self.speaker is None:
                print("[info] Speaker not initialized")
            elif not self.speaker.url:
                print("[info] ODIN_TTS_URL not configured")
            else:
                self.speaker.enabled = not self.speaker.enabled
                if self.speaker.enabled and self.speaker._worker is None:
                    import threading as _th
                    self.speaker._worker = _th.Thread(target=self.speaker._loop, daemon=True)
                    self.speaker._worker.start()
                print("[info] TTS = %s" % ("on" if self.speaker.enabled else "off"))
        elif command == "/volume":
            if self.speaker is None:
                print("[info] Speaker not initialized")
            elif len(parts) < 2:
                print("[info] Volume: %d%% (usage: /volume 30)" % int(self.speaker.volume * 100))
            else:
                try:
                    value = self.speaker.set_volume(float(parts[1]))
                    print("[info] Volume set to %d%%" % int(value * 100))
                except ValueError:
                    print("[fail] Provide a number, e.g. /volume 40")
        elif command == "/voice":
            if self.speaker is None:
                print("[info] Speaker not initialized")
            elif len(parts) < 2:
                print("[info] Current voice: %s" % (self.speaker.voice or "default (zh-CN-XiaoxiaoNeural)"))
                print("       Options: zh-CN-XiaoxiaoNeural / zh-CN-YunxiNeural / zh-CN-YunjianNeural / zh-CN-XiaoyiNeural")
            else:
                self.speaker.voice = parts[1]
                print("[info] Voice switched to %s" % parts[1])
        elif command == "/record":
            self.record_place(parts[1:])
        else:
            print("Unknown command: %s (type /help)" % command)
        return False

    def record_place(self, args: List[str]) -> None:
        if not args:
            print("Usage: /record <name> [alias1,alias2]")
            return
        name = args[0]
        aliases = []
        if len(args) > 1:
            aliases = [item for item in args[1].replace("，", ",").split(",") if item]
        pose = self.nav.get_pose() if self.nav else None
        if pose is None:
            print("[fail] Pose unavailable (requires %s -> %s TF and active relocalization)" % (
                self.args.map_frame, self.args.base_frame))
            return
        place = self.store.upsert(name, pose["x"], pose["y"], pose["yaw"], aliases=aliases)
        self.store.save()
        print("[ok  ] Recorded %s: x=%.3f y=%.3f yaw=%.3f (%s)" % (
            place.name, place.x, place.y, place.yaw, self.places_path))

    def print_places(self) -> None:
        print("Places %s (map_id=%s)" % (self.places_path, self.store.map_id or "-"))
        for place in self.store.places:
            alias = ("  aliases: " + ", ".join(place.aliases)) if place.aliases else ""
            print("  - %-10s x=%8.3f y=%8.3f yaw=%6.3f%s" % (
                place.name, place.x, place.y, place.yaw, alias))

    @staticmethod
    def print_help() -> None:
        print("""
Natural-language examples:
  带我去厕所
  先去厕所待五秒，然后再去小A工位
  停一下 / 继续 / 取消 / 回来
  还有多远
  你是谁

Commands:
  /places                 List places
  /record <name> [alias]  Record current pose as a place (requires TF)
  /status                 Print navigation status
  /history                Show recent conversation
  /dry                    Toggle dry-run mode
  /tts                    Toggle TTS output
  /volume 30              Set TTS volume
  /voice <voice_name>     Switch TTS voice
  /help                   Show help
  /quit                   Exit
""".strip())


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    # Redirect diagnostic logs to stderr so stdout carries only assistant replies.
    reply_stream = sys.stdout
    sys.stdout = sys.stderr

    cli = BrainCLI(args)
    cli._reply_stream = reply_stream
    try:
        cli.setup()
        if args.once:
            return cli.handle_once(args.once)
        return cli.loop()
    finally:
        cli.teardown()


if __name__ == "__main__":
    sys.exit(main())
