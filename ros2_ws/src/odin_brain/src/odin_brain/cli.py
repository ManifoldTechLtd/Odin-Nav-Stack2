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

"""Command-line interface for odin_brain (natural-language dialog and navigation)."""

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
    """Resolve places database path: $ODIN_BRAIN_PLACES -> ~/.config/odin_brain/places.json -> package config/places.json."""
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
        description="Natural-language dialog and waypoint navigation CLI",
    )
    parser.add_argument("--places", default="", help="地点库 JSON 路径")
    parser.add_argument("--dry-run", action="store_true", help="只解析与预览，不下发任务")
    parser.add_argument("--mock", action="store_true", help="不连 ROS，使用模拟导航后端")
    parser.add_argument("--once", default="", help="执行一条指令后退出")
    parser.add_argument("--no-llm", action="store_true", help="禁用大模型（纯规则）")
    parser.add_argument("--no-stream", action="store_true", help="对话不做流式输出")
    parser.add_argument("--loop", action="store_true", help="任务循环执行（mission_manager loop=true）")
    parser.add_argument("--map-frame", default=DEFAULT_MAP_FRAME)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--quiet", action="store_true", help="不打印逐点进度")
    parser.add_argument("--no-tts", action="store_true", help="关闭语音播报（默认按 ODIN_TTS_URL 自动开）")
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

    def setup(self) -> None:
        print("[info] 地点库: %s（map_id=%s，%d 个地点）" % (
            self.places_path, self.store.map_id or "-", len(self.store.places)))
        if not self.store.places:
            print("[warn] 地点库为空：先用 brain_places record / add 标定地点，再发指令")
        if self.llm is None:
            print("[info] 解析器: 规则（已按 --no-llm 禁用大模型）")
        elif self.llm.available:
            print("[info] 解析器: 大模型优先（%s），失败自动退回规则" % self.llm.config.describe())
        else:
            print("[info] 解析器: 规则（%s）" % self.llm.config.describe())

        if self.args.mock:
            from .mock_nav import MockNav
            self.nav = MockNav()
            print("[info] 导航后端: Mock（不会真的运动）")
        else:
            import rospy
            from .ros1_nav import Ros1Nav
            self._rospy = rospy
            if not rospy.core.is_initialized():
                rospy.init_node("odin_brain_cli", anonymous=True, disable_signals=True)
            self.nav = Ros1Nav(map_frame=self.args.map_frame, base_frame=self.args.base_frame)
            ready, message = self.nav.wait_ready(timeout_sec=8.0)
            if ready:
                print("[info] 导航后端: ROS1（已就绪）")
            else:
                print("[warn] 导航后端: ROS1（%s）" % message)
                print("[warn] 可以先 --dry-run 预览；确认 mission_manager 已启动后再下发。")
            print("[info] 运动开关: %s" % (
                "DRY-RUN（不下发）" if self.dry_run else "ENABLED（会真实下发）"))

        self.engine = BrainEngine(self.store, self.planner, self.llm, self.nav,
                                  dry_run=self.dry_run, verbose=self.verbose,
                                  loop=self.args.loop)

        self.speaker = Speaker(enabled=not self.args.no_tts)
        print("[info] 语音播报: %s" % self.speaker.describe())
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
        reply = "我在，请说"
        self._say(reply)
        if self.speaker is not None:
            self.speaker.speak(reply)

    def _on_voice_text(self, text: str) -> None:
        print("[mic ] 语音命令：%s" % text)
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
        print("[odin] %s" % text)

    def _on_token(self, delta: str) -> None:
        if not self._speaking:
            print("[odin] ", end="", flush=True)
            self._speaking = True
        print(delta, end="", flush=True)

    def _end_stream(self) -> None:
        if self._speaking:
            print("")
            self._speaking = False

    def handle_once(self, text: str) -> int:
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
        self._end_stream()
        if result.intent != "chat" or not result.reply:
            self._say(result.reply)
        elif self.stream and result.reply:
            print("")
        if self.speaker is not None and result.reply:
            self.speaker.speak(result.reply)
        if self.verbose and result.intent in ("navigate", "control", "query", "other"):
            print("[route] intent=%s source=%s state=%s" % (
                result.intent, result.source, result.state or "-"))

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
            print(self.nav.snapshot() if self.nav else "（没有导航后端）")
        elif command == "/history":
            text = self.engine.session.history_text()
            print(text or "（还没有对话）")
        elif command == "/dry":
            self.dry_run = not self.dry_run
            if self.engine:
                self.engine.dry_run = self.dry_run
                self.engine.executor.dry_run = self.dry_run
            print("[info] dry-run = %s" % self.dry_run)
        elif command == "/tts":
            if self.speaker is None:
                print("[info] 没有初始化播报器")
            elif not self.speaker.url:
                print("[info] 未配置 ODIN_TTS_URL，播报不可用")
            else:
                self.speaker.enabled = not self.speaker.enabled
                if self.speaker.enabled and self.speaker._worker is None:
                    import threading as _th
                    self.speaker._worker = _th.Thread(target=self.speaker._loop, daemon=True)
                    self.speaker._worker.start()
                print("[info] 语音播报 = %s" % ("开" if self.speaker.enabled else "关"))
        elif command == "/volume":
            if self.speaker is None:
                print("[info] 没有初始化播报器")
            elif len(parts) < 2:
                print("[info] 当前音量 %d%%（用法：/volume 30）" % int(self.speaker.volume * 100))
            else:
                try:
                    value = self.speaker.set_volume(float(parts[1]))
                    print("[info] 音量已设为 %d%%" % int(value * 100))
                except ValueError:
                    print("[fail] 请给一个数字，例如 /volume 40")
        elif command == "/voice":
            if self.speaker is None:
                print("[info] 没有初始化播报器")
            elif len(parts) < 2:
                print("[info] 当前音色：%s" % (self.speaker.voice or "默认（zh-CN-XiaoxiaoNeural）"))
                print("       可选：zh-CN-XiaoxiaoNeural / zh-CN-YunxiNeural / zh-CN-YunjianNeural / zh-CN-XiaoyiNeural")
            else:
                self.speaker.voice = parts[1]
                print("[info] 音色已切换为 %s（下一句生效）" % parts[1])
        elif command == "/record":
            self.record_place(parts[1:])
        else:
            print("未知命令：%s（/help 查看）" % command)
        return False

    def record_place(self, args: List[str]) -> None:
        if not args:
            print("用法：/record 名称 [别名1,别名2]")
            return
        name = args[0]
        aliases = []
        if len(args) > 1:
            aliases = [item for item in args[1].replace("，", ",").split(",") if item]
        pose = self.nav.get_pose() if self.nav else None
        if pose is None:
            print("[fail] 取不到位姿（需要 %s -> %s 的 TF，且已重定位）" % (
                self.args.map_frame, self.args.base_frame))
            return
        place = self.store.upsert(name, pose["x"], pose["y"], pose["yaw"], aliases=aliases)
        self.store.save()
        print("[ok  ] 已记录 %s: x=%.3f y=%.3f yaw=%.3f（%s）" % (
            place.name, place.x, place.y, place.yaw, self.places_path))

    def print_places(self) -> None:
        print("地点库 %s（map_id=%s）" % (self.places_path, self.store.map_id or "-"))
        for place in self.store.places:
            alias = ("  别名: " + "、".join(place.aliases)) if place.aliases else ""
            print("  - %-10s x=%8.3f y=%8.3f yaw=%6.3f%s" % (
                place.name, place.x, place.y, place.yaw, alias))

    @staticmethod
    def print_help() -> None:
        print("""
直接说话即可（会自动路由到对话/导航/控制/状态）：
  带我去厕所
  先去厕所待五秒，然后再去小A工位
  停一下 / 继续 / 取消 / 回来
  还有多远
  你是谁

命令：
  /places                 列出地点库
  /record 名称 [别名,..]   记录当前位置为地点（需要 TF）
  /status                 打印导航状态
  /history                查看最近对话
  /dry                    切换 dry-run
  /tts                    开/关语音播报
  /volume 30              调播报音量（当前值不带参数直接敲）
  /voice zh-CN-YunxiNeural  切换音色（不带参数看当前值）
  /help                   帮助
  /quit                   退出
""".strip())


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    cli = BrainCLI(args)
    try:
        cli.setup()
        if args.once:
            return cli.handle_once(args.once)
        return cli.loop()
    finally:
        cli.teardown()


if __name__ == "__main__":
    sys.exit(main())
