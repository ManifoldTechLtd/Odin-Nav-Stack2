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

"""Unit tests for wake-word detection and command extraction."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from odin_brain.audio_in import WAKE_WORDS_DEFAULT, extract_wake_command  # noqa: E402


class TestWakeWord(unittest.TestCase):
    def test_all_wake_words(self):
        for word in WAKE_WORDS_DEFAULT:
            hit, matched, command = extract_wake_command(word)
            self.assertTrue(hit, word)
            self.assertEqual(command, "", word)

    def test_wake_with_command(self):
        cases = {
            "奥丁去冰箱": ("奥丁", "去冰箱"),
            "奥丁，去冰箱": ("奥丁", "去冰箱"),
            "奥丁，帮我看看现在状态": ("奥丁", "帮我看看现在状态"),
            "丁子去冰箱": ("丁子", "去冰箱"),
            "哈吉丁去冰箱": ("哈吉丁", "去冰箱"),
        }
        for text, (word, command) in cases.items():
            hit, matched, rest = extract_wake_command(text)
            self.assertTrue(hit, text)
            self.assertEqual(matched, word, text)
            self.assertEqual(rest, command, text)

    def test_longest_word_first(self):
        hit, word, command = extract_wake_command("奥丁奥丁你好啊")
        self.assertTrue(hit)
        self.assertEqual(word, "奥丁奥丁")
        self.assertEqual(command, "你好啊")

    def test_wake_without_command(self):
        for text in ("奥丁", "奥丁奥丁", "丁子", "哈吉丁"):
            hit, _word, command = extract_wake_command(text)
            self.assertTrue(hit, text)
            self.assertEqual(command, "", text)

    def test_homophone_variants(self):
        for text in ("奥亭去冰箱", "奥顶去冰箱", "奥叮去冰箱", "熬丁去冰箱"):
            hit, _word, command = extract_wake_command(text)
            self.assertTrue(hit, text)
            self.assertEqual(command, "去冰箱", text)

    def test_no_wake_word_ignored(self):
        """Utterances without a wake word are ignored."""
        for text in ("去冰箱", "今天天气不错", "哎那个谁", "介绍一下留形科技", ""):
            hit, _word, _command = extract_wake_command(text)
            self.assertFalse(hit, text)

    def test_custom_wake_words(self):
        hit, word, command = extract_wake_command("小odin 去冰箱", ["小odin"])
        self.assertTrue(hit)
        self.assertEqual(word, "小odin")
        self.assertEqual(command, "去冰箱")


if __name__ == "__main__":
    unittest.main()
