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

from odin_brain.places import PlaceError, PlaceStore, normalize  # noqa: E402


def _write_places(path, places):
    data = {
        "schema_version": 1,
        "map_id": "map_opt",
        "frame_id": "map",
        "updated_at": "2026-10-23T00:00:00",
        "places": places,
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)


class TestNormalize(unittest.TestCase):
    def test_fullwidth_space_and_filler(self):
        self.assertEqual(normalize("请问　去　WC "), "去wc")

    def test_none_and_empty(self):
        self.assertEqual(normalize(""), "")
        self.assertEqual(normalize(None), "")


class TestPlaceStore(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.path = os.path.join(self.tmpdir, "places.json")
        _write_places(self.path, [
            {"id": "p.toilet", "name": "厕所", "aliases": ["卫生间", "WC"],
             "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5}},
            {"id": "p.desk", "name": "小A工位", "aliases": ["小A"],
             "pose": {"x": 3.0, "y": 4.0, "yaw": 0.1}, "dwell_default_sec": 2},
            {"id": "p.gate1", "name": "1号门", "aliases": [],
             "pose": {"x": 5.0, "y": 6.0}},
            {"id": "p.gate2", "name": "2号门", "aliases": [],
             "pose": {"x": 7.0, "y": 8.0}},
        ])
        self.store = PlaceStore(self.path)

    def test_load_meta(self):
        self.assertEqual(self.store.map_id, "map_opt")
        self.assertEqual(len(self.store.places), 4)

    def test_resolve_exact(self):
        place, options = self.store.resolve("去厕所")
        self.assertIsNotNone(place)
        self.assertEqual(place.name, "厕所")
        self.assertEqual(options, [])

    def test_resolve_alias(self):
        place, _ = self.store.resolve("带我去卫生间")
        self.assertEqual(place.name, "厕所")

    def test_resolve_longest_match(self):
        place, _ = self.store.resolve("先去小A工位等一会")
        self.assertEqual(place.name, "小A工位")

    def test_resolve_ambiguous_same_length(self):
        place, options = self.store.resolve("去1号门还是2号门")
        self.assertIsNone(place)
        self.assertEqual(len(options), 2)

    def test_resolve_unknown(self):
        place, options = self.store.resolve("去食堂")
        self.assertIsNone(place)
        self.assertEqual(options, [])

    def test_single_char_alias_ignored(self):
        self.assertEqual(self.store.candidates("工"), [])

    def test_upsert_and_save_roundtrip(self):
        place = self.store.upsert("茶水间", 9.5, -1.5, 1.2, aliases=["水房"])
        self.assertEqual(place.id, "茶水间")
        self.store.save()
        reloaded = PlaceStore(self.path)
        place2 = reloaded.get("茶水间")
        self.assertIsNotNone(place2)
        self.assertAlmostEqual(place2.x, 9.5)
        self.assertAlmostEqual(place2.yaw, 1.2)
        self.assertIn("水房", place2.aliases)

    def test_upsert_updates_existing(self):
        self.store.upsert("厕所", 10.0, 20.0, 0.0)
        place = self.store.get("厕所")
        self.assertAlmostEqual(place.x, 10.0)
        self.assertEqual(len(self.store.places), 4)

    def test_prompt_block_contains_places(self):
        block = self.store.prompt_block()
        self.assertIn("厕所", block)
        self.assertIn("map_opt", block)

    def test_missing_file_raises(self):
        with self.assertRaises(PlaceError):
            PlaceStore(os.path.join(self.tmpdir, "not_exist.json"))


if __name__ == "__main__":
    unittest.main()
