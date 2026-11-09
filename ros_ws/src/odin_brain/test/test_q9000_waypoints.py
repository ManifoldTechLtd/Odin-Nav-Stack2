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

"""Unit tests for the Q9000 waypoint file (trajControlPoint.txt) module."""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from odin_brain import q9000_waypoints as q9  # noqa: E402

REAL_LINE_1 = ("1 🚽厕所 1750185647.190387 0.005055206872 -0.03666175428 "
               "-0.011002445 0.9838514799 0.01038773099 0.1731927357 -0.04396176475")
REAL_LINE_2 = ("2 2 1750185661.47981 0.2024497788 -0.7082634653 -0.007171194917 "
               "0.8239997276 0.1057905413 0.1414113464 -0.5383638559")


class TestParse(unittest.TestCase):
    def test_parse_real_line_with_emoji(self):
        w = q9.parse_line(REAL_LINE_1)
        self.assertIsNotNone(w)
        self.assertEqual(w.seq, 1)
        self.assertEqual(w.name, "🚽厕所")
        self.assertAlmostEqual(w.stamp, 1750185647.190387, places=5)
        self.assertAlmostEqual(w.x, 0.005055206872, places=9)
        self.assertAlmostEqual(w.y, -0.03666175428, places=9)
        self.assertAlmostEqual(w.z, -0.011002445, places=9)
        self.assertAlmostEqual(w.qw, 0.9838514799, places=9)
        self.assertAlmostEqual(w.qz, -0.04396176475, places=9)

    def test_parse_numeric_name(self):
        w = q9.parse_line(REAL_LINE_2)
        self.assertIsNotNone(w)
        self.assertEqual(w.seq, 2)
        self.assertEqual(w.name, "2")

    def test_parse_skips_dirty_lines(self):
        text = "\n".join([
            "",
            "# comment line",
            "1 只有一个字段",
            "abc 名称 1 2 3 4 5 6 7 8",
            "1 名 1 2 3 4 5 6 7 not_a_number",
            REAL_LINE_1,
        ])
        points = q9.parse(text)
        self.assertEqual(len(points), 1)
        self.assertEqual(points[0].name, "🚽厕所")

    def test_parse_fills_missing_seq(self):
        line = "0 名 1 2 3 4 0 0 0 1"
        w = q9.parse_line(line, fallback_seq=7)
        self.assertIsNotNone(w)
        self.assertEqual(w.seq, 7)

    def test_parse_name_with_multi_token(self):
        line = "3 销售 办公室 1.0 1.0 2.0 0.0 1.0 0.0 0.0 0.0"
        w = q9.parse_line(line)
        self.assertIsNotNone(w)
        self.assertEqual(w.name, "销售 办公室")


class TestQuaternion(unittest.TestCase):
    def test_yaw_quat_roundtrip(self):
        for deg in (0, 30, 45, 90, 135, 180, -90, -135, -179):
            yaw = math.radians(deg)
            qw, qx, qy, qz = q9.yaw_to_quaternion(yaw)
            self.assertAlmostEqual(qx, 0.0, places=12)
            self.assertAlmostEqual(qy, 0.0, places=12)
            self.assertAlmostEqual(qw * qw + qz * qz, 1.0, places=12)
            self.assertAlmostEqual(q9.quaternion_to_yaw(qw, qx, qy, qz), yaw, places=9)

    def test_yaw_property_from_real_data(self):
        w = q9.parse_line(REAL_LINE_1)
        self.assertTrue(-math.pi <= w.yaw <= math.pi)
        expect = math.atan2(2.0 * (w.qw * w.qz + w.qx * w.qy),
                            1.0 - 2.0 * (w.qy * w.qy + w.qz * w.qz))
        self.assertAlmostEqual(w.yaw, expect, places=9)
        qw, qx, qy, qz = q9.yaw_to_quaternion(w.yaw)
        self.assertAlmostEqual(q9.quaternion_to_yaw(qw, qx, qy, qz), w.yaw, places=9)


class TestSerialize(unittest.TestCase):
    def test_dump_parse_roundtrip(self):
        points = q9.parse("\n".join([REAL_LINE_1, REAL_LINE_2]))
        again = q9.parse(q9.dump(points))
        self.assertEqual(len(again), len(points))
        for a, b in zip(points, again):
            self.assertEqual(a.name, b.name)
            self.assertAlmostEqual(a.x, b.x, places=6)
            self.assertAlmostEqual(a.y, b.y, places=6)
            self.assertAlmostEqual(a.z, b.z, places=6)
            self.assertAlmostEqual(a.yaw, b.yaw, places=6)

    def test_dump_renumbers_seq(self):
        points = [q9.Q9000Waypoint(seq=99, name="a", stamp=1.0, x=0, y=0),
                  q9.Q9000Waypoint(seq=42, name="b", stamp=2.0, x=1, y=1)]
        lines = q9.dump(points).strip().splitlines()
        self.assertTrue(lines[0].startswith("1 "))
        self.assertTrue(lines[1].startswith("2 "))

    def test_normalize_name_strips_whitespace(self):
        self.assertEqual(q9.normalize_name("销售 办公室"), "销售_办公室")
        self.assertEqual(q9.normalize_name("  厕所\t"), "厕所")
        w = q9.Q9000Waypoint.from_pose("销售 办公室", 1.0, 2.0, 0.0)
        back = q9.parse_line(w.to_line())
        self.assertEqual(back.name, "销售_办公室")

    def test_save_load_file(self):
        tmpdir = tempfile.mkdtemp()
        path = os.path.join(tmpdir, "trajControlPoint.txt")
        points = q9.parse("\n".join([REAL_LINE_1, REAL_LINE_2]))
        q9.save(path, points)
        self.assertTrue(os.path.isfile(path))
        loaded = q9.load(path)
        self.assertEqual([p.name for p in loaded], ["🚽厕所", "2"])


class TestPlacesBridge(unittest.TestCase):
    def test_from_places_uses_pose_yaw(self):
        places = [
            {"name": "厕所", "aliases": ["卫生间"], "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
            {"name": "小A工位", "pose": {"x": 3.0, "y": 4.0, "yaw": math.pi / 2}},
        ]
        points = q9.from_places(places, default_stamp=1000.0)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0].stamp, 1000.0, places=6)
        self.assertEqual(points[0].aliases, ["卫生间"])
        self.assertAlmostEqual(points[1].qw, math.sqrt(0.5), places=9)
        self.assertAlmostEqual(points[1].qz, math.sqrt(0.5), places=9)

    def test_places_roundtrip_keeps_pose(self):
        places = [{"name": "会议室", "aliases": ["大会议室"],
                   "pose": {"x": 5.0, "y": 6.0, "yaw": 1.57}}]
        points = q9.from_places(places, default_stamp=1.0)
        back = q9.to_places_record(points, map_id="map_opt")
        item = back["places"][0]
        self.assertEqual(item["name"], "会议室")
        self.assertEqual(item["aliases"], ["大会议室"])
        self.assertAlmostEqual(item["pose"]["x"], 5.0, places=6)
        self.assertAlmostEqual(item["pose"]["y"], 6.0, places=6)
        self.assertAlmostEqual(item["pose"]["yaw"], 1.57, places=6)
        self.assertEqual(back["map_id"], "map_opt")
        self.assertEqual(back["frame_id"], "map")

    def test_to_places_record_generates_unique_ids(self):
        points = [q9.Q9000Waypoint.from_pose("同名", 0, 0, 0),
                  q9.Q9000Waypoint.from_pose("同名", 1, 1, 0)]
        record = q9.to_places_record(points)
        ids = [it["id"] for it in record["places"]]
        self.assertEqual(len(ids), len(set(ids)), "id 必须唯一")


class TestFrame(unittest.TestCase):
    def test_default_frame_is_map(self):
        self.assertEqual(q9.FRAME, "map")
        self.assertEqual(q9.DEFAULT_FRAME, "map")

    def test_validate_frame(self):
        self.assertTrue(q9.validate_frame("map"))
        self.assertTrue(q9.validate_frame("MAP"))
        self.assertTrue(q9.validate_frame(""))
        self.assertFalse(q9.validate_frame("odom"))
        self.assertFalse(q9.validate_frame("base_link"))

    def test_wrong_frame_is_corrected_with_warning(self):
        record = q9.to_places_record(
            [q9.Q9000Waypoint.from_pose("厕所", 1.0, 2.0, 0.0)], frame_id="odom")
        self.assertEqual(record["frame_id"], "map")
        self.assertTrue(record.get("_warnings"))
        self.assertIn("map", record["_warnings"][0])

    def test_correct_frame_has_no_warning(self):
        record = q9.to_places_record(
            [q9.Q9000Waypoint.from_pose("厕所", 1.0, 2.0, 0.0)], frame_id="map")
        self.assertEqual(record["frame_id"], "map")
        self.assertFalse(record.get("_warnings"))

    def test_z_preserved_for_3d_waypoints(self):
        w = q9.parse_line(REAL_LINE_1)
        self.assertNotEqual(w.z, 0.0)
        again = q9.parse_line(w.to_line())
        self.assertAlmostEqual(again.z, w.z, places=6)


if __name__ == "__main__":
    unittest.main()
