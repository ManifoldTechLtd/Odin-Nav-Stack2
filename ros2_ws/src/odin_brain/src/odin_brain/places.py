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

"""Semantic place database: loading, matching, persistence, and relative bearing description."""

import json
import math
import os
import re
import unicodedata
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from .models import Place

_FILLERS = (
    "请问", "麻烦", "帮我", "帮忙", "一下", "那个", "这个", "这边", "那边",
    "请", "我想去", "我要去", "想要去",
)

_FULLWIDTH_SPACE = "\u3000"


class PlaceError(Exception):
    """Place store I/O or format error."""


def normalize(text: str) -> str:
    """Normalize text via NFKC, strip whitespace and conversational filler words."""
    if not text:
        return ""
    out = unicodedata.normalize("NFKC", text)
    out = out.replace(_FULLWIDTH_SPACE, " ").strip().lower()
    for filler in _FILLERS:
        out = out.replace(filler, "")
    out = re.sub(r"\s+", "", out)
    return out


class PlaceStore:
    """Semantic place store container."""

    def __init__(self, path: str):
        self.path = os.path.abspath(path)
        self.schema_version = 1
        self.map_id = ""
        self.frame_id = "map"
        self.updated_at = ""
        self.places: List[Place] = []
        self._raw: dict = {}
        self.load()

    def load(self) -> None:
        if not os.path.exists(self.path):
            raise PlaceError("地点库文件不存在: %s" % self.path)
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except json.JSONDecodeError as exc:
            raise PlaceError("地点库不是合法 JSON: %s (%s)" % (self.path, exc))
        if not isinstance(data, dict) or "places" not in data:
            raise PlaceError("地点库缺少顶层 places 字段: %s" % self.path)
        self._raw = data
        self.schema_version = int(data.get("schema_version", 1))
        self.map_id = str(data.get("map_id", ""))
        self.frame_id = str(data.get("frame_id", "map"))
        self.updated_at = str(data.get("updated_at", ""))

        places: List[Place] = []
        for index, item in enumerate(data["places"]):
            if not isinstance(item, dict):
                raise PlaceError("places[%d] 不是对象" % index)
            pose = item.get("pose") or {}
            if not isinstance(pose, dict) or "x" not in pose or "y" not in pose:
                raise PlaceError("places[%d] 缺少 pose.x / pose.y" % index)
            place = Place(
                id=str(item.get("id") or "place_%d" % index),
                name=str(item.get("name") or item.get("id") or "place_%d" % index),
                x=float(pose["x"]),
                y=float(pose["y"]),
                yaw=float(pose.get("yaw", 0.0)),
                aliases=[str(a) for a in (item.get("aliases") or [])],
                dwell_default_sec=float(item.get("dwell_default_sec", 0.0)),
                map_id=str(item.get("map_id", self.map_id)),
                notes=str(item.get("notes", "")),
            )
            places.append(place)
        self.places = places

    def save(self) -> None:
        """Persist places back to JSON atomically, preserving unrecognized fields."""
        payload = dict(self._raw)
        payload["schema_version"] = self.schema_version
        payload["map_id"] = self.map_id
        payload["frame_id"] = self.frame_id
        payload["updated_at"] = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        payload["places"] = [self._place_payload(place) for place in self.places]

        tmp_path = self.path + ".tmp"
        directory = os.path.dirname(self.path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory, exist_ok=True)
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(tmp_path, self.path)
        self._raw = payload
        self.updated_at = payload["updated_at"]

    def _place_payload(self, place: Place) -> dict:
        original: Dict = {}
        for item in self._raw.get("places", []):
            if isinstance(item, dict) and str(item.get("id", "")) == place.id:
                original = dict(item)
                break
        original.update({
            "id": place.id,
            "name": place.name,
            "aliases": list(place.aliases),
            "pose": {"x": place.x, "y": place.y, "yaw": place.yaw},
            "dwell_default_sec": place.dwell_default_sec,
            "map_id": place.map_id or self.map_id,
        })
        if place.notes:
            original["notes"] = place.notes
        return original

    def get(self, key: str) -> Optional[Place]:
        target = normalize(key)
        for place in self.places:
            if normalize(place.id) == target or normalize(place.name) == target:
                return place
        for place in self.places:
            if any(normalize(alias) == target for alias in place.aliases):
                return place
        return None

    def candidates(self, text: str) -> List[Tuple[int, Place, str]]:
        """Return matching (match_length, Place, matched_token) tuples sorted by length descending."""
        norm = normalize(text)
        if not norm:
            return []
        found: List[Tuple[int, Place, str]] = []
        for place in self.places:
            keys = [place.name] + list(place.aliases)
            best: Optional[Tuple[int, str]] = None
            for key in keys:
                norm_key = normalize(key)
                if len(norm_key) < 2:
                    continue
                if norm_key in norm:
                    if best is None or len(norm_key) > best[0]:
                        best = (len(norm_key), key)
            if best is not None:
                found.append((best[0], place, best[1]))
        found.sort(key=lambda item: (-item[0], item[1].name))
        return found

    def resolve(self, text: str) -> Tuple[Optional[Place], List[Place]]:
        """Resolve a place reference in `text`, returning (matched_place, ambiguous_candidates)."""
        found = self.candidates(text)
        if not found:
            return None, []
        top_len = found[0][0]
        top_places: List[Place] = []
        for length, place, _key in found:
            if length != top_len:
                break
            if place not in top_places:
                top_places.append(place)
        if len(top_places) > 1:
            return None, top_places
        return top_places[0], []

    def upsert(self, name: str, x: float, y: float, yaw: float = 0.0,
               aliases: Optional[List[str]] = None, notes: str = "") -> Place:
        place = self.get(name)
        if place is None:
            place = Place(
                id=self._make_id(name),
                name=name,
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                aliases=list(aliases or []),
                map_id=self.map_id,
                notes=notes,
            )
            self.places.append(place)
        else:
            place.x = float(x)
            place.y = float(y)
            place.yaw = float(yaw)
            if aliases:
                for alias in aliases:
                    if alias not in place.aliases:
                        place.aliases.append(alias)
            if notes:
                place.notes = notes
        return place

    def _make_id(self, name: str) -> str:
        base = normalize(name) or "place"
        base = re.sub(r"[^0-9a-z\u4e00-\u9fff]+", "_", base)
        existing = {place.id for place in self.places}
        candidate = base
        suffix = 2
        while candidate in existing:
            candidate = "%s_%d" % (base, suffix)
            suffix += 1
        return candidate

    def check_map_id(self, runtime_map_id: str) -> Tuple[bool, str]:
        """Verify that the store map_id matches `runtime_map_id` (skipped if either is empty)."""
        runtime_map_id = (runtime_map_id or "").strip()
        if not runtime_map_id or not self.map_id:
            return True, ""
        if runtime_map_id != self.map_id:
            return False, (
                "地图不匹配：当前运行地图为 %s，但地点库是 %s，请切换地点库或重新标定"
                % (runtime_map_id, self.map_id)
            )
        return True, ""

    def hotwords(self) -> List[str]:
        """Export deduplicated place names and aliases for ASR hotword boosting."""
        words: List[str] = []
        for place in self.places:
            for token in [place.name] + list(place.aliases):
                token = token.strip()
                if len(token) >= 2 and token not in words:
                    words.append(token)
        return words

    def prompt_block(self) -> str:
        """Format the place list for inclusion in LLM system prompts."""
        lines = ["map_id=%s frame=%s 共 %d 个地点：" % (self.map_id, self.frame_id, len(self.places))]
        for place in self.places:
            alias = ("/" + "、".join(place.aliases)) if place.aliases else ""
            lines.append(
                "  - %s%s: x=%.2f y=%.2f yaw=%.2f" % (place.name, alias, place.x, place.y, place.yaw)
            )
        return "\n".join(lines)


def describe_relative_pose(robot_pose: Dict, place: Place) -> str:
    """Format a natural-language relative direction and distance from `robot_pose` to `place`."""
    rx = float(robot_pose["x"])
    ry = float(robot_pose["y"])
    ryaw = float(robot_pose.get("yaw", 0.0))
    dx = float(place.x) - rx
    dy = float(place.y) - ry
    dist = math.hypot(dx, dy)
    if dist < 0.4:
        return "%s就在你身边（不到 0.5 米）" % place.name
    target_yaw = math.atan2(dy, dx)
    delta = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))
    deg = math.degrees(delta)
    abs_deg = abs(deg)
    if abs_deg <= 20:
        direction = "正前方"
    elif abs_deg <= 70:
        direction = "左前方" if deg > 0 else "右前方"
    elif abs_deg <= 110:
        direction = "正左方" if deg > 0 else "正右方"
    elif abs_deg <= 160:
        direction = "左后方" if deg > 0 else "右后方"
    else:
        direction = "正后方"
    return "%s在你的%s约 %.1f 米处" % (place.name, direction, dist)
