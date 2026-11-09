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

"""Q9000 waypoint file (trajControlPoint.txt) parser and serializer.

File format (space-separated, one waypoint per line):
    seq name timestamp x y z qw qx qy qz

Note:
- Quaternion order in Q9000 files is (qw, qx, qy, qz) with w first.
- Q9000 waypoints are recorded in the 3D mapping frame, which coincides with
  the navigation `map` frame once relocalization (`custom_map_mode=2`) is active.
"""

import math
import os
import time
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

FRAME = "map"
DEFAULT_FRAME = FRAME
DEFAULT_FILENAME = "trajControlPoint.txt"


def validate_frame(frame_id: str) -> bool:
    """Return True if frame_id is empty or 'map'."""
    value = (frame_id or "").strip().lower()
    return value in ("", FRAME)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert planar yaw (rad) to quaternion (qw, qx, qy, qz) around the Z axis."""
    half = 0.5 * float(yaw)
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def quaternion_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract planar yaw (rad) from quaternion (qw, qx, qy, qz)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_name(name: str) -> str:
    """Replace whitespace in waypoint names with underscores for space-delimited Q9000 output."""
    return "_".join(str(name or "").split())


@dataclass
class Q9000Waypoint:
    """Single Q9000 waypoint entry."""

    seq: int
    name: str
    stamp: float
    x: float
    y: float
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    aliases: List[str] = field(default_factory=list)
    dwell_default_sec: float = 0.0
    notes: str = ""

    @property
    def yaw(self) -> float:
        return quaternion_to_yaw(self.qw, self.qx, self.qy, self.qz)

    @property
    def pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    @classmethod
    def from_pose(cls, name: str, x: float, y: float, yaw: float, z: float = 0.0,
                  seq: int = 0, stamp: Optional[float] = None) -> "Q9000Waypoint":
        qw, qx, qy, qz = yaw_to_quaternion(yaw)
        return cls(seq=seq, name=name, stamp=time.time() if stamp is None else float(stamp),
                   x=float(x), y=float(y), z=float(z),
                   qw=qw, qx=qx, qy=qy, qz=qz)

    def to_line(self) -> str:
        """Format as a single Q9000 line."""
        return "%d %s %.6f %.9f %.9f %.9f %.9f %.9f %.9f %.9f" % (
            self.seq, normalize_name(self.name), self.stamp,
            self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)

    def to_pose_dict(self) -> dict:
        """Convert to places.json pose dictionary."""
        return {"x": round(self.x, 6), "y": round(self.y, 6), "yaw": round(self.yaw, 6)}


def parse_line(line: str, fallback_seq: int = 0) -> Optional[Q9000Waypoint]:
    """Parse a single Q9000 waypoint line, returning None on invalid or comment lines."""
    raw = (line or "").strip()
    if not raw or raw.startswith("#"):
        return None
    parts = raw.split()
    # seq + name (1 or more tokens) + 8 numeric fields (stamp, x, y, z, qw, qx, qy, qz)
    if len(parts) < 10:
        return None
    try:
        seq = int(float(parts[0]))
    except ValueError:
        return None
    nums = parts[-8:]
    try:
        stamp, x, y, z, qw, qx, qy, qz = [float(v) for v in nums]
    except ValueError:
        return None
    name = " ".join(parts[1:len(parts) - 8]).strip()
    if not name:
        return None
    return Q9000Waypoint(seq=seq or fallback_seq, name=name, stamp=stamp,
                         x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz)


def parse(text: str) -> List[Q9000Waypoint]:
    """Parse full Q9000 file contents."""
    points: List[Q9000Waypoint] = []
    for idx, line in enumerate((text or "").splitlines(), 1):
        point = parse_line(line, fallback_seq=idx)
        if point is not None:
            points.append(point)
    return points


def dump(points: Iterable[Q9000Waypoint]) -> str:
    """Serialize waypoints to Q9000 text with sequential 1-based numbering."""
    lines = []
    for idx, point in enumerate(points, 1):
        point.seq = idx
        lines.append(point.to_line())
    return "\n".join(lines) + ("\n" if lines else "")


def load(path: str) -> List[Q9000Waypoint]:
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        return parse(handle.read())


def save(path: str, points: Sequence[Q9000Waypoint]) -> str:
    """Save waypoints to path in UTF-8 without BOM."""
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(dump(points))
    return path


def from_places(places: Sequence[dict], default_stamp: Optional[float] = None) -> List[Q9000Waypoint]:
    """Convert places.json place dicts into Q9000Waypoint objects."""
    stamp = time.time() if default_stamp is None else float(default_stamp)
    points: List[Q9000Waypoint] = []
    for idx, item in enumerate(places or [], 1):
        name = str(item.get("name") or "").strip()
        if not name:
            continue
        pose = item.get("pose") or {}
        point = Q9000Waypoint.from_pose(
            name=name,
            x=float(pose.get("x", 0.0)),
            y=float(pose.get("y", 0.0)),
            yaw=float(pose.get("yaw", 0.0)),
            z=float(pose.get("z", 0.0)),
            seq=idx,
            stamp=stamp,
        )
        point.aliases = list(item.get("aliases") or [])
        point.dwell_default_sec = float(item.get("dwell_default_sec") or 0.0)
        point.notes = str(item.get("notes") or "")
        points.append(point)
    return points


def to_places_record(points: Sequence[Q9000Waypoint], map_id: str = "",
                     frame_id: str = FRAME) -> dict:
    """Convert Q9000Waypoint objects into a places.json record dict."""
    warnings: List[str] = []
    if not validate_frame(frame_id):
        warnings.append("Q9000 航点必须在 map 系，收到 frame_id=%r，已纠正为 %s"
                        % (frame_id, FRAME))
        frame_id = FRAME
    places: List[dict] = []
    used_ids = set()
    for idx, point in enumerate(points, 1):
        base = "q9000.%02d" % idx
        pid = base
        bump = 1
        while pid in used_ids:
            bump += 1
            pid = "%s_%d" % (base, bump)
        used_ids.add(pid)
        item = {
            "id": pid,
            "name": point.name,
            "aliases": list(point.aliases or []),
            "pose": point.to_pose_dict(),
            "dwell_default_sec": float(point.dwell_default_sec or 0.0),
        }
        if map_id:
            item["map_id"] = map_id
        notes = point.notes or "由 Q9000 trajControlPoint.txt 导入"
        item["notes"] = notes
        places.append(item)
    record = {
        "schema_version": 1,
        "map_id": map_id or "",
        "frame_id": frame_id or FRAME,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "places": places,
    }
    if warnings:
        record["_warnings"] = warnings
    return record


def default_path(places_path: str = "") -> str:
    """Infer trajControlPoint.txt path in the same directory as places_path."""
    if places_path:
        return os.path.join(os.path.dirname(os.path.abspath(places_path)), DEFAULT_FILENAME)
    return os.path.join(os.path.expanduser("~"), ".config", "odin_brain", DEFAULT_FILENAME)
