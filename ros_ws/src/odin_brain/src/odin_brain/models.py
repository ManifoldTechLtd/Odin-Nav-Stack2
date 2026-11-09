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

"""Data models: Place, Step, and Plan (ROS-independent)."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Place:
    """Registered semantic place in the map frame."""

    id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0
    # Preserved for round-trip compatibility with Q9000 3D waypoint files
    z: float = 0.0
    aliases: List[str] = field(default_factory=list)
    dwell_default_sec: float = 0.0
    map_id: str = ""
    notes: str = ""

    @property
    def pose(self) -> tuple:
        return (self.x, self.y, self.yaw)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "aliases": list(self.aliases),
            "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
            "dwell_default_sec": self.dwell_default_sec,
            "notes": self.notes,
        }


@dataclass
class Step:
    """A single step in an execution plan.

    kind:
      goto          -- Navigate to a semantic place (requires place)
      home          -- Return to origin
      speak         -- Speak text
      turn          -- In-place rotation (angle_deg, positive = CCW / left)
      move_rel      -- Relative displacement (direction + distance_m)
      wait          -- Dwell / wait (dwell_sec)
      ask_and_wait  -- Speak prompt and wait for ASR keywords or timeout
    """

    place: Optional[Place] = None
    dwell_sec: float = 0.0
    kind: str = "goto"
    text: str = ""
    angle_deg: float = 0.0
    distance_m: float = 0.0
    direction: str = "forward"   # forward | backward | left | right
    expect_keywords: List[str] = field(default_factory=list)
    timeout_sec: float = 20.0
    on_match: List["Step"] = field(default_factory=list)
    on_timeout: List["Step"] = field(default_factory=list)
    modifier: str = ""

    @property
    def label(self) -> str:
        if self.kind == "home":
            return "起点"
        if self.kind == "speak":
            return "说“%s”" % (self.text[:12] + ("…" if len(self.text) > 12 else ""))
        if self.kind == "turn":
            return "转 %.0f°" % self.angle_deg
        if self.kind == "move_rel":
            names = {"forward": "前", "backward": "后", "left": "左", "right": "右"}
            return "%s移 %.1fm" % (names.get(self.direction, self.direction), self.distance_m)
        if self.kind == "wait":
            return "等 %.0fs" % self.dwell_sec
        if self.kind == "ask_and_wait":
            return "问并等 %.0fs" % self.timeout_sec
        if self.place is not None and self.modifier:
            return "%s（%s）" % (self.place.name, self.modifier)
        return self.place.name if self.place else "?"


@dataclass
class Plan:
    """Parsed execution plan."""

    steps: List[Step] = field(default_factory=list)
    source: str = "rule"          # rule | llm
    text: str = ""                # Raw input text
    say: str = ""                 # Optional user-facing reply
    announce: str = ""            # Optional arrival announcement template (e.g. "到达{place}")
    notes: List[str] = field(default_factory=list)

    def goto_steps(self) -> List[Step]:
        return [s for s in self.steps if s.kind == "goto" and s.place is not None]

    def waypoints(self) -> List[dict]:
        """Convert goto steps into StartMission Waypoint dictionaries."""
        out = []
        for step in self.goto_steps():
            out.append({
                "x": float(step.place.x),
                "y": float(step.place.y),
                "yaw": float(step.place.yaw),
                "dwell_sec": float(step.dwell_sec),
                "name": step.place.name,
            })
        return out

    def describe(self) -> str:
        parts = []
        for idx, step in enumerate(self.steps, 1):
            if step.kind == "home":
                parts.append("%d. 返回起点" % idx)
            elif step.kind == "goto":
                if step.dwell_sec > 0:
                    parts.append("%d. %s（停留 %.1fs）" % (idx, step.label, step.dwell_sec))
                else:
                    parts.append("%d. %s" % (idx, step.label))
            else:
                parts.append("%d. %s" % (idx, step.label))
        return "；".join(parts) if parts else "（空计划）"
