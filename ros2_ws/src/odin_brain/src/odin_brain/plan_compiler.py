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

"""Plan compiler grouping consecutive goto steps into mission segments and keeping atomic actions separate."""

from typing import Dict, List

from .models import Plan, Step

MISSION = "mission"
ATOMIC_KINDS = ("speak", "turn", "move_rel", "wait", "home", "ask_and_wait", "som_goto")


def compile_segments(plan: Plan) -> List[Dict]:
    """Compile a Plan into an ordered list of executable segments."""
    segments: List[Dict] = []
    buffer: List[Step] = []

    def flush() -> None:
        if not buffer:
            return
        segments.append({
            "type": MISSION,
            "steps": list(buffer),
            "waypoints": [{
                "x": float(step.place.x),
                "y": float(step.place.y),
                "yaw": float(step.place.yaw),
                "dwell_sec": float(step.dwell_sec),
                "name": step.place.name,
            } for step in buffer],
            "labels": [step.label for step in buffer],
        })
        buffer.clear()

    for step in plan.steps:
        if step.kind == "goto" and step.place is not None:
            if step.modifier:
                flush()
                segments.append({
                    "type": "som_goto",
                    "step": step,
                    "waypoints": [{
                        "x": float(step.place.x),
                        "y": float(step.place.y),
                        "yaw": float(step.place.yaw),
                        "dwell_sec": float(step.dwell_sec),
                        "name": step.place.name,
                    }],
                })
                continue
            buffer.append(step)
            continue
        flush()
        segments.append({"type": step.kind, "step": step})

    flush()
    return segments


def is_pure_mission(segments: List[Dict]) -> bool:
    """Return True when the compiled plan consists of a single mission segment without atomic actions."""
    return len(segments) == 1 and segments[0]["type"] == MISSION


def describe_segments(segments: List[Dict]) -> str:
    parts = []
    for seg in segments:
        if seg["type"] == MISSION:
            parts.append("→".join(seg["labels"]))
        else:
            parts.append(seg["step"].label)
    return " | ".join(parts)