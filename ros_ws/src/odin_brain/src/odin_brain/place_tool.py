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

"""CLI utility for managing the place store (places.json) and Q9000 waypoint files (trajControlPoint.txt).

Examples:
    brain_places list
    brain_places list --format q9000
    brain_places add 厕所 --x 12.34 --y -3.21 --yaw 1.57 --aliases 卫生间,WC
    brain_places record 茶水间 --aliases 水房
    brain_places export-q9000 [path]
    brain_places import-q9000 <file> [--replace]
"""

import argparse
import json
import os
import sys
import time
from typing import List, Optional

from . import q9000_waypoints
from .places import PlaceStore


def _default_path() -> str:
    """Resolve places.json path: $ODIN_BRAIN_PLACES -> ~/.config/odin_brain/places.json -> package config/places.json."""
    env = os.environ.get("ODIN_BRAIN_PLACES", "")
    if env:
        return os.path.expanduser(env)
    user_path = os.path.expanduser("~/.config/odin_brain/places.json")
    if os.path.isfile(user_path):
        return user_path
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "config", "places.json"))


def _parse_aliases(text: str) -> List[str]:
    if not text:
        return []
    normalized = text.replace("，", ",").replace("、", ",")
    return [item.strip() for item in normalized.split(",") if item.strip()]


def _q9000_path(store: PlaceStore, args) -> str:
    """Resolve trajControlPoint.txt path (explicit argument or same directory as places.json)."""
    explicit = getattr(args, "q9000", "") or ""
    if explicit:
        return os.path.expanduser(explicit)
    return q9000_waypoints.default_path(store.path)


def _places_as_dicts(store: PlaceStore) -> List[dict]:
    """Convert in-memory Place objects to dicts for q9000_waypoints.from_places."""
    items = []
    for place in store.places:
        items.append({
            "name": place.name,
            "pose": {"x": place.x, "y": place.y, "yaw": place.yaw,
                     **({"z": place.z} if getattr(place, "z", 0.0) else {})},
            "aliases": list(place.aliases or []),
            "dwell_default_sec": getattr(place, "dwell_default_sec", 0.0),
            "notes": getattr(place, "notes", ""),
        })
    return items


def _sync_q9000(store: PlaceStore, args, stamp: Optional[float] = None) -> str:
    """Mirror the current place store to the Q9000 waypoint file."""
    path = _q9000_path(store, args)
    points = q9000_waypoints.from_places(_places_as_dicts(store), default_stamp=stamp)
    q9000_waypoints.save(path, points)
    return path


def _cmd_list(store: PlaceStore, fmt: str = "table") -> int:
    if fmt == "q9000":
        points = q9000_waypoints.from_places(_places_as_dicts(store))
        sys.stdout.write(q9000_waypoints.dump(points))
        return 0
    print("Places: %s" % store.path)
    print("map_id=%s frame=%s total=%d (updated_at=%s)" % (
        store.map_id or "-", store.frame_id, len(store.places), store.updated_at or "-"))
    for place in store.places:
        alias = ("  aliases: " + ", ".join(place.aliases)) if place.aliases else ""
        note = ("  notes: " + place.notes) if place.notes else ""
        print("  - %-12s x=%8.3f y=%8.3f yaw=%6.3f%s%s" % (
            place.name, place.x, place.y, place.yaw, alias, note))
    return 0


def _cmd_add(store: PlaceStore, args) -> int:
    if args.x is None or args.y is None:
        print("[fail] add requires --x and --y", file=sys.stderr)
        return 2
    place = store.upsert(args.name, args.x, args.y, args.yaw or 0.0,
                         aliases=_parse_aliases(args.aliases), notes=args.notes or "")
    store.save()
    q9_path = _sync_q9000(store, args, stamp=time.time())
    print("[ok  ] Saved %s: x=%.3f y=%.3f yaw=%.3f" % (place.name, place.x, place.y, place.yaw))
    print("       Synced Q9000 waypoints: %s" % q9_path)
    return 0


def _cmd_record(store: PlaceStore, args) -> int:
    """Record current robot pose from TF (supports ROS 1 and ROS 2 based on ROS_VERSION)."""
    if os.environ.get("ROS_VERSION") == "2":
        from .ros2_nav import Ros2Nav
        nav = Ros2Nav(map_frame=args.map_frame, base_frame=args.base_frame,
                      node_name="odin_brain_place_tool")
    else:
        import rospy
        from .ros1_nav import Ros1Nav
        if not rospy.core.is_initialized():
            rospy.init_node("odin_brain_place_tool", anonymous=True, disable_signals=True)
        nav = Ros1Nav(map_frame=args.map_frame, base_frame=args.base_frame)

    pose = None
    deadline = time.time() + args.timeout
    try:
        while time.time() < deadline:
            pose = nav.get_pose()
            if pose is not None:
                break
            time.sleep(0.2)
    finally:
        if hasattr(nav, "shutdown"):
            try:
                nav.shutdown()
            except Exception:
                pass

    if pose is None:
        print("[fail] TF %s -> %s unavailable (navigation stack not running or not relocalized)" % (
            args.map_frame, args.base_frame), file=sys.stderr)
        return 3

    place = store.upsert(args.name, pose["x"], pose["y"], pose["yaw"],
                         aliases=_parse_aliases(args.aliases), notes=args.notes or "")
    store.save()
    q9_path = _sync_q9000(store, args, stamp=time.time())
    print("[ok  ] Recorded %s: x=%.3f y=%.3f yaw=%.3f (from TF)" % (
        place.name, place.x, place.y, place.yaw))
    print("       Synced Q9000 waypoints: %s" % q9_path)
    return 0


def _cmd_export_q9000(store: PlaceStore, args) -> int:
    """Export places.json to trajControlPoint.txt."""
    path = os.path.expanduser(args.path) if args.path else _q9000_path(store, args)
    points = q9000_waypoints.from_places(_places_as_dicts(store), default_stamp=time.time())
    if not points:
        print("[fail] Place store is empty; nothing to export", file=sys.stderr)
        return 2
    q9000_waypoints.save(path, points)
    print("[ok  ] Exported %d waypoints -> %s" % (len(points), path))
    return 0


def _cmd_import_q9000(store: PlaceStore, args) -> int:
    """Import trajControlPoint.txt into places.json (merges by name unless --replace is set)."""
    src = os.path.expanduser(args.path)
    if not os.path.isfile(src):
        print("[fail] File not found: %s" % src, file=sys.stderr)
        return 2
    points = q9000_waypoints.load(src)
    if not points:
        print("[fail] No waypoints parsed; check format (index name timestamp x y z qw qx qy qz)",
              file=sys.stderr)
        return 3

    if args.replace:
        record = q9000_waypoints.to_places_record(points, map_id=store.map_id,
                                                   frame_id=store.frame_id or "map")
        with open(store.path, "w", encoding="utf-8") as handle:
            json.dump(record, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        store.load()
    else:
        for point in points:
            store.upsert(point.name, point.x, point.y, point.yaw, z=point.z)
        store.save()

    q9_path = _sync_q9000(store, args)
    print("[ok  ] Imported %d waypoints (from %s, mode=%s)" % (
        len(points), src, "replace" if args.replace else "merge"))
    print("       Places: %s (%d total)" % (store.path, len(store.places)))
    print("       Q9000 mirror: %s" % q9_path)
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="brain_places", description="Place store management utility")
    parser.add_argument("--places", default="", help="Path to places.json")
    parser.add_argument("--q9000", default="",
                        help="Path to Q9000 waypoint file (trajControlPoint.txt)")
    sub = parser.add_subparsers(dest="command", required=True)

    lst = sub.add_parser("list", help="List places")
    lst.add_argument("--format", choices=("table", "q9000"), default="table",
                     help="table=human-readable; q9000=Q9000 waypoint format")

    export_q9 = sub.add_parser("export-q9000", help="Export to Q9000 trajControlPoint.txt")
    export_q9.add_argument("path", nargs="?", default="", help="Output path")

    import_q9 = sub.add_parser("import-q9000", help="Import from Q9000 trajControlPoint.txt")
    import_q9.add_argument("path", help="Path to Q9000 waypoint file")
    import_q9.add_argument("--replace", action="store_true",
                           help="Replace entire place store instead of merging by name")

    add = sub.add_parser("add", help="Add or update a place manually (no ROS required)")
    add.add_argument("name")
    add.add_argument("--x", type=float, required=True)
    add.add_argument("--y", type=float, required=True)
    add.add_argument("--yaw", type=float, default=0.0)
    add.add_argument("--aliases", default="")
    add.add_argument("--notes", default="")

    record = sub.add_parser("record", help="Record current robot pose from TF")
    record.add_argument("name")
    record.add_argument("--aliases", default="")
    record.add_argument("--notes", default="")
    record.add_argument("--map-frame", default="map")
    record.add_argument("--base-frame", default="base_link")
    record.add_argument("--timeout", type=float, default=5.0)

    args = parser.parse_args(argv)
    store = PlaceStore(args.places or _default_path())
    if args.command == "list":
        return _cmd_list(store, getattr(args, "format", "table"))
    if args.command == "add":
        return _cmd_add(store, args)
    if args.command == "record":
        return _cmd_record(store, args)
    if args.command == "export-q9000":
        return _cmd_export_q9000(store, args)
    if args.command == "import-q9000":
        return _cmd_import_q9000(store, args)
    return 2


if __name__ == "__main__":
    try:
        import rospy
        _argv = rospy.myargv(argv=sys.argv)[1:]
    except Exception:
        _argv = sys.argv[1:]
    sys.exit(main(_argv))
