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

"""Command-line tool for maintaining the semantic place store (list, add, record from TF)."""

import argparse
import os
import sys
from typing import List, Optional

from .places import PlaceStore


def _default_path() -> str:
    """Resolve places database path matching cli.default_places_path."""
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


def _cmd_list(store: PlaceStore) -> int:
    print("地点库: %s" % store.path)
    print("map_id=%s frame=%s 共 %d 个地点（updated_at=%s）" % (
        store.map_id or "-", store.frame_id, len(store.places), store.updated_at or "-"))
    for place in store.places:
        alias = ("  别名: " + "、".join(place.aliases)) if place.aliases else ""
        note = ("  备注: " + place.notes) if place.notes else ""
        print("  - %-12s x=%8.3f y=%8.3f yaw=%6.3f%s%s" % (
            place.name, place.x, place.y, place.yaw, alias, note))
    return 0


def _cmd_add(store: PlaceStore, args) -> int:
    if args.x is None or args.y is None:
        print("[fail] add 需要 --x 与 --y", file=sys.stderr)
        return 2
    place = store.upsert(args.name, args.x, args.y, args.yaw or 0.0,
                         aliases=_parse_aliases(args.aliases), notes=args.notes or "")
    store.save()
    print("[ok  ] 已写入 %s: x=%.3f y=%.3f yaw=%.3f" % (place.name, place.x, place.y, place.yaw))
    return 0


def _cmd_record(store: PlaceStore, args) -> int:
    """Record the current robot pose from TF into the place store (supports ROS 1 and ROS 2)."""
    import time

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
        print("[fail] 取不到 %s -> %s 的 TF（导航栈没起或尚未重定位）" % (
            args.map_frame, args.base_frame), file=sys.stderr)
        return 3

    place = store.upsert(args.name, pose["x"], pose["y"], pose["yaw"],
                         aliases=_parse_aliases(args.aliases), notes=args.notes or "")
    store.save()
    print("[ok  ] 已记录 %s: x=%.3f y=%.3f yaw=%.3f（来自 TF）" % (
        place.name, place.x, place.y, place.yaw))
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="brain_places", description="地点库维护工具")
    parser.add_argument("--places", default="", help="地点库 JSON 路径")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="列出地点")

    add = sub.add_parser("add", help="手工录入坐标（无需 ROS）")
    add.add_argument("name")
    add.add_argument("--x", type=float, required=True)
    add.add_argument("--y", type=float, required=True)
    add.add_argument("--yaw", type=float, default=0.0)
    add.add_argument("--aliases", default="")
    add.add_argument("--notes", default="")

    record = sub.add_parser("record", help="用当前位姿录入（需要 TF）")
    record.add_argument("name")
    record.add_argument("--aliases", default="")
    record.add_argument("--notes", default="")
    record.add_argument("--map-frame", default="map")
    record.add_argument("--base-frame", default="base_link")
    record.add_argument("--timeout", type=float, default=5.0)

    args = parser.parse_args(argv)
    store = PlaceStore(args.places or _default_path())
    if args.command == "list":
        return _cmd_list(store)
    if args.command == "add":
        return _cmd_add(store, args)
    if args.command == "record":
        return _cmd_record(store, args)
    return 2


if __name__ == "__main__":
    try:
        import rospy
        _argv = rospy.myargv(argv=sys.argv)[1:]
    except Exception:
        _argv = sys.argv[1:]
    sys.exit(main(_argv))
