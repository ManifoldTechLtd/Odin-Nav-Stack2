#!/usr/bin/env python3
"""Start the ONS2 web console. Source the ROS 1 or ROS 2 environment first."""

import argparse
import os
import threading

from web_console.hub import MonitorStateHub
from web_console.safety import StallDetector
from web_console.server import make_server


def parse_args():
    parser = argparse.ArgumentParser(description="ONS2 web console: map, control, safety, camera and intercom")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18090, help="HTTP + HTTPS share this port")
    parser.add_argument("--no-tls", action="store_true", help="Disable the HTTPS side of the port")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--pose-hz", type=float, default=10.0)
    parser.add_argument("--odom-topic", default="/odin1/odometry")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--image-topic", default="/odin1/image/compressed")
    parser.add_argument("--stall-window-sec", type=float, default=30.0)
    parser.add_argument("--stall-radius-m", type=float, default=0.20)
    parser.add_argument("--ros-version", choices=["1", "2"], default=os.environ.get("ROS_VERSION"))
    return parser.parse_args()


def _ensure_fastdds_large_shm():
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidate = os.path.join(repo_root, "ros2_ws", "src", "map_planner", "config", "odin_cloud_filter_fastdds.xml")
    if os.path.isfile(candidate):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = candidate


def main():
    args = parse_args()
    ros_version = args.ros_version or "2"
    if ros_version == "1":
        from web_console import ros1_adapter as adapter
    else:
        _ensure_fastdds_large_shm()
        from web_console import ros2_adapter as adapter
    hub = MonitorStateHub(stall_detector=StallDetector(window_sec=args.stall_window_sec,
                                                       stall_radius_m=args.stall_radius_m))
    server = make_server(args.host, args.port, hub, enable_tls=not args.no_tls)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    scheme = "http" if args.no_tls else "http/https"
    print("ONS2 web console: {}://{}:{} (ROS {})".format(scheme, args.host, args.port, ros_version), flush=True)
    try:
        adapter.run(args, hub)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
