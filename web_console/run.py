#!/usr/bin/env python3
"""Start the ONS2 web console. Source the ROS 1 or ROS 2 environment first."""

import argparse
import os
import threading

from web_console.core import ConsoleState
from web_console.server import make_server


def parse_args():
    parser = argparse.ArgumentParser(description="ONS2 web console")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18090)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--pose-hz", type=float, default=10.0)
    parser.add_argument("--ros-version", choices=["1", "2"], default=os.environ.get("ROS_VERSION"))
    return parser.parse_args()


def main():
    args = parse_args()
    ros_version = args.ros_version or "2"
    if ros_version == "1":
        from web_console import ros1_adapter as adapter
    else:
        from web_console import ros2_adapter as adapter
    state = ConsoleState()
    server = make_server(args.host, args.port, state)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("ONS2 web console: http://{}:{} (ROS {})".format(args.host, args.port, ros_version), flush=True)
    try:
        adapter.run(args, state)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
