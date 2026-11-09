#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

"""Load relocalization map for Odin1Lite via ROS services.

Services used (odin_ros_driver_rev1):
  - /<prefix>/<device>/device_mode   (uint8 mode: 0=Standby, 1=Normal, 2=Upgrade,
                                      3=SLAM, 4=Relocation, 5=ReMap)
  - /<prefix>/<device>/load_map      (string path)
  - /<prefix>/<device>/reset_initial_pose

Exit codes: 0=success, 2=service timeout, 3=mode switch failed, 4=map load failed.
"""
import argparse
import os
import sys
import time

import rospy
from rospkg import RosPack

SERVICE_WAIT_SEC = 0.5


def _resolve_srv_type(pkg, name):
    """Dynamically import a service type from a ROS package."""
    import importlib
    mod = importlib.import_module("{}.srv".format(pkg))
    return getattr(mod, name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-bin", required=True, help="Absolute path to relocalization .bin map")
    ap.add_argument("--prefix", default="odin1", help="Topic prefix (default: odin1)")
    ap.add_argument("--device", default="device0", help="Device identifier (default: device0)")
    ap.add_argument("--timeout", type=int, default=90, help="Service wait timeout in seconds")
    ap.add_argument("--no-mode-switch", action="store_true", help="Do not switch to device_mode(4)")
    ap.add_argument("--no-reset", action="store_true", help="Do not call reset_initial_pose")
    ap.add_argument("--pkg", default="odin_ros_driver_rev1", help="Driver package name")
    args = ap.parse_args()

    if not os.path.isfile(args.map_bin):
        print("[load_map] [FAIL] Map file not found: {}".format(args.map_bin), file=sys.stderr)
        return 4

    rospy.init_node("odin_load_map_once", anonymous=True, disable_signals=True)

    base = "/{}/{}/".format(args.prefix, args.device)
    svc_mode = base + "device_mode"
    svc_load = base + "load_map"
    svc_reset = base + "reset_initial_pose"

    # 1. Wait for driver services to become available
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        try:
            rospy.wait_for_service(svc_load, timeout=SERVICE_WAIT_SEC)
            break
        except rospy.ROSException:
            pass
        time.sleep(1.0)
    else:
        print("[load_map] [FAIL] Timed out waiting {:.0f}s for service {}".format(args.timeout, svc_load),
              file=sys.stderr)
        return 2

    DeviceMode = _resolve_srv_type(args.pkg, "DeviceMode")
    LoadMap = _resolve_srv_type(args.pkg, "LoadMap")
    ResetPose = _resolve_srv_type(args.pkg, "ResetInitialPose")

    # 2. Switch to relocation mode (mode=4)
    if not args.no_mode_switch:
        try:
            rospy.wait_for_service(svc_mode, timeout=5.0)
            call = rospy.ServiceProxy(svc_mode, DeviceMode)
            resp = call(mode=4)
            ok = getattr(resp, "success", False)
            print("[load_map] device_mode(4=Relocation) -> success={} code={} msg={}"
                  .format(ok, getattr(resp, "error_code", "?"),
                          getattr(resp, "message", "")))
            if not ok:
                print("[load_map] [FAIL] Failed to switch to relocation mode", file=sys.stderr)
                if getattr(resp, "error_code", None) == 1:
                    print("[load_map] error_code=1: SDK/device or relocation session not ready; "
                          "check InitLcSession/loader error in odin.log. "
                          "If the device is connected, verify libmanifold_lc_sdk.so.2 and its dependencies.",
                          file=sys.stderr)
                return 3
            time.sleep(1.0)
        except Exception as exc:  # noqa: BLE001
            print("[load_map] [FAIL] device_mode call raised exception: {}".format(exc), file=sys.stderr)
            return 3

    # 3. Load map file
    t0 = time.time()
    try:
        call = rospy.ServiceProxy(svc_load, LoadMap)
        resp = call(path=args.map_bin)
    except Exception as exc:  # noqa: BLE001
        print("[load_map] [FAIL] load_map call raised exception: {}".format(exc), file=sys.stderr)
        return 4
    ok = getattr(resp, "success", False)
    print("[load_map] load_map({}) -> success={} code={} msg={} elapsed {:.1f}s"
          .format(os.path.basename(args.map_bin), ok,
                  getattr(resp, "error_code", "?"), getattr(resp, "message", ""),
                  time.time() - t0))
    if not ok:
        return 4

    # 4. Reset initial pose
    if not args.no_reset:
        try:
            rospy.wait_for_service(svc_reset, timeout=3.0)
            call = rospy.ServiceProxy(svc_reset, ResetPose)
            resp = call()
            print("[load_map] reset_initial_pose -> success={}"
                  .format(getattr(resp, "success", False)))
        except Exception as exc:  # noqa: BLE001
            print("[load_map] [WARN] reset_initial_pose skipped: {}".format(exc), file=sys.stderr)

    print("[load_map] [OK] Map load complete; waiting for TF odom->map")
    return 0


if __name__ == "__main__":
    sys.exit(main())
