/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file dobot_cmd_vel_bridge.cpp
 * @brief Entry point for dobot_cmd_vel_bridge.
 *
 * Execution modes:
 *   1) Normal ROS bridge node (default):
 *        rosrun dobot_ros_control dobot_cmd_vel_bridge
 *      or roslaunch dobot_ros_control dobot_control.launch dry_run:=true
 *
 *   2) One-shot FSM query (read-only):
 *        rosrun dobot_ros_control dobot_cmd_vel_bridge --getfsm [ip] [port]
 *
 *   3) One-shot FSM state switch (actuates robot):
 *        rosrun dobot_ros_control dobot_cmd_vel_bridge --setfsm <id> [ip] [port]
 *      Note: FSM transitions must follow valid sequence (1 -> 100 -> 304).
 */
#include <ros/ros.h>

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "dobot_ros_control/dobot_bridge.h"
#include "dobot_ros_control/dobot_rpc.h"

int main(int argc, char** argv)
{
  // One-shot query mode
  if (argc >= 2 && std::strcmp(argv[1], "--getfsm") == 0)
  {
    const std::string ip = (argc >= 3) ? argv[2] : "192.168.8.234";
    const int port = (argc >= 4) ? std::atoi(argv[3]) : 51234;

    std::unique_ptr<dobot_ros_control::RpcClient> rpc;
    std::vector<std::string> backends;
    if (const char* forced = std::getenv("DOBOT_BACKEND")) backends.push_back(forced);
    else { backends.push_back("sdk"); backends.push_back("raw"); }
    for (const auto& backend : backends)
    {
      if (!dobot_ros_control::rpcBackendAvailable(backend)) continue;
      rpc = dobot_ros_control::makeRpcClient(backend);
      std::string err;
      std::printf("[--getfsm] trying backend=%s -> %s:%d ...\n", backend.c_str(), ip.c_str(), port);
      if (rpc->connect(ip, port, 3000, &err))
      {
        std::printf("[--getfsm] connected (backend=%s)\n", rpc->backendName());
        break;
      }
      std::printf("[--getfsm] connect failed: %s\n", err.c_str());
      rpc.reset();
    }
    if (!rpc)
    {
      std::fprintf(stderr, "[--getfsm] all backends failed; check robot ip/port/cable\n");
      return 2;
    }

    int32_t fsm_id = -1;
    auto rc = rpc->getFsmId(&fsm_id);
    std::printf("[--getfsm] GetFsmId -> rc=%s, fsm_id=%d\n",
                dobot_ros_control::rpcCodeString(rc), fsm_id);

    std::string list;
    rc = rpc->getFsmList(&list);
    if (rc == dobot_ros_control::RpcCode::kOk && !list.empty())
    {
      std::printf("[--getfsm] GetFsmList -> %s\n", list.c_str());
      std::printf("[--getfsm] Set fsm_prepare / fsm_walk in config/dobot_control.yaml accordingly.\n");
    }
    else
    {
      std::printf("[--getfsm] GetFsmList unavailable (rc=%s)\n",
                  dobot_ros_control::rpcCodeString(rc));
    }
    return 0;
  }

  // One-shot FSM switch mode
  if (argc >= 3 && std::strcmp(argv[1], "--setfsm") == 0)
  {
    const int target = std::atoi(argv[2]);
    const std::string ip = (argc >= 4) ? argv[3] : "192.168.8.234";
    const int port = (argc >= 5) ? std::atoi(argv[4]) : 51234;

    const char* note = "";
    switch (target)
    {
      case 1:   note = "kPassive (damping / emergency stop)"; break;
      case 100: note = "kReady (prepare standing - robot will stand up!)"; break;
      case 300: note = "kMiniStand (standing)"; break;
      case 304: note = "kAmpWalk (humanoid walking - requires 100 first)"; break;
      case 2:   note = "kDebug (low-level debug - do not use during navigation)"; break;
      default:  note = "(unknown FSM id)"; break;
    }
    std::printf("[--setfsm] target=%d %s\n", target, note);
    if (target != 1)
    {
      std::printf("[--setfsm] !! Ensure robot is supported/hoisted and area is clear; transition sequence: 1 -> 100 -> 304\n");
    }

    std::unique_ptr<dobot_ros_control::RpcClient> rpc;
    std::vector<std::string> backends;
    if (const char* forced = std::getenv("DOBOT_BACKEND")) backends.push_back(forced);
    else { backends.push_back("sdk"); backends.push_back("raw"); }
    for (const auto& backend : backends)
    {
      if (!dobot_ros_control::rpcBackendAvailable(backend)) continue;
      rpc = dobot_ros_control::makeRpcClient(backend);
      std::string err;
      if (rpc->connect(ip, port, 3000, &err)) break;
      std::printf("[--setfsm] backend=%s connect failed: %s\n", backend.c_str(), err.c_str());
      rpc.reset();
    }
    if (!rpc)
    {
      std::fprintf(stderr, "[--setfsm] cannot reach robot; check ip/port/cable\n");
      return 2;
    }

    int32_t cur = -1;
    rpc->getFsmId(&cur);
    std::printf("[--setfsm] before: fsm_id=%d\n", cur);

    auto rc = rpc->setControlMode(2);
    std::printf("[--setfsm] SetControlMode(2) -> %s\n", dobot_ros_control::rpcCodeString(rc));

    rc = rpc->setFsmId(target);
    std::printf("[--setfsm] SetFsmId(%d) -> %s\n", target, dobot_ros_control::rpcCodeString(rc));
    if (rc != dobot_ros_control::RpcCode::kOk)
    {
      std::fprintf(stderr, "[--setfsm] rejected; ensure intermediate states are reached first (e.g. 1 -> 100 -> 304)\n");
      return 3;
    }

    const int max_wait_ms = (target == 1) ? 3000 : 20000;
    int waited = 0;
    while (waited < max_wait_ms)
    {
      usleep(500 * 1000);
      waited += 500;
      if (rpc->getFsmId(&cur) == dobot_ros_control::RpcCode::kOk && cur == target)
      {
        std::printf("[--setfsm] OK: now fsm_id=%d (%.1fs)\n", cur, waited / 1000.0);
        return 0;
      }
      std::printf("[--setfsm] waiting... fsm_id=%d (%.1fs)\n", cur, waited / 1000.0);
    }
    std::fprintf(stderr, "[--setfsm] timeout: fsm_id=%d != %d; check robot front panel / logs\n",
                 cur, target);
    return 4;
  }

  ros::init(argc, argv, "dobot_cmd_vel_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  dobot_ros_control::DobotBridge bridge(nh, pnh);
  if (!bridge.init())
  {
    ROS_FATAL("[dobot] init failed, exiting (see log above)");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("[dobot] dobot_ros_control started");
  bridge.spin();
  ROS_INFO("[dobot] dobot_ros_control exited cleanly");
  return 0;
}
