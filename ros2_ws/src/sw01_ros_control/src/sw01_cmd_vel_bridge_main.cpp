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
 * @file sw01_cmd_vel_bridge_main.cpp
 * @brief Entry point of the sw01_cmd_vel_bridge node (ASTRALL SDK backend).
 */
#include "sw01_ros_control/sw01_bridge.h"

#include <csignal>
#include <thread>

namespace
{
sw01_ros_control::Sw01Bridge* g_bridge = nullptr;
volatile sig_atomic_t g_sigint_received = 0;

/** @brief Signal handler setting the stop flag so shutdown runs on the main thread. */
void sigintHandler(int /*sig*/)
{
  g_sigint_received = 1;
  if (g_bridge != nullptr)
  {
    g_bridge->requestStop();
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Install custom SIGINT/SIGTERM handler so zero velocity is sent before SDK deinitialization.
  rclcpp::uninstall_signal_handlers();

  auto node = std::make_shared<rclcpp::Node>("sw01_cmd_vel_bridge");
  sw01_ros_control::Sw01Bridge bridge(node, sw01_ros_control::makeAstrallChassisSdk());
  g_bridge = &bridge;
  signal(SIGINT, sigintHandler);
  signal(SIGTERM, sigintHandler);

  if (!bridge.init())
  {
    RCLCPP_ERROR(node->get_logger(), "sw01_cmd_vel_bridge 初始化失败，节点退出");
    g_bridge = nullptr;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bridge.spin();
  bridge.shutdown();

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  g_bridge = nullptr;
  rclcpp::shutdown();
  return 0;
}
