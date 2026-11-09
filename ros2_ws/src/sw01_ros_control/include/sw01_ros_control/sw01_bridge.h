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
 * @file sw01_bridge.h
 * @brief ROS 2 cmd_vel bridge for Hypertron SW01 quadruped robot via ASTRALL SDK.
 */
#ifndef SW01_ROS_CONTROL_SW01_BRIDGE_H
#define SW01_ROS_CONTROL_SW01_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

// ASTRALL SDK header (see sdk/README_SDK_SETUP.md)
#include "interface.h"

namespace sw01_ros_control
{

/** @brief Configurable node parameters */
struct BridgeParams
{
  std::string cmd_vel_topic = "/cmd_vel/throttle";  // Subscribed velocity topic
  double max_linear_vel = 1.0;                      // Linear velocity limit vx/vy (m/s)
  double max_angular_vel = 1.0;                     // Angular velocity limit vyaw (rad/s)
  double cmd_vel_timeout = 0.5;                     // Command timeout (s); sends zero velocity on expiry
  // Linear decay window before cmd_vel_timeout (s). 0.0 = disabled (recommended:
  // normal command interval jitter up to 200-500ms would otherwise trigger premature deceleration).
  double cmd_vel_decay_window = 0.0;
  double heartbeat_rate = 10.0;                     // Heartbeat frequency (Hz)
  double control_rate = 50.0;                       // Motion command dispatch frequency (Hz)
  int sdk_timeout = 10000;                          // SDK init timeout (ms)
  int auth_timeout = 2000;                          // Control authority request timeout (ms)
  int move_timeout = 20;                            // Single motion command timeout (ms)
  int heartbeat_timeout = 20;                       // Single heartbeat timeout (ms)
  bool request_control_authority = true;            // Request SDK control authority on startup
  bool require_move_mode = true;                    // Only send velocity commands when in MOVE mode
  bool auto_enter_move_mode = false;                // Automatically transition to MOVE mode (default false for safety)
  bool publish_status = true;                        // Publish SDK link and authority status topics
};

/**
 * @brief cmd_vel -> ASTRALL SDK bridge
 *
 * Threading model:
 *  - Main thread: motion command dispatch loop at control_rate
 *  - Executor thread: ROS 2 callbacks (cmd_vel subscription)
 *  - Heartbeat thread: calls AstrallHeartbeat() at heartbeat_rate
 *  - SDK callback thread: updates atomic link/authority flags
 * All SDK calls are serialized via sdk_mutex_.
 */
class Sw01Bridge
{
public:
  explicit Sw01Bridge(const rclcpp::Node::SharedPtr& node);
  ~Sw01Bridge();

  Sw01Bridge(const Sw01Bridge&) = delete;
  Sw01Bridge& operator=(const Sw01Bridge&) = delete;

  /** @brief Load parameters, initialize SDK, request authority, and create subscriptions. */
  bool init();

  /** @brief Main control loop dispatching motion commands at control_rate. */
  void spin();

  /** @brief Signal-safe stop request invoked by the SIGINT handler. */
  void requestStop();

  /** @brief Graceful shutdown: send zero velocity, stop threads, and uninitialize SDK. */
  void shutdown();

private:
  void loadParams();
  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
  void heartbeatLoop();

  /** @brief Send one motion command; returns SDK result code (or 0 if preconditions not met). */
  uint16_t sendMove(float vx, float vy, float vyaw);

  /** @brief Send repeated zero-velocity commands for immediate stop on exit or error. */
  void sendZeroVelocity(int repeat);

  /** @brief Request SDK control authority. */
  bool requestControlAuthority();

  /** @brief Check whether motion commands are allowed (and optionally transition to MOVE mode). */
  bool motionAllowed();

  void publishStatus();
  /** Publish diagnostic trace of SDK commands and bridge state. */
  void publishControlTrace(float vx, float vy, float vyaw, double cmd_age,
                           bool timed_out, bool motion_allowed,
                           bool move_attempted, uint16_t move_result);

  static double clampValue(double value, double limit);
  static const char* sportStatusToString(uint16_t status);
  static const char* resultToString(uint16_t res);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr authority_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sent_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_trace_pub_;

  BridgeParams params_;

  std::mutex cmd_mutex_;
  double cmd_vx_ = 0.0;
  double cmd_vy_ = 0.0;
  double cmd_vyaw_ = 0.0;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool cmd_received_ = false;

  std::mutex sdk_mutex_;

  std::thread heartbeat_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> sdk_initialized_{false};
  std::atomic<bool> shutdown_done_{false};
  std::atomic<bool> sdk_linked_{false};
  std::atomic<bool> has_authority_{false};
  std::atomic<bool> cmd_timed_out_{false};

  rclcpp::Time last_auth_retry_{0, 0, RCL_ROS_TIME};
};

}  // namespace sw01_ros_control

#endif  // SW01_ROS_CONTROL_SW01_BRIDGE_H
