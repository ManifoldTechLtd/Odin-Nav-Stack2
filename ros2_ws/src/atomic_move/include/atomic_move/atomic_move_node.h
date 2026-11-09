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

#ifndef ATOMIC_MOVE_NODE_H
#define ATOMIC_MOVE_NODE_H

// ROS 2 Jazzy port of atomic_move.
// Uses a MultiThreadedExecutor(2) and a shared Reentrant callback group for the
// three services (~/move, ~/turn, ~/align) and the /neupan/stop subscription so
// stop commands can interrupt an active control loop.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic_move/srv/align_to.hpp>
#include <atomic_move/srv/move.hpp>
#include <atomic_move/srv/turn.hpp>

class AtomicMoveNode : public rclcpp::Node {
public:
  AtomicMoveNode();

private:
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  bool getTransform(TransformStamped& transform);
  void moveCallback(const std::shared_ptr<atomic_move::srv::Move::Request> req,
                    std::shared_ptr<atomic_move::srv::Move::Response> res);
  void turnCallback(const std::shared_ptr<atomic_move::srv::Turn::Request> req,
                    std::shared_ptr<atomic_move::srv::Turn::Response> res);
  void alignCallback(const std::shared_ptr<atomic_move::srv::AlignTo::Request> req,
                     std::shared_ptr<atomic_move::srv::AlignTo::Response> res);
  void stopMsg();
  static double normalizeAngle(double angle);
  void stopCallback(const std_msgs::msg::Empty::SharedPtr message);

  static double elapsedSec(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  }
  static void sleepSec(double seconds) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds)));
  }

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<atomic_move::srv::Move>::SharedPtr move_service_;
  rclcpp::Service<atomic_move::srv::Turn>::SharedPtr turn_service_;
  rclcpp::Service<atomic_move::srv::AlignTo>::SharedPtr align_service_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::atomic<bool> stop_requested_{false};
  double linear_kp_{1.0};
  double linear_ki_{0.02};
  double linear_kd_{0.02};
  double linear_max_vel_{0.5};
  double linear_min_vel_{0.15};
  double linear_tolerance_{0.20};
  double linear_max_duration_{30.0};
  double linear_fine_window_{0.12};
  double linear_pulse_duration_{0.08};
  double linear_settle_duration_{0.15};
  double angular_kp_{1.0};
  double angular_ki_{0.01};
  double angular_kd_{0.03};
  double angular_max_vel_{1.0};
  double angular_min_vel_{0.45};
  // Must be >= the smallest yaw correction the robot can execute cleanly (~15-20 deg on Go2).
  // Keep <= alignment_heading_tolerance_ (0.35).
  double angular_tolerance_{0.3491};
  double angular_fine_window_{0.35};
  double angular_pulse_duration_{0.08};
  double angular_settle_duration_{0.15};
  double angular_max_duration_{30.0};
  double angular_kick_duration_{0.25};
  double alignment_heading_tolerance_{0.35};
  double alignment_max_duration_{20.0};
  std::string control_frame_{"map"};
  std::string base_frame_{"base_link"};
  double control_frequency_{20.0};
  double stuck_timeout_{2.0};
  double linear_stuck_tolerance_{0.01};
  // Above the stationary SLAM yaw jitter floor (~0.018 rad) so stuck-kick triggers reliably.
  double angular_stuck_tolerance_{0.03};
};

#endif
