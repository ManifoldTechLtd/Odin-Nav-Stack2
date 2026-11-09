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

#include "atomic_move/atomic_move_node.h"

#include <algorithm>
#include <cmath>

#include <tf2/time.h>

AtomicMoveNode::AtomicMoveNode() : rclcpp::Node("atomic_move_node") {
  // Place all services and the stop subscription in a shared Reentrant callback group
  // so MultiThreadedExecutor(2) can deliver /neupan/stop while a service callback is executing.
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  linear_kp_ = this->declare_parameter("linear_kp", linear_kp_);
  linear_ki_ = this->declare_parameter("linear_ki", linear_ki_);
  linear_kd_ = this->declare_parameter("linear_kd", linear_kd_);
  linear_max_vel_ = this->declare_parameter("linear_max_vel", linear_max_vel_);
  linear_min_vel_ = this->declare_parameter("linear_min_vel", linear_min_vel_);
  linear_tolerance_ = this->declare_parameter("linear_tolerance", linear_tolerance_);
  linear_max_duration_ = this->declare_parameter("linear_max_duration", linear_max_duration_);
  linear_fine_window_ = this->declare_parameter("linear_fine_window", linear_fine_window_);
  linear_pulse_duration_ =
      this->declare_parameter("linear_pulse_duration", linear_pulse_duration_);
  linear_settle_duration_ =
      this->declare_parameter("linear_settle_duration", linear_settle_duration_);
  angular_kp_ = this->declare_parameter("angular_kp", angular_kp_);
  angular_ki_ = this->declare_parameter("angular_ki", angular_ki_);
  angular_kd_ = this->declare_parameter("angular_kd", angular_kd_);
  angular_max_vel_ = this->declare_parameter("angular_max_vel", angular_max_vel_);
  angular_min_vel_ = this->declare_parameter("angular_min_vel", angular_min_vel_);
  angular_tolerance_ = this->declare_parameter("angular_tolerance", angular_tolerance_);
  angular_fine_window_ = this->declare_parameter("angular_fine_window", angular_fine_window_);
  angular_pulse_duration_ =
      this->declare_parameter("angular_pulse_duration", angular_pulse_duration_);
  angular_settle_duration_ =
      this->declare_parameter("angular_settle_duration", angular_settle_duration_);
  angular_max_duration_ = this->declare_parameter("angular_max_duration", angular_max_duration_);
  angular_kick_duration_ = this->declare_parameter("angular_kick_duration", angular_kick_duration_);
  alignment_heading_tolerance_ =
      this->declare_parameter("alignment_heading_tolerance", alignment_heading_tolerance_);
  alignment_max_duration_ =
      this->declare_parameter("alignment_max_duration", alignment_max_duration_);
  control_frame_ = this->declare_parameter("control_frame", control_frame_);
  base_frame_ = this->declare_parameter("base_frame", base_frame_);
  control_frequency_ = this->declare_parameter("control_frequency", control_frequency_);
  stuck_timeout_ = this->declare_parameter("stuck_timeout", stuck_timeout_);
  linear_stuck_tolerance_ =
      this->declare_parameter("linear_stuck_tolerance", linear_stuck_tolerance_);
  angular_stuck_tolerance_ =
      this->declare_parameter("angular_stuck_tolerance", angular_stuck_tolerance_);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  move_service_ = this->create_service<atomic_move::srv::Move>(
      "~/move",
      std::bind(&AtomicMoveNode::moveCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);
  turn_service_ = this->create_service<atomic_move::srv::Turn>(
      "~/turn",
      std::bind(&AtomicMoveNode::turnCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);
  align_service_ = this->create_service<atomic_move::srv::AlignTo>(
      "~/align",
      std::bind(&AtomicMoveNode::alignCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;
  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/neupan/stop", rclcpp::QoS(1),
      std::bind(&AtomicMoveNode::stopCallback, this, std::placeholders::_1), sub_opts);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, true, tf2_ros::DynamicListenerQoS().best_effort());

  RCLCPP_INFO(this->get_logger(), "[atomic_move] control_frame=%s base_frame=%s",
              control_frame_.c_str(), base_frame_.c_str());
}

bool AtomicMoveNode::getTransform(TransformStamped& transform) {
  try {
    transform = tf_buffer_->lookupTransform(
        control_frame_, base_frame_, tf2::TimePointZero, std::chrono::milliseconds(1000));
    const rclcpp::Time stamp(transform.header.stamp);
    const double age = (this->now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > 1.0)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Robot pose TF is not fresh (age %.3fs, limit 1.0s)", age);
      return false;
    }
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
    return false;
  }
}

void AtomicMoveNode::moveCallback(
    const std::shared_ptr<atomic_move::srv::Move::Request> req,
    std::shared_ptr<atomic_move::srv::Move::Response> res) {
  const auto started_at = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "[atomic_move] move request distance=%.3f m frame=%s",
              req->distance, control_frame_.c_str());
  if (std::abs(req->distance) < linear_tolerance_) {
    res->success = true;
    res->message = "Distance within tolerance";
    return;
  }

  TransformStamped start_transform;
  if (!getTransform(start_transform)) {
    res->success = false;
    res->message = "MOVE_TRANSFORM_UNAVAILABLE";
    return;
  }
  const double start_yaw = tf2::getYaw(start_transform.transform.rotation);
  RCLCPP_INFO(this->get_logger(),
              "[atomic_move] move start frame=%s pose=(%.3f, %.3f) yaw=%.3f rad",
              control_frame_.c_str(), start_transform.transform.translation.x,
              start_transform.transform.translation.y, start_yaw);

  stop_requested_.store(false);
  const bool backward = req->distance < 0.0;
  const double target = std::abs(req->distance);
  double last_progress_x = start_transform.transform.translation.x;
  double last_progress_y = start_transform.transform.translation.y;
  auto last_progress_time = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(control_frequency_);
  double integral = 0.0;
  double previous_error = target;
  double distance_moved = 0.0;
  TransformStamped final_transform = start_transform;

  while (rclcpp::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res->success = false;
      res->message = "MOVE_CANCELLED";
      return;
    }
    if (elapsedSec(started_at) > linear_max_duration_) {
      stopMsg();
      res->success = false;
      res->message = "MOVE_TIMEOUT";
      return;
    }
    TransformStamped current_transform;
    if (!getTransform(current_transform)) {
      stopMsg();
      res->success = false;
      res->message = "MOVE_TRANSFORM_LOST";
      RCLCPP_ERROR(this->get_logger(),
                   "[atomic_move] move result success=false message='%s' elapsed=%.3f s",
                   res->message.c_str(), elapsedSec(started_at));
      return;
    }

    {
      const double ddx = current_transform.transform.translation.x - last_progress_x;
      const double ddy = current_transform.transform.translation.y - last_progress_y;
      if (std::hypot(ddx, ddy) > linear_stuck_tolerance_) {
        last_progress_x = current_transform.transform.translation.x;
        last_progress_y = current_transform.transform.translation.y;
        last_progress_time = std::chrono::steady_clock::now();
      } else if (elapsedSec(last_progress_time) > stuck_timeout_) {
        stopMsg();
        res->success = false;
        res->message = "MOVE_STUCK";
        RCLCPP_WARN(this->get_logger(),
                    "[atomic_move] move result success=false message='%s' elapsed=%.3f s",
                    res->message.c_str(), elapsedSec(started_at));
        return;
      }
    }

    final_transform = current_transform;
    {
      const double dx = current_transform.transform.translation.x -
                        start_transform.transform.translation.x;
      const double dy = current_transform.transform.translation.y -
                        start_transform.transform.translation.y;
      distance_moved = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
    }
    if (backward) {
      distance_moved = -distance_moved;
    }
    const double error = target - distance_moved;
    if (std::abs(error) <= linear_tolerance_) {
      stopMsg();
      sleepSec(linear_settle_duration_);
      TransformStamped settled_transform;
      if (!getTransform(settled_transform)) {
        res->success = false;
        res->message = "MOVE_SETTLE_TRANSFORM_LOST";
        return;
      }
      final_transform = settled_transform;
      {
        const double dx = settled_transform.transform.translation.x -
                          start_transform.transform.translation.x;
        const double dy = settled_transform.transform.translation.y -
                          start_transform.transform.translation.y;
        distance_moved = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
      }
      if (backward) {
        distance_moved = -distance_moved;
      }
      if (std::abs(target - distance_moved) <= linear_tolerance_) {
        break;
      }
      last_progress_time = std::chrono::steady_clock::now();
      continue;
    }

    integral += error / control_frequency_;
    integral = std::max(-1.0, std::min(1.0, integral));
    const double derivative = (error - previous_error) * control_frequency_;
    previous_error = error;
    double velocity = linear_kp_ * error + linear_ki_ * integral +
                      linear_kd_ * derivative;
    velocity = std::max(-linear_max_vel_, std::min(linear_max_vel_, velocity));
    if (std::abs(velocity) < linear_min_vel_) {
      velocity = std::copysign(linear_min_vel_, velocity);
    }
    if (backward) {
      velocity = -velocity;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = velocity;
    cmd_vel_pub_->publish(command);
    if (std::abs(error) <= linear_fine_window_) {
      sleepSec(linear_pulse_duration_);
      stopMsg();
      sleepSec(linear_settle_duration_);
    } else {
      rate.sleep();
    }
  }

  stopMsg();
  res->success = true;
  res->message = "Move completed";
  RCLCPP_INFO(this->get_logger(),
              "[atomic_move] move result success=true message='%s' moved=%.3f m "
              "final_odom=(%.3f, %.3f) yaw=%.3f rad elapsed=%.3f s",
              res->message.c_str(), distance_moved,
              final_transform.transform.translation.x,
              final_transform.transform.translation.y,
              tf2::getYaw(final_transform.transform.rotation), elapsedSec(started_at));
}

void AtomicMoveNode::turnCallback(
    const std::shared_ptr<atomic_move::srv::Turn::Request> req,
    std::shared_ptr<atomic_move::srv::Turn::Response> res) {
  const auto started_at = std::chrono::steady_clock::now();
  const double target = req->angle * M_PI / 180.0;
  RCLCPP_INFO(this->get_logger(), "[atomic_move] turn request angle=%.3f deg target=%.3f rad",
              req->angle, target);
  TransformStamped start_transform;
  if (!getTransform(start_transform)) {
    res->success = false;
    res->message = "TURN_TRANSFORM_UNAVAILABLE";
    RCLCPP_ERROR(this->get_logger(),
                 "[atomic_move] turn result success=false message='%s' elapsed=%.3f s",
                 res->message.c_str(), elapsedSec(started_at));
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "[atomic_move] turn start frame=%s pose=(%.3f, %.3f) yaw=%.3f rad",
              control_frame_.c_str(), start_transform.transform.translation.x,
              start_transform.transform.translation.y,
              tf2::getYaw(start_transform.transform.rotation));

  stop_requested_.store(false);
  double last_yaw = tf2::getYaw(start_transform.transform.rotation);
  double turned = 0.0;
  double last_progress_angle = 0.0;
  auto last_progress_time = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(control_frequency_);
  double integral = 0.0;
  double previous_error = target;
  TransformStamped final_transform = start_transform;

  while (rclcpp::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res->success = false;
      res->message = "TURN_CANCELLED";
      return;
    }
    if (elapsedSec(started_at) > angular_max_duration_) {
      stopMsg();
      res->success = false;
      res->message = "TURN_TIMEOUT";
      RCLCPP_WARN(this->get_logger(),
                  "[atomic_move] turn result success=false message='%s' requested=%.3f deg "
                  "turned=%.3f deg error=%.3f deg elapsed=%.3f s",
                  res->message.c_str(), req->angle, turned * 180.0 / M_PI,
                  (target - turned) * 180.0 / M_PI, elapsedSec(started_at));
      return;
    }
    TransformStamped current_transform;
    if (!getTransform(current_transform)) {
      stopMsg();
      res->success = false;
      res->message = "TURN_TRANSFORM_LOST";
      RCLCPP_ERROR(this->get_logger(),
                   "[atomic_move] turn result success=false message='%s' requested=%.3f deg "
                   "turned=%.3f deg elapsed=%.3f s",
                   res->message.c_str(), req->angle, turned * 180.0 / M_PI,
                   elapsedSec(started_at));
      return;
    }

    final_transform = current_transform;
    double delta = tf2::getYaw(current_transform.transform.rotation) - last_yaw;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    turned += delta;
    last_yaw += delta;
    while (last_yaw > M_PI) last_yaw -= 2.0 * M_PI;
    while (last_yaw < -M_PI) last_yaw += 2.0 * M_PI;

    const double error = target - turned;
    if (std::abs(turned - last_progress_angle) > angular_stuck_tolerance_) {
      last_progress_angle = turned;
      last_progress_time = std::chrono::steady_clock::now();
    } else if (elapsedSec(last_progress_time) > stuck_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "[atomic_move] turn has no progress; applying kick angular_z=%.3f "
                  "duration=%.3f s",
                  std::copysign(angular_max_vel_, error), angular_kick_duration_);
      geometry_msgs::msg::Twist kick;
      kick.angular.z = std::copysign(angular_max_vel_, error);
      const int kick_cycles = std::max(
          1, static_cast<int>(std::ceil(angular_kick_duration_ * control_frequency_)));
      for (int i = 0; i < kick_cycles && rclcpp::ok() && !stop_requested_.load(); ++i) {
        cmd_vel_pub_->publish(kick);
        rate.sleep();
      }
      stopMsg();
      last_progress_angle = turned;
      last_progress_time = std::chrono::steady_clock::now();
      continue;
    }
    if (std::abs(error) <= angular_tolerance_) {
      stopMsg();
      sleepSec(angular_settle_duration_);
      TransformStamped settled_transform;
      if (!getTransform(settled_transform)) {
        res->success = false;
        res->message = "TURN_SETTLE_TRANSFORM_LOST";
        return;
      }
      const double settled_yaw = tf2::getYaw(settled_transform.transform.rotation);
      double settled_delta = settled_yaw - last_yaw;
      while (settled_delta > M_PI) settled_delta -= 2.0 * M_PI;
      while (settled_delta < -M_PI) settled_delta += 2.0 * M_PI;
      turned += settled_delta;
      last_yaw += settled_delta;
      while (last_yaw > M_PI) last_yaw -= 2.0 * M_PI;
      while (last_yaw < -M_PI) last_yaw += 2.0 * M_PI;
      final_transform = settled_transform;
      if (std::abs(target - turned) <= angular_tolerance_) {
        break;
      }
      last_progress_time = std::chrono::steady_clock::now();
      continue;
    }

    integral += error / control_frequency_;
    integral = std::max(-1.0, std::min(1.0, integral));
    const double derivative = (error - previous_error) * control_frequency_;
    previous_error = error;
    double velocity = angular_kp_ * error + angular_ki_ * integral +
                      angular_kd_ * derivative;
    velocity = std::max(-angular_max_vel_, std::min(angular_max_vel_, velocity));
    if (std::abs(velocity) < angular_min_vel_) {
      velocity = std::copysign(angular_min_vel_, error);
    }

    geometry_msgs::msg::Twist command;
    command.angular.z = velocity;
    cmd_vel_pub_->publish(command);
    if (std::abs(error) <= angular_fine_window_) {
      sleepSec(angular_pulse_duration_);
      stopMsg();
      sleepSec(angular_settle_duration_);
    } else {
      rate.sleep();
    }
  }

  stopMsg();
  res->success = true;
  res->message = "Turn completed";
  RCLCPP_INFO(this->get_logger(),
              "[atomic_move] turn result success=true message='%s' requested=%.3f deg "
              "turned=%.3f deg final_yaw=%.3f rad elapsed=%.3f s",
              res->message.c_str(), req->angle, turned * 180.0 / M_PI,
              tf2::getYaw(final_transform.transform.rotation), elapsedSec(started_at));
}

void AtomicMoveNode::alignCallback(
    const std::shared_ptr<atomic_move::srv::AlignTo::Request> req,
    std::shared_ptr<atomic_move::srv::AlignTo::Response> res) {
  const auto started_at = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "[atomic_move] align request frame=%s target=(%.3f, %.3f, yaw=%.3f)",
              control_frame_.c_str(), req->x, req->y, req->yaw);
  stop_requested_.store(false);
  rclcpp::WallRate rate(control_frequency_);
  double linear_integral = 0.0;
  double angular_integral = 0.0;
  double previous_distance_error = 0.0;
  double previous_heading_error = 0.0;
  bool final_yaw_phase = false;
  double last_yaw_progress_error = 0.0;
  auto last_yaw_progress_time = std::chrono::steady_clock::now();
  // Kept so the ALIGN_TIMEOUT branch can report *why* it gave up without
  // needing an offline bag/log correlation pass afterwards.
  double last_distance_error = -1.0;
  double last_yaw_error = 0.0;

  while (rclcpp::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res->success = false;
      res->message = "ALIGN_CANCELLED";
      return;
    }
    if (elapsedSec(started_at) > alignment_max_duration_) {
      stopMsg();
      res->success = false;
      res->message = "ALIGN_TIMEOUT";
      RCLCPP_ERROR(this->get_logger(),
                   "[atomic_move] align result success=false message='ALIGN_TIMEOUT' "
                   "distance=%.3f (linear_tolerance=%.3f exit_tolerance=%.3f) "
                   "yaw_error=%.3f (angular_tolerance=%.3f) final_yaw_phase=%s "
                   "elapsed=%.3f s",
                   last_distance_error, linear_tolerance_, linear_tolerance_ * 1.5,
                   last_yaw_error, angular_tolerance_,
                   final_yaw_phase ? "true" : "false", elapsedSec(started_at));
      return;
    }

    TransformStamped current;
    if (!getTransform(current)) {
      stopMsg();
      res->success = false;
      res->message = "ALIGN_TRANSFORM_LOST";
      return;
    }
    const double current_x = current.transform.translation.x;
    const double current_y = current.transform.translation.y;
    const double current_yaw = tf2::getYaw(current.transform.rotation);
    const double dx = req->x - current_x;
    const double dy = req->y - current_y;
    const double distance_error = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_heading - current_yaw);
    const double yaw_error = normalizeAngle(req->yaw - current_yaw);
    last_distance_error = distance_error;
    last_yaw_error = yaw_error;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[atomic_move] align frame=%s current=(%.3f, %.3f, yaw=%.3f) "
                         "distance=%.3f heading_error=%.3f yaw_error=%.3f",
                         control_frame_.c_str(), current_x, current_y, current_yaw,
                         distance_error, heading_error, yaw_error);

    // Hysteresis: once the final yaw phase starts, keep it active until distance_error
    // exceeds distance_exit_tolerance (1.5x linear_tolerance_), and use the same
    // position tolerance for final acceptance to avoid a deadlock band.
    const double distance_exit_tolerance = linear_tolerance_ * 1.5;
    const double position_tolerance =
        final_yaw_phase ? distance_exit_tolerance : linear_tolerance_;

    if (distance_error <= position_tolerance &&
        std::abs(yaw_error) <= angular_tolerance_) {
      stopMsg();
      res->success = true;
      res->message = "Alignment completed";
      RCLCPP_INFO(this->get_logger(),
                  "[atomic_move] align result success=true distance=%.3f "
                  "(tolerance=%.3f final_yaw_phase=%s) yaw_error=%.3f elapsed=%.3f s",
                  distance_error, position_tolerance,
                  final_yaw_phase ? "true" : "false", yaw_error, elapsedSec(started_at));
      return;
    }

    geometry_msgs::msg::Twist command;
    const bool chase_point = final_yaw_phase
        ? (distance_error > distance_exit_tolerance)
        : (distance_error > linear_tolerance_);
    if (chase_point) {
      if (final_yaw_phase) {
        angular_integral = 0.0;
        previous_heading_error = heading_error;
        final_yaw_phase = false;
      }
      if (std::abs(heading_error) > alignment_heading_tolerance_) {
        angular_integral += heading_error / control_frequency_;
        angular_integral = std::max(-1.0, std::min(1.0, angular_integral));
        const double derivative =
            (heading_error - previous_heading_error) * control_frequency_;
        previous_heading_error = heading_error;
        double angular_velocity = angular_kp_ * heading_error +
                                  angular_ki_ * angular_integral +
                                  angular_kd_ * derivative;
        angular_velocity = std::max(-angular_max_vel_,
                                    std::min(angular_max_vel_, angular_velocity));
        if (std::abs(angular_velocity) < angular_min_vel_) {
          angular_velocity = std::copysign(angular_min_vel_, heading_error);
        }
        command.angular.z = angular_velocity;
      } else {
        linear_integral += distance_error / control_frequency_;
        linear_integral = std::max(-1.0, std::min(1.0, linear_integral));
        const double derivative =
            (distance_error - previous_distance_error) * control_frequency_;
        previous_distance_error = distance_error;
        double linear_velocity = linear_kp_ * distance_error +
                                 linear_ki_ * linear_integral +
                                 linear_kd_ * derivative;
        linear_velocity = std::max(0.0,
                                   std::min(linear_max_vel_, linear_velocity));
        if (linear_velocity < linear_min_vel_) {
          linear_velocity = linear_min_vel_;
        }
        command.linear.x = linear_velocity;
        if (distance_error <= linear_fine_window_) {
          cmd_vel_pub_->publish(command);
          sleepSec(linear_pulse_duration_);
          stopMsg();
          sleepSec(linear_settle_duration_);
          continue;
        }
      }
    } else {
      if (!final_yaw_phase) {
        angular_integral = 0.0;
        previous_heading_error = yaw_error;
        final_yaw_phase = true;
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = std::chrono::steady_clock::now();
      }
      if (std::abs(yaw_error - last_yaw_progress_error) > angular_stuck_tolerance_) {
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = std::chrono::steady_clock::now();
      } else if (elapsedSec(last_yaw_progress_time) > stuck_timeout_) {
        RCLCPP_WARN(this->get_logger(),
                    "[atomic_move] align final yaw has no progress; applying kick "
                    "angular_z=%.3f duration=%.3f s",
                    std::copysign(angular_max_vel_, yaw_error), angular_kick_duration_);
        geometry_msgs::msg::Twist kick;
        kick.angular.z = std::copysign(angular_max_vel_, yaw_error);
        const int kick_cycles = std::max(
            1, static_cast<int>(std::ceil(angular_kick_duration_ * control_frequency_)));
        for (int i = 0; i < kick_cycles && rclcpp::ok() && !stop_requested_.load(); ++i) {
          cmd_vel_pub_->publish(kick);
          rate.sleep();
        }
        stopMsg();
        sleepSec(angular_settle_duration_);
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = std::chrono::steady_clock::now();
        continue;
      }
      angular_integral += yaw_error / control_frequency_;
      angular_integral = std::max(-1.0, std::min(1.0, angular_integral));
      const double derivative =
          (yaw_error - previous_heading_error) * control_frequency_;
      previous_heading_error = yaw_error;
      double angular_velocity = angular_kp_ * yaw_error +
                                angular_ki_ * angular_integral +
                                angular_kd_ * derivative;
      angular_velocity = std::max(-angular_max_vel_,
                                  std::min(angular_max_vel_, angular_velocity));
      if (std::abs(angular_velocity) < angular_min_vel_) {
        angular_velocity = std::copysign(angular_min_vel_, yaw_error);
      }
      command.angular.z = angular_velocity;
    }

    cmd_vel_pub_->publish(command);
    rate.sleep();
  }

  stopMsg();
  res->success = false;
  res->message = "ALIGN_ROS_SHUTDOWN";
}

void AtomicMoveNode::stopMsg() {
  geometry_msgs::msg::Twist command;
  cmd_vel_pub_->publish(command);
}

void AtomicMoveNode::stopCallback(const std_msgs::msg::Empty::SharedPtr) {
  stop_requested_.store(true);
  stopMsg();
}

double AtomicMoveNode::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AtomicMoveNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
