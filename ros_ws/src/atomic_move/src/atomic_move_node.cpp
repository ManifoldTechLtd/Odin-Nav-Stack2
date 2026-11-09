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

AtomicMoveNode::AtomicMoveNode() : tf_listener_(), pnh_("~") {
  pnh_.param("linear_kp", linear_kp_, linear_kp_);
  pnh_.param("linear_ki", linear_ki_, linear_ki_);
  pnh_.param("linear_kd", linear_kd_, linear_kd_);
  pnh_.param("linear_max_vel", linear_max_vel_, linear_max_vel_);
  pnh_.param("linear_min_vel", linear_min_vel_, linear_min_vel_);
  pnh_.param("linear_tolerance", linear_tolerance_, linear_tolerance_);
  pnh_.param("linear_max_duration", linear_max_duration_, linear_max_duration_);
  pnh_.param("linear_fine_window", linear_fine_window_, linear_fine_window_);
  pnh_.param("linear_pulse_duration", linear_pulse_duration_, linear_pulse_duration_);
  pnh_.param("linear_settle_duration", linear_settle_duration_, linear_settle_duration_);
  pnh_.param("angular_kp", angular_kp_, angular_kp_);
  pnh_.param("angular_ki", angular_ki_, angular_ki_);
  pnh_.param("angular_kd", angular_kd_, angular_kd_);
  pnh_.param("angular_max_vel", angular_max_vel_, angular_max_vel_);
  pnh_.param("angular_min_vel", angular_min_vel_, angular_min_vel_);
  pnh_.param("angular_tolerance", angular_tolerance_, angular_tolerance_);
  pnh_.param("angular_fine_window", angular_fine_window_, angular_fine_window_);
  pnh_.param("angular_pulse_duration", angular_pulse_duration_, angular_pulse_duration_);
  pnh_.param("angular_settle_duration", angular_settle_duration_,
             angular_settle_duration_);
  pnh_.param("angular_max_duration", angular_max_duration_,
             angular_max_duration_);
  pnh_.param("angular_kick_duration", angular_kick_duration_,
             angular_kick_duration_);
  pnh_.param("alignment_heading_tolerance", alignment_heading_tolerance_,
             alignment_heading_tolerance_);
  pnh_.param("alignment_max_duration", alignment_max_duration_,
             alignment_max_duration_);
  pnh_.param("control_frame", control_frame_, control_frame_);
  pnh_.param("base_frame", base_frame_, base_frame_);
  pnh_.param("control_frequency", control_frequency_, control_frequency_);
  pnh_.param("stuck_timeout", stuck_timeout_, stuck_timeout_);
  pnh_.param("linear_stuck_tolerance", linear_stuck_tolerance_,
             linear_stuck_tolerance_);
  pnh_.param("angular_stuck_tolerance", angular_stuck_tolerance_,
             angular_stuck_tolerance_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  move_service_ = pnh_.advertiseService("move", &AtomicMoveNode::moveCallback, this);
  turn_service_ = pnh_.advertiseService("turn", &AtomicMoveNode::turnCallback, this);
  align_service_ = pnh_.advertiseService("align", &AtomicMoveNode::alignCallback, this);
  stop_sub_ = nh_.subscribe("/neupan/stop", 1,
                            &AtomicMoveNode::stopCallback, this);
  ROS_INFO("[atomic_move] control_frame=%s base_frame=%s", control_frame_.c_str(),
           base_frame_.c_str());
}

bool AtomicMoveNode::getTransform(tf::StampedTransform &transform) {
  try {
    tf_listener_.waitForTransform(control_frame_, base_frame_, ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener_.lookupTransform(control_frame_, base_frame_, ros::Time(0),
                                 transform);
    return true;
  } catch (const tf::TransformException &ex) {
    ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
    return false;
  }
}

bool AtomicMoveNode::moveCallback(atomic_move::Move::Request &req,
                                  atomic_move::Move::Response &res) {
  const ros::WallTime started_at = ros::WallTime::now();
  ROS_INFO("[atomic_move] move request distance=%.3f m frame=%s", req.distance,
           control_frame_.c_str());
  if (std::abs(req.distance) < linear_tolerance_) {
    res.success = true;
    res.message = "Distance within tolerance";
    return true;
  }

  tf::StampedTransform start_transform;
  if (!getTransform(start_transform)) {
    res.success = false;
    res.message = "MOVE_TRANSFORM_UNAVAILABLE";
    return true;
  }
  ROS_INFO("[atomic_move] move start frame=%s pose=(%.3f, %.3f) yaw=%.3f rad",
           control_frame_.c_str(), start_transform.getOrigin().x(),
           start_transform.getOrigin().y(), tf::getYaw(start_transform.getRotation()));

  stop_requested_.store(false);
  const bool backward = req.distance < 0.0;
  const double target = std::abs(req.distance);
  tf::Transform start_pose(start_transform.getRotation(),
                           start_transform.getOrigin());
  start_pose = start_pose.inverse();
  tf::Vector3 last_progress = start_transform.getOrigin();
  ros::Time last_progress_time = ros::Time::now();
  ros::Rate rate(control_frequency_);
  double integral = 0.0;
  double previous_error = target;
  double distance_moved = 0.0;
  tf::StampedTransform final_transform = start_transform;

  while (ros::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res.success = false;
      res.message = "MOVE_CANCELLED";
      return true;
    }
    if ((ros::WallTime::now() - started_at).toSec() > linear_max_duration_) {
      stopMsg();
      res.success = false;
      res.message = "MOVE_TIMEOUT";
      return true;
    }
    tf::StampedTransform current_transform;
    if (!getTransform(current_transform)) {
      stopMsg();
      res.success = false;
      res.message = "MOVE_TRANSFORM_LOST";
      ROS_ERROR("[atomic_move] move result success=false message='%s' elapsed=%.3f s",
                res.message.c_str(), (ros::WallTime::now() - started_at).toSec());
      return true;
    }

    if (last_progress.distance(current_transform.getOrigin()) >
        linear_stuck_tolerance_) {
      last_progress = current_transform.getOrigin();
      last_progress_time = ros::Time::now();
    } else if ((ros::Time::now() - last_progress_time).toSec() > stuck_timeout_) {
      stopMsg();
      res.success = false;
      res.message = "MOVE_STUCK";
      ROS_WARN("[atomic_move] move result success=false message='%s' elapsed=%.3f s",
               res.message.c_str(), (ros::WallTime::now() - started_at).toSec());
      return true;
    }

    final_transform = current_transform;
    distance_moved = (start_pose * current_transform.getOrigin()).x();
    if (backward) {
      distance_moved = -distance_moved;
    }
    const double error = target - distance_moved;
    if (std::abs(error) <= linear_tolerance_) {
      stopMsg();
      ros::Duration(linear_settle_duration_).sleep();
      tf::StampedTransform settled_transform;
      if (!getTransform(settled_transform)) {
        res.success = false;
        res.message = "MOVE_SETTLE_TRANSFORM_LOST";
        return true;
      }
      final_transform = settled_transform;
      distance_moved = (start_pose * settled_transform.getOrigin()).x();
      if (backward) {
        distance_moved = -distance_moved;
      }
      if (std::abs(target - distance_moved) <= linear_tolerance_) {
        break;
      }
      last_progress_time = ros::Time::now();
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

    geometry_msgs::Twist command;
    command.linear.x = velocity;
    cmd_vel_pub_.publish(command);
    if (std::abs(error) <= linear_fine_window_) {
      ros::Duration(linear_pulse_duration_).sleep();
      stopMsg();
      ros::Duration(linear_settle_duration_).sleep();
    } else {
      rate.sleep();
    }
  }

  stopMsg();
  res.success = true;
  res.message = "Move completed";
  ROS_INFO("[atomic_move] move result success=true message='%s' moved=%.3f m final_odom=(%.3f, %.3f) yaw=%.3f rad elapsed=%.3f s",
           res.message.c_str(), distance_moved, final_transform.getOrigin().x(),
           final_transform.getOrigin().y(), tf::getYaw(final_transform.getRotation()),
           (ros::WallTime::now() - started_at).toSec());
  return true;
}

bool AtomicMoveNode::turnCallback(atomic_move::Turn::Request &req,
                                  atomic_move::Turn::Response &res) {
  const ros::WallTime started_at = ros::WallTime::now();
  const double target = req.angle * M_PI / 180.0;
  ROS_INFO("[atomic_move] turn request angle=%.3f deg target=%.3f rad", req.angle,
           target);
  tf::StampedTransform start_transform;
  if (!getTransform(start_transform)) {
    res.success = false;
    res.message = "TURN_TRANSFORM_UNAVAILABLE";
    ROS_ERROR("[atomic_move] turn result success=false message='%s' elapsed=%.3f s",
              res.message.c_str(), (ros::WallTime::now() - started_at).toSec());
    return true;
  }
  ROS_INFO("[atomic_move] turn start frame=%s pose=(%.3f, %.3f) yaw=%.3f rad",
           control_frame_.c_str(), start_transform.getOrigin().x(),
           start_transform.getOrigin().y(), tf::getYaw(start_transform.getRotation()));

  stop_requested_.store(false);
  double last_yaw = tf::getYaw(start_transform.getRotation());
  double turned = 0.0;
  double last_progress_angle = 0.0;
  ros::Time last_progress_time = ros::Time::now();
  ros::Rate rate(control_frequency_);
  double integral = 0.0;
  double previous_error = target;
  tf::StampedTransform final_transform = start_transform;

  while (ros::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res.success = false;
      res.message = "TURN_CANCELLED";
      return true;
    }
    if ((ros::WallTime::now() - started_at).toSec() > angular_max_duration_) {
      stopMsg();
      res.success = false;
      res.message = "TURN_TIMEOUT";
      ROS_WARN("[atomic_move] turn result success=false message='%s' requested=%.3f deg turned=%.3f deg error=%.3f deg elapsed=%.3f s",
               res.message.c_str(), req.angle, turned * 180.0 / M_PI,
               (target - turned) * 180.0 / M_PI,
               (ros::WallTime::now() - started_at).toSec());
      return true;
    }
    tf::StampedTransform current_transform;
    if (!getTransform(current_transform)) {
      stopMsg();
      res.success = false;
      res.message = "TURN_TRANSFORM_LOST";
      ROS_ERROR("[atomic_move] turn result success=false message='%s' requested=%.3f deg turned=%.3f deg elapsed=%.3f s",
                res.message.c_str(), req.angle, turned * 180.0 / M_PI,
                (ros::WallTime::now() - started_at).toSec());
      return true;
    }

    final_transform = current_transform;
    double delta = tf::getYaw(current_transform.getRotation()) - last_yaw;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    turned += delta;
    last_yaw += delta;
    while (last_yaw > M_PI) last_yaw -= 2.0 * M_PI;
    while (last_yaw < -M_PI) last_yaw += 2.0 * M_PI;

    const double error = target - turned;
    if (std::abs(turned - last_progress_angle) > angular_stuck_tolerance_) {
      last_progress_angle = turned;
      last_progress_time = ros::Time::now();
    } else if ((ros::Time::now() - last_progress_time).toSec() > stuck_timeout_) {
      ROS_WARN("[atomic_move] turn has no progress; applying kick angular_z=%.3f duration=%.3f s",
               std::copysign(angular_max_vel_, error), angular_kick_duration_);
      geometry_msgs::Twist kick;
      kick.angular.z = std::copysign(angular_max_vel_, error);
      const int kick_cycles = std::max(
          1, static_cast<int>(std::ceil(angular_kick_duration_ * control_frequency_)));
      for (int i = 0; i < kick_cycles && ros::ok() && !stop_requested_.load(); ++i) {
        cmd_vel_pub_.publish(kick);
        rate.sleep();
      }
      stopMsg();
      last_progress_angle = turned;
      last_progress_time = ros::Time::now();
      continue;
    }
    if (std::abs(error) <= angular_tolerance_) {
      stopMsg();
      ros::Duration(angular_settle_duration_).sleep();
      tf::StampedTransform settled_transform;
      if (!getTransform(settled_transform)) {
        res.success = false;
        res.message = "TURN_SETTLE_TRANSFORM_LOST";
        return true;
      }
      const double settled_yaw = tf::getYaw(settled_transform.getRotation());
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
      last_progress_time = ros::Time::now();
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

    geometry_msgs::Twist command;
    command.angular.z = velocity;
    cmd_vel_pub_.publish(command);
    if (std::abs(error) <= angular_fine_window_) {
      ros::Duration(angular_pulse_duration_).sleep();
      stopMsg();
      ros::Duration(angular_settle_duration_).sleep();
    } else {
      rate.sleep();
    }
  }

  stopMsg();
  res.success = true;
  res.message = "Turn completed";
  ROS_INFO("[atomic_move] turn result success=true message='%s' requested=%.3f deg turned=%.3f deg final_yaw=%.3f rad elapsed=%.3f s",
           res.message.c_str(), req.angle, turned * 180.0 / M_PI,
           tf::getYaw(final_transform.getRotation()),
           (ros::WallTime::now() - started_at).toSec());
  return true;
}

bool AtomicMoveNode::alignCallback(atomic_move::AlignTo::Request &req,
                                   atomic_move::AlignTo::Response &res) {
  const ros::WallTime started_at = ros::WallTime::now();
  ROS_INFO("[atomic_move] align request frame=%s target=(%.3f, %.3f, yaw=%.3f)",
           control_frame_.c_str(), req.x, req.y, req.yaw);
  stop_requested_.store(false);
  ros::Rate rate(control_frequency_);
  double linear_integral = 0.0;
  double angular_integral = 0.0;
  double previous_distance_error = 0.0;
  double previous_heading_error = 0.0;
  bool final_yaw_phase = false;
  double last_yaw_progress_error = 0.0;
  ros::Time last_yaw_progress_time = ros::Time::now();
  // Kept so the ALIGN_TIMEOUT branch can report *why* it gave up without
  // needing an offline bag/log correlation pass afterwards.
  double last_distance_error = -1.0;
  double last_yaw_error = 0.0;

  while (ros::ok()) {
    if (stop_requested_.load()) {
      stopMsg();
      res.success = false;
      res.message = "ALIGN_CANCELLED";
      return true;
    }
    if ((ros::WallTime::now() - started_at).toSec() > alignment_max_duration_) {
      stopMsg();
      res.success = false;
      res.message = "ALIGN_TIMEOUT";
      ROS_ERROR("[atomic_move] align result success=false message='ALIGN_TIMEOUT' "
                "distance=%.3f (linear_tolerance=%.3f exit_tolerance=%.3f) "
                "yaw_error=%.3f (angular_tolerance=%.3f) final_yaw_phase=%s "
                "elapsed=%.3f s",
                last_distance_error, linear_tolerance_, linear_tolerance_ * 1.5,
                last_yaw_error, angular_tolerance_,
                final_yaw_phase ? "true" : "false",
                (ros::WallTime::now() - started_at).toSec());
      return true;
    }

    tf::StampedTransform current;
    if (!getTransform(current)) {
      stopMsg();
      res.success = false;
      res.message = "ALIGN_TRANSFORM_LOST";
      return true;
    }
    const double current_x = current.getOrigin().x();
    const double current_y = current.getOrigin().y();
    const double current_yaw = tf::getYaw(current.getRotation());
    const double dx = req.x - current_x;
    const double dy = req.y - current_y;
    const double distance_error = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_heading - current_yaw);
    const double yaw_error = normalizeAngle(req.yaw - current_yaw);
    last_distance_error = distance_error;
    last_yaw_error = yaw_error;

    ROS_INFO_THROTTLE(0.5,
                      "[atomic_move] align frame=%s current=(%.3f, %.3f, yaw=%.3f) "
                      "distance=%.3f heading_error=%.3f yaw_error=%.3f",
                      control_frame_.c_str(), current_x, current_y, current_yaw,
                      distance_error, heading_error, yaw_error);

    // Hysteresis: once the final yaw phase has started, do not fall back to
    // chasing the target point unless distance_error drifts past
    // distance_exit_tolerance (1.5x linear_tolerance_). Use the same threshold
    // for acceptance so the final-yaw phase cannot deadlock in the band
    // (linear_tolerance_, distance_exit_tolerance].
    const double distance_exit_tolerance = linear_tolerance_ * 1.5;
    const double position_tolerance =
        final_yaw_phase ? distance_exit_tolerance : linear_tolerance_;

    if (distance_error <= position_tolerance &&
        std::abs(yaw_error) <= angular_tolerance_) {
      stopMsg();
      res.success = true;
      res.message = "Alignment completed";
      ROS_INFO("[atomic_move] align result success=true distance=%.3f "
               "(tolerance=%.3f final_yaw_phase=%s) yaw_error=%.3f elapsed=%.3f s",
               distance_error, position_tolerance,
               final_yaw_phase ? "true" : "false", yaw_error,
               (ros::WallTime::now() - started_at).toSec());
      return true;
    }

    geometry_msgs::Twist command;
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
          cmd_vel_pub_.publish(command);
          ros::Duration(linear_pulse_duration_).sleep();
          stopMsg();
          ros::Duration(linear_settle_duration_).sleep();
          continue;
        }
      }
    } else {
      if (!final_yaw_phase) {
        angular_integral = 0.0;
        previous_heading_error = yaw_error;
        final_yaw_phase = true;
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = ros::Time::now();
      }
      // Apply stuck-detection kick if final yaw stalls within stuck_timeout_.
      if (std::abs(yaw_error - last_yaw_progress_error) > angular_stuck_tolerance_) {
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = ros::Time::now();
      } else if ((ros::Time::now() - last_yaw_progress_time).toSec() > stuck_timeout_) {
        ROS_WARN("[atomic_move] align final yaw has no progress; applying kick angular_z=%.3f duration=%.3f s",
                 std::copysign(angular_max_vel_, yaw_error), angular_kick_duration_);
        geometry_msgs::Twist kick;
        kick.angular.z = std::copysign(angular_max_vel_, yaw_error);
        const int kick_cycles = std::max(
            1, static_cast<int>(std::ceil(angular_kick_duration_ * control_frequency_)));
        for (int i = 0; i < kick_cycles && ros::ok() && !stop_requested_.load(); ++i) {
          cmd_vel_pub_.publish(kick);
          rate.sleep();
        }
        stopMsg();
        ros::Duration(angular_settle_duration_).sleep();
        last_yaw_progress_error = yaw_error;
        last_yaw_progress_time = ros::Time::now();
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

    cmd_vel_pub_.publish(command);
    rate.sleep();
  }

  stopMsg();
  res.success = false;
  res.message = "ALIGN_ROS_SHUTDOWN";
  return true;
}

void AtomicMoveNode::stopMsg() {
  geometry_msgs::Twist command;
  cmd_vel_pub_.publish(command);
}

void AtomicMoveNode::stopCallback(const std_msgs::EmptyConstPtr &) {
  stop_requested_.store(true);
  stopMsg();
}

double AtomicMoveNode::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "atomic_move_node");
  AtomicMoveNode node;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
