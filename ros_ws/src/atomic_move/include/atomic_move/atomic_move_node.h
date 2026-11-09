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

#include <atomic>
#include <cmath>
#include <string>

#include <atomic_move/AlignTo.h>
#include <atomic_move/Move.h>
#include <atomic_move/Turn.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>

class AtomicMoveNode {
public:
  AtomicMoveNode();

private:
  bool getTransform(tf::StampedTransform &transform);
  bool moveCallback(atomic_move::Move::Request &req,
                    atomic_move::Move::Response &res);
  bool turnCallback(atomic_move::Turn::Request &req,
                    atomic_move::Turn::Response &res);
  bool alignCallback(atomic_move::AlignTo::Request &req,
                     atomic_move::AlignTo::Response &res);
  void stopMsg();
  static double normalizeAngle(double angle);
  void stopCallback(const std_msgs::EmptyConstPtr &message);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_vel_pub_;
  ros::ServiceServer move_service_;
  ros::ServiceServer turn_service_;
  ros::ServiceServer align_service_;
  ros::Subscriber stop_sub_;
  tf::TransformListener tf_listener_;
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
  // Must be >= the smallest yaw correction Go2 can execute cleanly (~15 deg).
  // Keep <= alignment_heading_tolerance_ (0.35) so the approach-phase heading
  // gate is never tighter than final-yaw acceptance.
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
  // Set above the ~0.018 rad map->base_link TF yaw jitter floor so stationary
  // SLAM noise does not mask genuine angular stall.
  double angular_stuck_tolerance_{0.03};
};

#endif
