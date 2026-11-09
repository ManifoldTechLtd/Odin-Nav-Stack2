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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_interfaces/msg/follow_target.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class SlidingMapNode : public rclcpp::Node {
public:
  enum CellMemoryState : uint8_t {
    kCellStatic = 0,
    kCellDynamic = 1,
    kCellVacatedDynamic = 2,
  };

  SlidingMapNode();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void dynamicScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // Follow-target obstacle masking (disabled by default): excludes the tracked
  // target and dynamic_goal circles from the dynamic obstacle layer.
  void followTargetCb(const nav_interfaces::msg::FollowTarget::SharedPtr msg);
  void dynamicGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool maskActive(const double recv_wall_sec) const;
  bool insideActiveMask(double wx, double wy) const;
  void clearMaskCircle(double cx, double cy, double radius, const rclcpp::Time& stamp);
  // Recenters the rolling grid window on (robot_x, robot_y) once it drifts more
  // than recenter_threshold_m_ from the current window center, shifting existing
  // cell data and timestamps without discarding them.
  void maybeRecenter(double robot_x, double robot_y);
  void updateMap(const sensor_msgs::msg::LaserScan& scan, const tf2::Transform& tf_map_laser,
                 const rclcpp::Time& stamp);
  void publishFakeScan(const rclcpp::Time& stamp);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  void setFree(const std::vector<std::pair<int, int>>& cells, const rclcpp::Time& stamp);
  void setOccupied(int mx, int my, const rclcpp::Time& stamp, bool is_dynamic = false);
  int index(int mx, int my) const;
  // Resets occupied cells that have not been refreshed for cell_decay_sec_ back to unknown.
  void decayStaleCells(const rclcpp::Time& now);

  static bool timeIsZero(const rclcpp::Time& t) { return t.nanoseconds() == 0; }
  static rclcpp::Time zeroTime() { return rclcpp::Time(0, 0, RCL_ROS_TIME); }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr dynamic_scan_sub_;
  rclcpp::Subscription<nav_interfaces::msg::FollowTarget>::SharedPtr follow_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dynamic_goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_scan_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid map_;
  double max_fake_range_;
  double range_min_;
  double ray_step_;
  int output_points_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string fake_scan_topic_;
  std::string map_topic_;

  std::vector<rclcpp::Time> cell_stamp_;
  std::vector<uint8_t> cell_state_;
  std::vector<rclcpp::Time> cell_dyn_since_;
  std::vector<uint8_t> cell_seen_free_;
  double cell_decay_sec_;          // <= 0 disables decay
  double decay_check_period_sec_;
  rclcpp::Time last_decay_check_;
  bool dynamic_memory_enable_{false};
  double dynamic_decay_sec_{15.0};
  double dynamic_min_hold_sec_{0.0};
  std::string dynamic_input_topic_{""};
  bool dynamic_require_vacated_{true};
  bool footprint_clear_enable_{false};
  sensor_msgs::msg::LaserScan::SharedPtr latest_dynamic_scan_;
  long cnt_decay_static_{0};
  long cnt_decay_dynamic_{0};
  long cnt_dropped_early_{0};
  // Radial self-filter fallback radius (<= 0 disables)
  double footprint_radius_m_;
  // Rectangular footprint self-filter in footprint_frame_ (base_link)
  bool use_footprint_polygon_{false};
  std::string footprint_frame_{"base_link"};
  double footprint_length_m_{0.96};
  double footprint_width_m_{0.55};
  double footprint_margin_m_{0.10};
  long cnt_hits_{0};
  long cnt_drop_far_{0};
  long cnt_drop_footprint_{0};
  long cnt_drop_mask_{0};
  double trace_period_sec_{5.0};
  rclcpp::Time last_trace_print_;
  double recenter_threshold_m_;

  // Follow-target obstacle masking (disabled by default)
  bool follow_target_enable_{false};
  std::string follow_target_topic_{"/waypoint_manager/follow_target"};
  std::string dynamic_goal_topic_{"/waypoint_manager/dynamic_goal"};
  double follow_stale_sec_{2.0};
  double follow_person_radius_{0.4};
  double follow_goal_radius_{0.5};
  double follow_person_x_{0.0}, follow_person_y_{0.0};
  double follow_goal_x_{0.0}, follow_goal_y_{0.0};
  double follow_person_recv_{-1.0};
  double follow_goal_recv_{-1.0};
};
