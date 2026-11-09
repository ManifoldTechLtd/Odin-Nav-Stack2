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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_interfaces/FollowTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <string>
#include <utility>
#include <vector>

class SlidingMapNode {
public:
  SlidingMapNode();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  // Follow-target masking (disabled by default): masks out the followed person
  // and dynamic goal circles from the dynamic obstacle layer so the goal cell
  // does not fall inside the person's recent footprint trail.
  void followTargetCb(const nav_interfaces::FollowTarget::ConstPtr& msg);
  void dynamicGoalCb(const geometry_msgs::PoseStampedConstPtr& msg);
  bool maskActive(const double recv_wall_sec) const;
  bool insideActiveMask(double wx, double wy) const;
  void clearMaskCircle(double cx, double cy, double radius, const ros::Time& stamp);
  // Recenter the rolling grid window on (robot_x, robot_y) when the robot drifts
  // more than recenter_threshold_m_ from the current window center, shifting
  // existing cell data and timestamps instead of discarding them.
  void maybeRecenter(double robot_x, double robot_y);
  void updateMap(const sensor_msgs::LaserScan& scan, const tf2::Transform& tf_map_laser,
                 const ros::Time& stamp);
  void publishFakeScan(const ros::Time& stamp);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  void setFree(const std::vector<std::pair<int, int>>& cells, const ros::Time& stamp);
  void setOccupied(int mx, int my, const ros::Time& stamp);
  int index(int mx, int my) const;
  // Cells that haven't been refreshed for cell_decay_sec_ are reset to unknown.
  // Only occupied cells decay; free cells are kept permanently so once a corridor
  // has been observed clear it stays usable for fake-scan raycasts.
  void decayStaleCells(const ros::Time& now);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber follow_sub_;
  ros::Subscriber dynamic_goal_sub_;
  ros::Publisher map_pub_;
  ros::Publisher fake_scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::OccupancyGrid map_;
  double max_fake_range_;
  double range_min_;
  double ray_step_;
  int output_points_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string fake_scan_topic_;
  std::string map_topic_;

  // Per-cell last-update timestamps. Refreshed every time a cell is touched by
  // setFree/setOccupied. Used by decayStaleCells to expire cells that fell out
  // of the LiDAR FOV.
  std::vector<ros::Time> cell_stamp_;
  double cell_decay_sec_;          // <= 0 disables decay
  double decay_check_period_sec_;  // how often decayStaleCells actually scans
  ros::Time last_decay_check_;
  // Radial self-filter radius around the scanner origin (<= 0 disables).
  double footprint_radius_m_;
  // Rectangular footprint self-filter in footprint_frame_ (base_link):
  // |x| <= length/2 + margin and |y| <= width/2 + margin.
  bool use_footprint_polygon_{false};
  std::string footprint_frame_{"base_link"};
  double footprint_length_m_{0.96};
  double footprint_width_m_{0.55};
  double footprint_margin_m_{0.10};
  // Periodic hit-filtering counters
  long cnt_hits_{0};
  long cnt_drop_far_{0};
  long cnt_drop_footprint_{0};
  long cnt_drop_mask_{0};
  double trace_period_sec_{5.0};
  ros::Time last_trace_print_;
  double recenter_threshold_m_;

  // Follow-target masking state
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
