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

#include "sliding_map.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

SlidingMapNode::SlidingMapNode() : rclcpp::Node("sliding_map_node") {
  map_frame_ = this->declare_parameter("map_frame", std::string("odom"));
  base_frame_ = this->declare_parameter("base_frame", std::string("base_link"));
  scan_topic_ = this->declare_parameter("input_scan_topic", std::string("/scan_input"));
  fake_scan_topic_ = this->declare_parameter("fake_scan_topic", std::string("/scan_360"));
  map_topic_ = this->declare_parameter("map_topic", std::string("/sliding_map"));
  max_fake_range_ = this->declare_parameter("max_fake_range", 30.0);
  range_min_ = this->declare_parameter("range_min", 0.05);
  output_points_ = static_cast<int>(std::max<int64_t>(1, this->declare_parameter("output_points", 720L)));
  double resolution = this->declare_parameter("resolution", 0.05);
  int width = this->declare_parameter("width", 400);
  int height = this->declare_parameter("height", 400);
  double origin_x = this->declare_parameter("origin_x", -width * resolution / 2.0);
  double origin_y = this->declare_parameter("origin_y", -height * resolution / 2.0);

  map_.header.frame_id = map_frame_;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);

  ray_step_ = std::max(0.01, resolution * 0.5);

  cell_decay_sec_ = this->declare_parameter("cell_decay_sec", 6.0);
  decay_check_period_sec_ = this->declare_parameter("decay_check_period_sec", 0.5);
  footprint_radius_m_ = this->declare_parameter("footprint_radius_m", 0.35);
  use_footprint_polygon_ = this->declare_parameter("use_footprint_polygon", false);
  footprint_frame_ = this->declare_parameter("footprint_frame", std::string("base_link"));
  footprint_length_m_ = this->declare_parameter("footprint_length_m", 0.96);
  footprint_width_m_ = this->declare_parameter("footprint_width_m", 0.55);
  footprint_margin_m_ = this->declare_parameter("footprint_margin_m", 0.10);
  trace_period_sec_ = this->declare_parameter("trace_period_sec", 5.0);
  recenter_threshold_m_ = this->declare_parameter(
      "recenter_threshold_m", std::min(width, height) * resolution / 4.0);
  dynamic_memory_enable_ = this->declare_parameter("dynamic_memory_enable", false);
  dynamic_decay_sec_ = this->declare_parameter("dynamic_decay_sec", cell_decay_sec_);
  dynamic_min_hold_sec_ = this->declare_parameter("dynamic_min_hold_sec", 0.0);
  dynamic_input_topic_ = this->declare_parameter("dynamic_input_topic", std::string(""));
  dynamic_require_vacated_ = this->declare_parameter("dynamic_require_vacated", true);
  footprint_clear_enable_ = this->declare_parameter("footprint_clear_enable", false);

  const size_t total_cells = static_cast<size_t>(width) * static_cast<size_t>(height);
  cell_stamp_.assign(total_cells, zeroTime());
  cell_state_.assign(total_cells, kCellStatic);
  cell_dyn_since_.assign(total_cells, zeroTime());
  cell_seen_free_.assign(total_cells, 0);
  last_decay_check_ = zeroTime();
  last_trace_print_ = zeroTime();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, true, tf2_ros::DynamicListenerQoS().best_effort());

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local());
  fake_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      fake_scan_topic_, rclcpp::QoS(5));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(5),
      std::bind(&SlidingMapNode::scanCallback, this, std::placeholders::_1));
  if (dynamic_memory_enable_ && !dynamic_input_topic_.empty()) {
    dynamic_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        dynamic_input_topic_, rclcpp::QoS(5),
        std::bind(&SlidingMapNode::dynamicScanCallback, this, std::placeholders::_1));
  }

  follow_target_enable_ = this->declare_parameter("follow_target_enable", false);
  follow_target_topic_ =
      this->declare_parameter("follow_target_topic", follow_target_topic_);
  dynamic_goal_topic_ =
      this->declare_parameter("dynamic_goal_topic", dynamic_goal_topic_);
  follow_stale_sec_ = this->declare_parameter("follow_stale_sec", follow_stale_sec_);
  follow_person_radius_ = this->declare_parameter("follow_person_radius", follow_person_radius_);
  follow_goal_radius_ = this->declare_parameter("follow_goal_radius", follow_goal_radius_);
  if (follow_target_enable_) {
    follow_sub_ = this->create_subscription<nav_interfaces::msg::FollowTarget>(
        follow_target_topic_, 10,
        std::bind(&SlidingMapNode::followTargetCb, this, std::placeholders::_1));
    dynamic_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        dynamic_goal_topic_, 10,
        std::bind(&SlidingMapNode::dynamicGoalCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "[sliding_map] follow-target masking ENABLED: person_r=%.2f goal_r=%.2f "
                "stale=%.1fs",
                follow_person_radius_, follow_goal_radius_, follow_stale_sec_);
  } else {
    RCLCPP_INFO(this->get_logger(), "[sliding_map] follow-target masking disabled (default)");
  }
}

void SlidingMapNode::dynamicScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  latest_dynamic_scan_ = msg;
}

void SlidingMapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  const rclcpp::Time stamp(msg->header.stamp);
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, stamp,
                                         rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_map_laser;
  tf2::fromMsg(tf_msg.transform, tf_map_laser);

  const tf2::Vector3 laser_origin = tf_map_laser.getOrigin();
  maybeRecenter(laser_origin.x(), laser_origin.y());

  updateMap(*msg, tf_map_laser, stamp);

  if (follow_target_enable_) {
    if (follow_person_recv_ >= 0.0 && maskActive(follow_person_recv_)) {
      clearMaskCircle(follow_person_x_, follow_person_y_,
                      follow_person_radius_, stamp);
    }
    if (follow_goal_recv_ >= 0.0 && maskActive(follow_goal_recv_)) {
      clearMaskCircle(follow_goal_x_, follow_goal_y_,
                      follow_goal_radius_, stamp);
    }
  }

  decayStaleCells(stamp);

  map_.header.stamp = stamp;
  map_.info.map_load_time = stamp;
  map_pub_->publish(map_);

  publishFakeScan(stamp);
}

void SlidingMapNode::maybeRecenter(double robot_x, double robot_y) {
  const double half_w = map_.info.width * map_.info.resolution / 2.0;
  const double half_h = map_.info.height * map_.info.resolution / 2.0;
  const double center_x = map_.info.origin.position.x + half_w;
  const double center_y = map_.info.origin.position.y + half_h;

  if (std::abs(robot_x - center_x) < recenter_threshold_m_ &&
      std::abs(robot_y - center_y) < recenter_threshold_m_) {
    return;
  }

  const double new_origin_x = robot_x - half_w;
  const double new_origin_y = robot_y - half_h;
  const int shift_x = static_cast<int>(std::round(
      (new_origin_x - map_.info.origin.position.x) / map_.info.resolution));
  const int shift_y = static_cast<int>(std::round(
      (new_origin_y - map_.info.origin.position.y) / map_.info.resolution));

  const int w = static_cast<int>(map_.info.width);
  const int h = static_cast<int>(map_.info.height);
  const size_t total = static_cast<size_t>(w) * h;
  std::vector<int8_t> new_data(total, -1);
  std::vector<rclcpp::Time> new_stamp(total, zeroTime());
  std::vector<uint8_t> new_state(total, kCellStatic);
  std::vector<rclcpp::Time> new_dyn_since(total, zeroTime());
  std::vector<uint8_t> new_seen_free(total, 0);
  for (int y = 0; y < h; ++y) {
    const int old_y = y + shift_y;
    if (old_y < 0 || old_y >= h) continue;
    for (int x = 0; x < w; ++x) {
      const int old_x = x + shift_x;
      if (old_x < 0 || old_x >= w) continue;
      const int old_idx = old_y * w + old_x;
      const int new_idx = y * w + x;
      new_data[new_idx] = map_.data[old_idx];
      new_stamp[new_idx] = cell_stamp_[old_idx];
      new_state[new_idx] = cell_state_[old_idx];
      new_dyn_since[new_idx] = cell_dyn_since_[old_idx];
      new_seen_free[new_idx] = cell_seen_free_[old_idx];
    }
  }
  map_.data.swap(new_data);
  cell_stamp_.swap(new_stamp);
  cell_state_.swap(new_state);
  cell_dyn_since_.swap(new_dyn_since);
  cell_seen_free_.swap(new_seen_free);
  map_.info.origin.position.x =
      map_.info.origin.position.x + shift_x * map_.info.resolution;
  map_.info.origin.position.y =
      map_.info.origin.position.y + shift_y * map_.info.resolution;
}

void SlidingMapNode::updateMap(const sensor_msgs::msg::LaserScan& scan,
                            const tf2::Transform& tf_map_laser,
                            const rclcpp::Time& stamp) {
  tf2::Vector3 origin = tf_map_laser.getOrigin();
  int origin_mx, origin_my;
  if (!worldToMap(origin.x(), origin.y(), origin_mx, origin_my)) {
    return;
  }

  tf2::Transform tf_map_base;
  bool have_tf_map_base = false;
  if (use_footprint_polygon_) {
    try {
      geometry_msgs::msg::TransformStamped tmb = tf_buffer_->lookupTransform(
          map_frame_, footprint_frame_, stamp, rclcpp::Duration::from_seconds(0.0));
      tf2::fromMsg(tmb.transform, tf_map_base);
      have_tf_map_base = true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "[sliding_map] footprint polygon needs TF %s->%s: %s "
          "(falling back to radial footprint filter)",
          map_frame_.c_str(), footprint_frame_.c_str(), ex.what());
    }
  }

  bool have_dyn_input = false;
  if (dynamic_memory_enable_ && latest_dynamic_scan_) {
    const rclcpp::Time dyn_stamp(latest_dynamic_scan_->header.stamp);
    if (std::fabs((stamp - dyn_stamp).seconds()) <= 0.05 &&
        latest_dynamic_scan_->intensities.size() == scan.ranges.size()) {
      have_dyn_input = true;
    }
  }
  std::vector<std::pair<double, double>> frame_hits;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float raw_range = scan.ranges[i];
    if (std::isnan(raw_range)) {
      continue;
    }
    double range = std::isfinite(raw_range) ? static_cast<double>(raw_range)
                                            : scan.range_max;
    bool has_hit = std::isfinite(raw_range) && raw_range >= scan.range_min &&
                   raw_range < scan.range_max;
    double angle =
        scan.angle_min + static_cast<double>(i) * scan.angle_increment;

    tf2::Vector3 direction(std::cos(angle), std::sin(angle), 0.0);
    double traveled = std::min(static_cast<double>(scan.range_max), range);

    std::vector<std::pair<int, int>> ray_cells;
    ray_cells.emplace_back(origin_mx, origin_my);

    double free_limit = has_hit ? std::max(range - 1e-3, 0.0) : traveled;
    for (double dist = ray_step_; dist <= free_limit; dist += ray_step_) {
      tf2::Vector3 point = tf_map_laser * (direction * dist);
      int mx, my;
      if (!worldToMap(point.x(), point.y(), mx, my)) {
        continue;
      }
      ray_cells.emplace_back(mx, my);
    }

    setFree(ray_cells, stamp);

    // Three-stage hit filtering: range bound, robot footprint self-filter, and follow-target mask.
    if (has_hit) {
      ++cnt_hits_;
      if (raw_range > max_fake_range_) {
        ++cnt_drop_far_;
      } else {
        tf2::Vector3 hit_point =
            tf_map_laser * (direction * static_cast<double>(raw_range));
        bool in_footprint;
        if (use_footprint_polygon_ && have_tf_map_base) {
          const tf2::Vector3 hb = tf_map_base.inverse() * hit_point;
          in_footprint =
              (std::fabs(hb.x()) <=
               footprint_length_m_ * 0.5 + footprint_margin_m_) &&
              (std::fabs(hb.y()) <=
               footprint_width_m_ * 0.5 + footprint_margin_m_);
        } else {
          in_footprint = (raw_range < footprint_radius_m_);
        }
        if (in_footprint) {
          ++cnt_drop_footprint_;
        } else if (follow_target_enable_ &&
                   insideActiveMask(hit_point.x(), hit_point.y())) {
          ++cnt_drop_mask_;
        } else {
          int mx, my;
          if (worldToMap(hit_point.x(), hit_point.y(), mx, my)) {
            const int cidx = index(mx, my);
            bool is_dyn = false;
            if (dynamic_memory_enable_) {
              if (have_dyn_input) {
                is_dyn = (latest_dynamic_scan_->intensities[i] >= 0.5f);
                // Do not classify a newly observed cell (transitioning directly from UNKNOWN) as vacated dynamic.
                if (dynamic_require_vacated_ && cidx >= 0 && !cell_seen_free_[cidx] &&
                    map_.data[cidx] == -1) {
                  is_dyn = false;
                }
              } else if (cidx >= 0 && cell_seen_free_[cidx] && map_.data[cidx] != 100) {
                is_dyn = true;
              }
              if (is_dyn) {
                frame_hits.emplace_back(hit_point.x(), hit_point.y());
              }
            }
            setOccupied(mx, my, stamp, is_dyn);
          }
        }
      }
    }
  }

  if (dynamic_memory_enable_ || footprint_clear_enable_) {
    const double res = map_.info.resolution;
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;
    const int w = static_cast<int>(map_.info.width);
    for (size_t idx = 0; idx < map_.data.size(); ++idx) {
      if (map_.data[idx] != 100) continue;
      const int mx = static_cast<int>(idx % w);
      const int my = static_cast<int>(idx / w);
      const double wx = ox + (mx + 0.5) * res;
      const double wy = oy + (my + 0.5) * res;
      bool in_zero_margin_fp = false;
      if (have_tf_map_base) {
        const tf2::Vector3 hb = tf_map_base.inverse() * tf2::Vector3(wx, wy, 0.0);
        in_zero_margin_fp = (std::fabs(hb.x()) <= footprint_length_m_ * 0.5) &&
                            (std::fabs(hb.y()) <= footprint_width_m_ * 0.5);
      }
      if (footprint_clear_enable_ && in_zero_margin_fp) {
        map_.data[idx] = 0;
        cell_stamp_[idx] = stamp;
        cell_state_[idx] = kCellStatic;
        cell_dyn_since_[idx] = zeroTime();
        cell_seen_free_[idx] = 1;
        continue;
      }
      if (dynamic_memory_enable_ && cell_state_[idx] == kCellDynamic &&
          cell_stamp_[idx] != stamp) {
        bool displaced_nearby = in_zero_margin_fp;
        for (const auto& hp : frame_hits) {
          const double d = std::hypot(wx - hp.first, wy - hp.second);
          if (d >= 0.12 && d <= 0.85) {
            displaced_nearby = true;
            break;
          }
        }
        if (displaced_nearby) {
          cell_state_[idx] = kCellVacatedDynamic;
          cell_dyn_since_[idx] = stamp;
        }
      }
    }
  }

  if (trace_period_sec_ > 0.0 &&
      (timeIsZero(last_trace_print_) ||
       (stamp - last_trace_print_).seconds() >= trace_period_sec_)) {
    last_trace_print_ = stamp;
    long occ_static = 0;
    long occ_dynamic = 0;
    for (size_t i = 0; i < map_.data.size(); ++i) {
      if (map_.data[i] != 100) continue;
      if (cell_state_[i] == kCellStatic) {
        ++occ_static;
      } else {
        ++occ_dynamic;
      }
    }
    RCLCPP_INFO(this->get_logger(),
             "[sliding_map] filter trace(%.0fs): hits=%ld drop_far(>%.2fm)=%ld "
             "drop_footprint=%ld drop_mask=%ld",
             trace_period_sec_, cnt_hits_, max_fake_range_, cnt_drop_far_,
             cnt_drop_footprint_, cnt_drop_mask_);
    RCLCPP_INFO(this->get_logger(),
             "[sliding_map] memory trace: occ_static=%ld occ_dynamic=%ld "
             "decay_static=%ld decay_dynamic=%ld dropped_early=%ld",
             occ_static, occ_dynamic, cnt_decay_static_,
             cnt_decay_dynamic_, cnt_dropped_early_);
    cnt_hits_ = 0;
    cnt_drop_far_ = 0;
    cnt_drop_footprint_ = 0;
    cnt_drop_mask_ = 0;
    cnt_decay_static_ = 0;
    cnt_decay_dynamic_ = 0;
    cnt_dropped_early_ = 0;
  }
}

void SlidingMapNode::publishFakeScan(const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(map_frame_, base_frame_, stamp,
                                         rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF lookup for base failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_map_base;
  tf2::fromMsg(tf_msg.transform, tf_map_base);
  tf2::Vector3 origin = tf_map_base.getOrigin();

  // Semantic of unobserved beams: report +inf (LiDAR "no return"). Before
  // this fix, unknown cells (-1) and grid-exits caused the beam to report
  // exactly max_fake_range_. NeuPAN's scan_callback filter
  // `distance <= scan_range[1]` (=4.5) accepted those, so 305/360 beams
  // became a fake 4.5m obstacle ring around the robot — destroying the
  // whole point of the memory grid. +inf is rejected by scan_callback's
  // upper-bound filter, so unobserved directions correctly contribute no
  // obstacle points.
  const float kNoReturn = std::numeric_limits<float>::infinity();

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = base_frame_;
  scan.angle_min = -kPi;
  scan.angle_max = kPi;
  scan.angle_increment =
      (scan.angle_max - scan.angle_min) / static_cast<double>(output_points_);
  scan.range_min = range_min_;
  scan.range_max = max_fake_range_;
  scan.ranges.assign(output_points_, kNoReturn);

  int origin_mx, origin_my;
  if (!worldToMap(origin.x(), origin.y(), origin_mx, origin_my)) {
    fake_scan_pub_->publish(scan);
    return;
  }

  tf2::Matrix3x3 rotation(tf_map_base.getRotation());

  for (int i = 0; i < output_points_; ++i) {
    double angle = scan.angle_min + i * scan.angle_increment;
    tf2::Vector3 dir_base(std::cos(angle), std::sin(angle), 0.0);
    tf2::Vector3 dir_map = rotation * dir_base;

    // Default: no information along this ray (no occupied cell, no grid).
    float measured = kNoReturn;
    for (double dist = range_min_; dist <= max_fake_range_; dist += ray_step_) {
      tf2::Vector3 point = origin + dir_map * dist;
      int mx, my;
      if (!worldToMap(point.x(), point.y(), mx, my)) {
        // Ray exited the grid without seeing an obstacle: no info → leave
        // `measured` at +inf so the beam is filtered out downstream.
        break;
      }

      int idx = index(mx, my);
      int value = map_.data[idx];
      if (value == 100) {
        // First occupied cell along the ray: that's the obstacle distance.
        measured = static_cast<float>(dist);
        break;
      }
      if (value == -1) {
        // First unknown cell along the ray: stop tracing (we have no
        // information past this point). DO NOT pretend it is free at
        // max_range — that creates a phantom obstacle ring around the
        // robot.
        break;
      }
      // value == 0 (free): keep walking the ray.
    }

    scan.ranges[i] = measured;
  }

  fake_scan_pub_->publish(scan);
}

bool SlidingMapNode::worldToMap(double wx, double wy, int& mx, int& my) const {
  double origin_x = map_.info.origin.position.x;
  double origin_y = map_.info.origin.position.y;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<int>((wx - origin_x) / map_.info.resolution);
  my = static_cast<int>((wy - origin_y) / map_.info.resolution);

  if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) ||
      my >= static_cast<int>(map_.info.height)) {
    return false;
  }
  return true;
}

void SlidingMapNode::setFree(const std::vector<std::pair<int, int>>& cells,
                          const rclcpp::Time& stamp) {
  for (const auto& cell : cells) {
    int idx = index(cell.first, cell.second);
    if (idx >= 0) {
      map_.data[idx] = 0;
      cell_stamp_[idx] = stamp;
      cell_state_[idx] = kCellStatic;
      cell_dyn_since_[idx] = zeroTime();
      cell_seen_free_[idx] = 1;
    }
  }
}

void SlidingMapNode::setOccupied(int mx, int my, const rclcpp::Time& stamp,
                              bool is_dynamic) {
  int idx = index(mx, my);
  if (idx >= 0) {
    map_.data[idx] = 100;
    cell_stamp_[idx] = stamp;
    if (dynamic_memory_enable_ && is_dynamic) {
      cell_state_[idx] = kCellDynamic;
      cell_dyn_since_[idx] = stamp;
    } else {
      cell_state_[idx] = kCellStatic;
      cell_dyn_since_[idx] = zeroTime();
    }
  }
}

// Follow-target obstacle masking

void SlidingMapNode::followTargetCb(const nav_interfaces::msg::FollowTarget::SharedPtr msg) {
  if (!follow_target_enable_) return;
  if (!msg->valid || msg->radius <= 0.0) {
    follow_person_recv_ = -1.0;
    return;
  }
  follow_person_x_ = msg->x;
  follow_person_y_ = msg->y;
  follow_person_radius_ = msg->radius;
  follow_person_recv_ = this->now().seconds();
}

void SlidingMapNode::dynamicGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!follow_target_enable_) return;
  follow_goal_x_ = msg->pose.position.x;
  follow_goal_y_ = msg->pose.position.y;
  follow_goal_recv_ = this->now().seconds();
}

bool SlidingMapNode::maskActive(const double recv_wall_sec) const {
  if (recv_wall_sec < 0.0) return false;
  return (this->now().seconds() - recv_wall_sec) <= follow_stale_sec_;
}

bool SlidingMapNode::insideActiveMask(double wx, double wy) const {
  if (follow_person_recv_ >= 0.0 && maskActive(follow_person_recv_)) {
    const double dx = wx - follow_person_x_;
    const double dy = wy - follow_person_y_;
    if (dx * dx + dy * dy <= follow_person_radius_ * follow_person_radius_) {
      return true;
    }
  }
  if (follow_goal_recv_ >= 0.0 && maskActive(follow_goal_recv_)) {
    const double dx = wx - follow_goal_x_;
    const double dy = wy - follow_goal_y_;
    if (dx * dx + dy * dy <= follow_goal_radius_ * follow_goal_radius_) {
      return true;
    }
  }
  return false;
}

void SlidingMapNode::clearMaskCircle(double cx, double cy, double radius,
                                  const rclcpp::Time& stamp) {
  const double res = map_.info.resolution;
  const int w = static_cast<int>(map_.info.width);
  const int h = static_cast<int>(map_.info.height);
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const int r_cells = std::max(1, static_cast<int>(std::ceil(radius / res)));
  const double r2 = radius * radius;

  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const double wx = cx + dx * res;
      const double wy = cy + dy * res;
      if (wx < origin_x || wy < origin_y) continue;
      const int mx = static_cast<int>((wx - origin_x) / res);
      const int my = static_cast<int>((wy - origin_y) / res);
      if (mx < 0 || my < 0 || mx >= w || my >= h) continue;
      if (dx * dx * res * res + dy * dy * res * res > r2) continue;
      const int idx = my * w + mx;
      if (map_.data[idx] != 0) {
        map_.data[idx] = 0;
      }
      cell_stamp_[idx] = stamp;
    }
  }
}

void SlidingMapNode::decayStaleCells(const rclcpp::Time& now) {
  if (cell_decay_sec_ <= 0.0) return;
  if (timeIsZero(last_decay_check_)) {
    last_decay_check_ = now;
    return;
  }
  const double elapsed_check = (now - last_decay_check_).seconds();
  if (elapsed_check < 0.0) {
    last_decay_check_ = now;
    return;
  }
  if (elapsed_check < decay_check_period_sec_) return;
  last_decay_check_ = now;

  // Only occupied cells decay. Free cells stay permanent so a once-observed
  // corridor remains usable for fake-scan raycasts even if the dog never
  // looks down it again. Decaying a cell back to unknown causes fake-scan
  // rays through it to terminate at range_max ("unknown blocks the ray").
  for (size_t i = 0; i < map_.data.size(); ++i) {
    if (map_.data[i] != 100) continue;
    if (timeIsZero(cell_stamp_[i])) continue;
    double timeout_sec = cell_decay_sec_;
    if (dynamic_memory_enable_) {
      if (cell_state_[i] == kCellVacatedDynamic) {
        timeout_sec = (dynamic_min_hold_sec_ > 0.0) ? dynamic_min_hold_sec_
                                                    : dynamic_decay_sec_;
      } else if (cell_state_[i] == kCellDynamic && !dynamic_require_vacated_) {
        timeout_sec = dynamic_decay_sec_;
      }
    }
    const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
    if ((now - cell_stamp_[i]) > timeout) {
      if (cell_state_[i] == kCellStatic) {
        ++cnt_decay_static_;
      } else {
        ++cnt_decay_dynamic_;
        if (timeout_sec < cell_decay_sec_) {
          ++cnt_dropped_early_;
        }
      }
      map_.data[i] = -1;
      cell_stamp_[i] = zeroTime();
      cell_state_[i] = kCellStatic;
      cell_dyn_since_[i] = zeroTime();
    }
  }
}

int SlidingMapNode::index(int mx, int my) const {
  if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) ||
      my >= static_cast<int>(map_.info.height)) {
    return -1;
  }
  return my * map_.info.width + mx;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlidingMapNode>());
  rclcpp::shutdown();
  return 0;
}
