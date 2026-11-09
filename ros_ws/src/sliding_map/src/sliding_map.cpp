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

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

SlidingMapNode::SlidingMapNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
  map_frame_ = pnh_.param<std::string>("map_frame", "odom");
  base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
  scan_topic_ = pnh_.param<std::string>("input_scan_topic", "/scan_input");
  fake_scan_topic_ = pnh_.param<std::string>("fake_scan_topic", "/scan_360");
  map_topic_ = pnh_.param<std::string>("map_topic", "/sliding_map");
  max_fake_range_ = pnh_.param("max_fake_range", 4.5);
  range_min_ = pnh_.param("range_min", 0.05);
  output_points_ = std::max(1, pnh_.param("output_points", 360));
  double resolution = pnh_.param("resolution", 0.05);
  int width = pnh_.param("width", 400);
  int height = pnh_.param("height", 400);
  double origin_x = pnh_.param("origin_x", -width * resolution / 2.0);
  double origin_y = pnh_.param("origin_y", -height * resolution / 2.0);

  map_.header.frame_id = map_frame_;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);

  ray_step_ = std::max(0.01, resolution * 0.5);

  // Time-decay knobs. cell_decay_sec_ <= 0 disables the decay loop entirely
  // (i.e. pure persistence, original behavior). decay_check_period_sec_ keeps
  // the per-frame cost bounded by amortizing the full-grid scan.
  cell_decay_sec_ = pnh_.param("cell_decay_sec", 8.0);
  decay_check_period_sec_ = pnh_.param("decay_check_period_sec", 0.5);
  footprint_radius_m_ = pnh_.param("footprint_radius_m", 0.35);
  // Footprint polygon self-filter and diagnostic trace.
  // use_footprint_polygon=false falls back to radial filtering.
  use_footprint_polygon_ = pnh_.param("use_footprint_polygon", false);
  footprint_frame_ = pnh_.param<std::string>("footprint_frame", "base_link");
  footprint_length_m_ = pnh_.param("footprint_length_m", 0.96);
  footprint_width_m_ = pnh_.param("footprint_width_m", 0.55);
  footprint_margin_m_ = pnh_.param("footprint_margin_m", 0.10);
  trace_period_sec_ = pnh_.param("trace_period_sec", 5.0);
  recenter_threshold_m_ = pnh_.param("recenter_threshold_m",
                                     std::min(width, height) * resolution / 4.0);
  cell_stamp_.assign(static_cast<size_t>(width) * static_cast<size_t>(height),
                     ros::Time(0));
  last_decay_check_ = ros::Time(0);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
  fake_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 1);
  scan_sub_ = nh_.subscribe(scan_topic_, 1, &SlidingMapNode::scanCallback, this);

  // Follow-target masking (disabled by default): masks out the followed person
  // and dynamic goal circles from the dynamic obstacle layer so the goal cell
  // does not fall inside the person's recent footprint trail.
  follow_target_enable_ = pnh_.param("follow_target_enable", false);
  follow_target_topic_ =
      pnh_.param<std::string>("follow_target_topic", follow_target_topic_);
  dynamic_goal_topic_ =
      pnh_.param<std::string>("dynamic_goal_topic", dynamic_goal_topic_);
  follow_stale_sec_ = pnh_.param("follow_stale_sec", follow_stale_sec_);
  follow_person_radius_ = pnh_.param("follow_person_radius", follow_person_radius_);
  follow_goal_radius_ = pnh_.param("follow_goal_radius", follow_goal_radius_);
  if (follow_target_enable_) {
    follow_sub_ = nh_.subscribe(follow_target_topic_, 1,
                                &SlidingMapNode::followTargetCb, this);
    dynamic_goal_sub_ = nh_.subscribe(dynamic_goal_topic_, 1,
                                      &SlidingMapNode::dynamicGoalCb, this);
    ROS_INFO("[sliding_map] follow-target masking ENABLED: person_r=%.2f goal_r=%.2f "
             "stale=%.1fs", follow_person_radius_, follow_goal_radius_,
             follow_stale_sec_);
  } else {
    ROS_INFO("[sliding_map] follow-target masking disabled (default)");
  }
}

void SlidingMapNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  geometry_msgs::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                        msg->header.stamp, ros::Duration(0.1));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_map_laser;
  tf2::fromMsg(tf_msg.transform, tf_map_laser);

  const tf2::Vector3 laser_origin = tf_map_laser.getOrigin();
  maybeRecenter(laser_origin.x(), laser_origin.y());

  updateMap(*msg, tf_map_laser, msg->header.stamp);

  // Follow-target masking: clear any existing occupied cells inside the target circles.
  if (follow_target_enable_) {
    double cx, cy;
    if (follow_person_recv_ >= 0.0 && maskActive(follow_person_recv_)) {
      clearMaskCircle(follow_person_x_, follow_person_y_,
                      follow_person_radius_, msg->header.stamp);
    }
    if (follow_goal_recv_ >= 0.0 && maskActive(follow_goal_recv_)) {
      clearMaskCircle(follow_goal_x_, follow_goal_y_,
                      follow_goal_radius_, msg->header.stamp);
    }
  }

  // Expire occupied cells that haven't been refreshed for too long. Cells
  // currently inside the LiDAR FOV got their stamp bumped by updateMap, so
  // they survive; cells behind the robot do not, so they decay back to
  // unknown after cell_decay_sec_.
  decayStaleCells(msg->header.stamp);

  map_.header.stamp = msg->header.stamp;
  map_.info.map_load_time = msg->header.stamp;
  map_pub_.publish(map_);

  publishFakeScan(msg->header.stamp);
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
  std::vector<int8_t> new_data(static_cast<size_t>(w) * h, -1);
  std::vector<ros::Time> new_stamp(static_cast<size_t>(w) * h, ros::Time(0));
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
    }
  }
  map_.data.swap(new_data);
  cell_stamp_.swap(new_stamp);
  map_.info.origin.position.x =
      map_.info.origin.position.x + shift_x * map_.info.resolution;
  map_.info.origin.position.y =
      map_.info.origin.position.y + shift_y * map_.info.resolution;
}

void SlidingMapNode::updateMap(const sensor_msgs::LaserScan &scan,
                            const tf2::Transform &tf_map_laser,
                            const ros::Time &stamp) {
  tf2::Vector3 origin = tf_map_laser.getOrigin();
  int origin_mx, origin_my;
  if (!worldToMap(origin.x(), origin.y(), origin_mx, origin_my)) {
    return;
  }

  // Footprint polygon requires map -> footprint_frame (robot center base_link);
  // falls back to radial filtering if lookup fails.
  tf2::Transform tf_map_base;
  bool have_tf_map_base = false;
  if (use_footprint_polygon_) {
    try {
      geometry_msgs::TransformStamped tmb = tf_buffer_.lookupTransform(
          map_frame_, footprint_frame_, stamp, ros::Duration(0.0));
      tf2::fromMsg(tmb.transform, tf_map_base);
      have_tf_map_base = true;
    } catch (const tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5.0,
          "[sliding_map] footprint polygon needs TF %s->%s: %s "
          "(falling back to radial footprint filter)",
          map_frame_.c_str(), footprint_frame_.c_str(), ex.what());
    }
  }

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float raw_range = scan.ranges[i];
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

    // Hit filtering:
    // 1) Enforce raw_range <= max_fake_range_ so map and controller ranges match.
    // 2) Apply rectangular or radial self-filter so body/leg hits do not poison start cells.
    // 3) Apply follow-target mask circles when enabled.
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
            setOccupied(mx, my, stamp);
          }
        }
      }
    }
  }

  if (trace_period_sec_ > 0.0 &&
      (last_trace_print_.isZero() ||
       (stamp - last_trace_print_).toSec() >= trace_period_sec_)) {
    last_trace_print_ = stamp;
    ROS_INFO("[sliding_map] filter trace(%.0fs): hits=%ld drop_far(>%.2fm)=%ld "
             "drop_footprint=%ld drop_mask=%ld",
             trace_period_sec_, cnt_hits_, max_fake_range_, cnt_drop_far_,
             cnt_drop_footprint_, cnt_drop_mask_);
    cnt_hits_ = 0;
    cnt_drop_far_ = 0;
    cnt_drop_footprint_ = 0;
    cnt_drop_mask_ = 0;
  }
}

void SlidingMapNode::publishFakeScan(const ros::Time &stamp) {
  geometry_msgs::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_.lookupTransform(map_frame_, base_frame_, stamp,
                                        ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "TF lookup for base failed: %s", ex.what());
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

  sensor_msgs::LaserScan scan;
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
    fake_scan_pub_.publish(scan);
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

  fake_scan_pub_.publish(scan);
}

bool SlidingMapNode::worldToMap(double wx, double wy, int &mx, int &my) const {
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

void SlidingMapNode::setFree(const std::vector<std::pair<int, int>> &cells,
                          const ros::Time &stamp) {
  for (const auto &cell : cells) {
    int idx = index(cell.first, cell.second);
    if (idx >= 0) {
      map_.data[idx] = 0;
      cell_stamp_[idx] = stamp;
    }
  }
}

void SlidingMapNode::setOccupied(int mx, int my, const ros::Time &stamp) {
  int idx = index(mx, my);
  if (idx >= 0) {
    map_.data[idx] = 100;
    cell_stamp_[idx] = stamp;
  }
}

// Follow-target masking callbacks and helpers

void SlidingMapNode::followTargetCb(const nav_interfaces::FollowTarget::ConstPtr &msg) {
  if (!follow_target_enable_) return;
  if (!msg->valid || msg->radius <= 0.0) {
    follow_person_recv_ = -1.0;
    return;
  }
  follow_person_x_ = msg->x;
  follow_person_y_ = msg->y;
  follow_person_radius_ = msg->radius;
  follow_person_recv_ = ros::Time::now().toSec();
}

void SlidingMapNode::dynamicGoalCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!follow_target_enable_) return;
  follow_goal_x_ = msg->pose.position.x;
  follow_goal_y_ = msg->pose.position.y;
  follow_goal_recv_ = ros::Time::now().toSec();
}

bool SlidingMapNode::maskActive(const double recv_wall_sec) const {
  if (recv_wall_sec < 0.0) return false;
  return (ros::Time::now().toSec() - recv_wall_sec) <= follow_stale_sec_;
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
                                  const ros::Time &stamp) {
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

void SlidingMapNode::decayStaleCells(const ros::Time &now) {
  if (cell_decay_sec_ <= 0.0) return;
  if (last_decay_check_.isZero()) {
    last_decay_check_ = now;
    return;
  }
  if ((now - last_decay_check_).toSec() < decay_check_period_sec_) return;
  last_decay_check_ = now;

  const ros::Duration timeout(cell_decay_sec_);
  // Only occupied cells decay. Free cells stay permanent so a once-observed
  // corridor remains usable for fake-scan raycasts even if the dog never
  // looks down it again. Decaying a cell back to unknown causes fake-scan
  // rays through it to terminate at range_max ("unknown blocks the ray").
  for (size_t i = 0; i < map_.data.size(); ++i) {
    if (map_.data[i] != 100) continue;
    if (cell_stamp_[i].isZero()) continue;
    if ((now - cell_stamp_[i]) > timeout) {
      map_.data[i] = -1;
      cell_stamp_[i] = ros::Time(0);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "sliding_map_node");
  SlidingMapNode node;
  ros::spin();
  return 0;
}
