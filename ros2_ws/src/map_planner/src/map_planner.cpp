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

#include "map_planner.h"
#include "path_safety.h"
#include "ec_trace.h"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
struct Node {
  int index;
  double g;
  double f;
  bool operator>(const Node& other) const { return f > other.f; }
};

constexpr double SQRT2 = 1.41421356237;
}  // namespace

MapPlanner::MapPlanner(const rclcpp::Node::SharedPtr& node) : node_(node) {
  inflation_radius_ = node_->declare_parameter("inflation_radius", inflation_radius_);
  obstacle_threshold_ = node_->declare_parameter("obstacle_threshold", obstacle_threshold_);
  publish_path_ = node_->declare_parameter("publish_path", publish_path_);
  plan_service_name_ = node_->declare_parameter("service_name", plan_service_name_);
  sliding_map_max_age_sec_ =
      node_->declare_parameter("sliding_map_max_age_sec", sliding_map_max_age_sec_);
  require_fresh_sliding_map_ =
      node_->declare_parameter("require_fresh_sliding_map", require_fresh_sliding_map_);
  path_max_start_offset_m_ =
      node_->declare_parameter("path_max_start_offset_m", path_max_start_offset_m_);
  inflated_map_publish_period_sec_ = node_->declare_parameter(
      "inflated_map_publish_period_sec", inflated_map_publish_period_sec_);
  inflated_map_window_m_ =
      node_->declare_parameter("inflated_map_window_m", inflated_map_window_m_);
  smoothing_iterations_ =
      node_->declare_parameter("smoothing_iterations", smoothing_iterations_);
  smoothing_max_deviation_m_ =
      node_->declare_parameter("smoothing_max_deviation_m", smoothing_max_deviation_m_);
  obstacle_cost_weight_ =
      node_->declare_parameter("obstacle_cost_weight", obstacle_cost_weight_);
  obstacle_cost_safe_distance_m_ = node_->declare_parameter(
      "obstacle_cost_safe_distance", obstacle_cost_safe_distance_m_);
  line_of_sight_safe_distance_m_ = node_->declare_parameter(
      "line_of_sight_safe_distance", line_of_sight_safe_distance_m_);
  start_heading_penalty_weight_ = node_->declare_parameter(
      "start_heading_penalty_weight", start_heading_penalty_weight_);
  start_heading_penalty_radius_m_ = node_->declare_parameter(
      "start_heading_penalty_radius_m", start_heading_penalty_radius_m_);
  turn_penalty_weight_ =
      node_->declare_parameter("turn_penalty_weight", turn_penalty_weight_);
  use_global_memory_ = node_->declare_parameter("use_global_memory", use_global_memory_);
  global_memory_res_ = node_->declare_parameter("global_memory_res", global_memory_res_);
  global_memory_inflation_radius_ = node_->declare_parameter(
      "global_memory_inflation_radius", global_memory_inflation_radius_);
  global_memory_exempt_radius_m_ = node_->declare_parameter(
      "global_memory_exempt_radius_m", global_memory_exempt_radius_m_);
  path_initial_heading_tol_deg_ = node_->declare_parameter(
      "path_initial_heading_tol_deg", path_initial_heading_tol_deg_);
  path_protect_initial_m_ =
      node_->declare_parameter("path_protect_initial_m", path_protect_initial_m_);
  path_validate_enable_ =
      node_->declare_parameter("path_validate_enable", path_validate_enable_);
  path_min_clear_m_ = node_->declare_parameter("path_min_clear_m", path_min_clear_m_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, node_, true, tf2_ros::DynamicListenerQoS().best_effort());

  rclcpp::QoS latched(1);
  latched.transient_local();
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("initial_path", latched);
  inflated_map_pub_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_map", latched);
  plan_result_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("/map_planner/result", latched);
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched, std::bind(&MapPlanner::mapCallback, this, std::placeholders::_1));
  sliding_map_input_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = sliding_map_input_group_;
  sliding_map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/sliding_map", latched,
      std::bind(&MapPlanner::slidingMapCallback, this, std::placeholders::_1), input_options);
  map_visualization_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(std::max(0.05, inflated_map_publish_period_sec_)),
      std::bind(&MapPlanner::refreshMapVisualization, this));
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(1),
      std::bind(&MapPlanner::goalCallback, this, std::placeholders::_1));

  std::string service_name = plan_service_name_;
  if (service_name.empty()) service_name = "plan";
  if (service_name.front() != '/') service_name = "~/" + service_name;
  plan_service_ = node_->create_service<nav_interfaces::srv::PlanPath>(
      service_name,
      std::bind(&MapPlanner::planService, this, std::placeholders::_1,
                std::placeholders::_2));
  plan_sequence_service_ = node_->create_service<nav_interfaces::srv::PlanSequence>(
      "~/plan_sequence",
      std::bind(&MapPlanner::planSequenceService, this, std::placeholders::_1,
                std::placeholders::_2));
  validate_path_service_ = node_->create_service<nav_interfaces::srv::ValidatePath>(
      "~/validate_path",
      std::bind(&MapPlanner::validatePathService, this, std::placeholders::_1,
                std::placeholders::_2));
}

void MapPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
  map_ = *msg;
  if (map_.info.resolution <= 0.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Map resolution invalid.");
    map_ready_ = false;
    return;
  }
  inflation_cells_ = std::max(
      1, static_cast<int>(std::ceil(inflation_radius_ / map_.info.resolution)));
  inflateMap();
  map_ready_ = true;

  // Build the static distance transform cache exclusively from inflated_data_
  // before merging any sliding_map dynamic obstacles.
  plan_data_ = inflated_data_;
  static_distance_ready_ = false;
  computeDistanceTransform();
  distance_static_cache_ = distance_to_blocked_;
  static_distance_ready_ = true;
  static_distance_built_at_ = node_->now();
  RCLCPP_INFO(node_->get_logger(),
              "[map_planner] static distance transform cached (%zu cells, pure static)",
              distance_static_cache_.size());

  if (use_global_memory_) initGlobalMemory();

  buildPlanData();
  publishInflatedMap();
  RCLCPP_INFO_ONCE(node_->get_logger(), "Inflated map ready for planning.");
}

void MapPlanner::slidingMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
  std::atomic_store(&latest_sliding_map_, msg);
}

void MapPlanner::refreshMapVisualization() {
  const auto latest = std::atomic_load(&latest_sliding_map_);
  if (!map_ready_ || !latest || latest == visualized_sliding_map_) return;
  buildPlanData();
  visualized_sliding_map_ = latest;
  if (sliding_map_ready_) publishInflatedMap();
}

void MapPlanner::buildPlanData() {
  const auto snapshot = std::atomic_load(&latest_sliding_map_);
  if (snapshot) {
    sliding_map_ = *snapshot;
    sliding_map_ready_ = sliding_map_.info.resolution > 0.0 &&
        sliding_map_.info.width > 0 && sliding_map_.info.height > 0 &&
        sliding_map_.data.size() ==
            static_cast<std::size_t>(sliding_map_.info.width) * sliding_map_.info.height;
  } else {
    sliding_map_ready_ = false;
  }
  plan_data_ = inflated_data_;
  dynamic_touched_.clear();
  dynamic_map_error_.clear();

  auto inject_sliding_map = [&]() {
    if (!sliding_map_ready_) {
      dynamic_map_error_ = "DYNAMIC_MAP_UNAVAILABLE";
      return;
    }

    const rclcpp::Time now = node_->now();
    if (sliding_map_.header.stamp.sec != 0 || sliding_map_.header.stamp.nanosec != 0) {
      const double age = (now - rclcpp::Time(sliding_map_.header.stamp)).seconds();
      if (age < -0.1 || age > sliding_map_max_age_sec_) {
        dynamic_map_error_ = "DYNAMIC_MAP_STALE";
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "sliding_map stale (%.1fs old), dynamic planning unavailable.",
                             age);
        return;
      }
    } else {
      dynamic_map_error_ = "DYNAMIC_MAP_STALE";
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
          map_.header.frame_id,
          sliding_map_.header.frame_id,
          tf2::TimePointZero,
          std::chrono::milliseconds(50));
    } catch (const tf2::TransformException& ex) {
      dynamic_map_error_ = "DYNAMIC_MAP_TF_UNAVAILABLE";
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Cannot merge sliding_map: TF %s -> %s failed (%s).",
                           sliding_map_.header.frame_id.c_str(),
                           map_.header.frame_id.c_str(), ex.what());
      return;
    }

    const double cx = tf_msg.transform.translation.x;
    const double cy = tf_msg.transform.translation.y;
    const double qx = tf_msg.transform.rotation.x;
    const double qy = tf_msg.transform.rotation.y;
    const double qz = tf_msg.transform.rotation.z;
    const double qw = tf_msg.transform.rotation.w;
    const double yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);

    const int fw = static_cast<int>(sliding_map_.info.width);
    const int fh = static_cast<int>(sliding_map_.info.height);
    const double fres = sliding_map_.info.resolution;
    const double fox = sliding_map_.info.origin.position.x;
    const double foy = sliding_map_.info.origin.position.y;
    const int mw = static_cast<int>(map_.info.width);
    const int mh = static_cast<int>(map_.info.height);

    for (int fy = 0; fy < fh; ++fy) {
      for (int fx = 0; fx < fw; ++fx) {
        const int fidx = fy * fw + fx;
        if (sliding_map_.data[fidx] != 100) continue;
        const double wx_odom = fox + (fx + 0.5) * fres;
        const double wy_odom = foy + (fy + 0.5) * fres;
        const double wx_map = cos_y * wx_odom - sin_y * wy_odom + cx;
        const double wy_map = sin_y * wx_odom + cos_y * wy_odom + cy;
        const int mx = static_cast<int>(
            std::floor((wx_map - map_.info.origin.position.x) / map_.info.resolution));
        const int my = static_cast<int>(
            std::floor((wy_map - map_.info.origin.position.y) / map_.info.resolution));
        if (mx < 0 || my < 0 || mx >= mw || my >= mh) continue;
        if (use_global_memory_ && gm_ready_) {
          // Stamp into persistent coarse memory only when the uninflated static cell is not an obstacle.
          const int sidx = my * mw + mx;
          if (map_.data[sidx] < obstacle_threshold_) {
            stampGlobalMemory(wx_map, wy_map);
          }
        }
        for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
          for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
            const int nx = mx + dx;
            const int ny = my + dy;
            if (nx < 0 || ny < 0 || nx >= mw || ny >= mh) continue;
            if (std::hypot(dx, dy) * map_.info.resolution > inflation_radius_) continue;
            const int cell = ny * mw + nx;
            if (plan_data_[cell] != 100) dynamic_touched_.push_back(cell);
            plan_data_[cell] = 100;
          }
        }
      }
    }

    // Allow fresh sliding_map FREE cells to clear UNKNOWN (-1) cells in plan_data_
    // without overriding static or dynamic occupied cells (100).
    for (int fy = 0; fy < fh; ++fy) {
      for (int fx = 0; fx < fw; ++fx) {
        const int fidx = fy * fw + fx;
        if (sliding_map_.data[fidx] != 0) continue;
        const double wx_odom = fox + (fx + 0.5) * fres;
        const double wy_odom = foy + (fy + 0.5) * fres;
        const double wx_map = cos_y * wx_odom - sin_y * wy_odom + cx;
        const double wy_map = sin_y * wx_odom + cos_y * wy_odom + cy;
        const int mx = static_cast<int>(
            std::floor((wx_map - map_.info.origin.position.x) / map_.info.resolution));
        const int my = static_cast<int>(
            std::floor((wy_map - map_.info.origin.position.y) / map_.info.resolution));
        if (mx < 0 || my < 0 || mx >= mw || my >= mh) continue;
        const int idx = my * mw + mx;
        if (plan_data_[idx] < 0) plan_data_[idx] = 0;
      }
    }
  };
  inject_sliding_map();

  if (use_global_memory_ && gm_ready_ && gm_cell_count_ > 0) {
    mergeGlobalMemory();
  }

  if (!static_distance_ready_) {
    computeDistanceTransform();
    return;
  }
  distance_to_blocked_ = distance_static_cache_;

  if (dynamic_touched_.empty()) {
    return;
  }
  computeLocalDistanceTransform(dynamic_touched_);
}

// Global coarse persistent memory

void MapPlanner::initGlobalMemory() {
  if (map_.info.resolution <= 0.0 || global_memory_res_ <= 0.0) return;
  gm_w_ = std::max(1, static_cast<int>(std::ceil(
      map_.info.width * map_.info.resolution / global_memory_res_)));
  gm_h_ = std::max(1, static_cast<int>(std::ceil(
      map_.info.height * map_.info.resolution / global_memory_res_)));
  gm_ox_ = map_.info.origin.position.x;
  gm_oy_ = map_.info.origin.position.y;
  global_memory_.assign(static_cast<size_t>(gm_w_) * gm_h_, 0);
  gm_cell_count_ = 0;
  gm_ready_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "[map_planner] R2 global memory ready: %dx%d @%.2fm (cells=%zu), "
              "inflation=%.2fm", gm_w_, gm_h_, global_memory_res_,
              global_memory_.size(), global_memory_inflation_radius_);
}

void MapPlanner::stampGlobalMemory(double wx_map, double wy_map) {
  const int gx = static_cast<int>((wx_map - gm_ox_) / global_memory_res_);
  const int gy = static_cast<int>((wy_map - gm_oy_) / global_memory_res_);
  if (gx < 0 || gy < 0 || gx >= gm_w_ || gy >= gm_h_) return;
  const size_t idx = static_cast<size_t>(gy) * gm_w_ + gx;
  if (global_memory_[idx] != 100) {
    global_memory_[idx] = 100;
    ++gm_cell_count_;
  }
}

void MapPlanner::mergeGlobalMemory() {
  const double exempt_r = global_memory_exempt_radius_m_ > 0.0
      ? global_memory_exempt_radius_m_
      : inflation_radius_ + 0.3;
  const double exempt_sq = exempt_r * exempt_r;
  const int icells = std::max(1, static_cast<int>(std::ceil(
      global_memory_inflation_radius_ / map_.info.resolution)));
  const int mw = static_cast<int>(map_.info.width);
  const int mh = static_cast<int>(map_.info.height);
  size_t injected = 0;
  for (int gy = 0; gy < gm_h_; ++gy) {
    for (int gx = 0; gx < gm_w_; ++gx) {
      if (global_memory_[static_cast<size_t>(gy) * gm_w_ + gx] != 100) continue;
      const double wx = gm_ox_ + (gx + 0.5) * global_memory_res_;
      const double wy = gm_oy_ + (gy + 0.5) * global_memory_res_;
      if (gm_exempt_valid_) {
        const double ds = (wx - gm_exempt_start_.x) * (wx - gm_exempt_start_.x)
                        + (wy - gm_exempt_start_.y) * (wy - gm_exempt_start_.y);
        const double dg = (wx - gm_exempt_goal_.x) * (wx - gm_exempt_goal_.x)
                        + (wy - gm_exempt_goal_.y) * (wy - gm_exempt_goal_.y);
        if (ds < exempt_sq || dg < exempt_sq) continue;
      }
      const int mx = static_cast<int>(std::floor(
          (wx - map_.info.origin.position.x) / map_.info.resolution));
      const int my = static_cast<int>(std::floor(
          (wy - map_.info.origin.position.y) / map_.info.resolution));
      if (mx < 0 || my < 0 || mx >= mw || my >= mh) continue;
      for (int dy = -icells; dy <= icells; ++dy) {
        for (int dx = -icells; dx <= icells; ++dx) {
          const int nx = mx + dx;
          const int ny = my + dy;
          if (nx < 0 || ny < 0 || nx >= mw || ny >= mh) continue;
          if (std::hypot(dx, dy) * map_.info.resolution >
              global_memory_inflation_radius_) continue;
          const int cell = ny * mw + nx;
          if (plan_data_[cell] != 100) {
            dynamic_touched_.push_back(cell);
            plan_data_[cell] = 100;
            ++injected;
          }
        }
      }
    }
  }
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                       "[map_planner] R2 global memory merged: %zu gm cells, %zu plan cells "
                       "injected (exempt r=%.2fm)", gm_cell_count_, injected, exempt_r);
}

void MapPlanner::publishInflatedMap() {
  if (!inflated_map_pub_) return;
  const std::vector<int8_t>& src =
      plan_data_.empty() ? inflated_data_ : plan_data_;

  // Crop /inflated_map around the robot when inflated_map_window_m_ > 0 to limit visualization bandwidth.
  if (inflated_map_window_m_ > 0.0 && map_.info.resolution > 0.0) {
    geometry_msgs::msg::TransformStamped tf;
    bool have_pose = true;
    try {
      tf = tf_buffer_->lookupTransform(map_.header.frame_id.empty()
                                          ? std::string("map")
                                          : map_.header.frame_id,
                                      "base_link", tf2::TimePointZero,
                                      std::chrono::milliseconds(50));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "inflated_map window: no robot pose (%s), "
                           "publishing full map", ex.what());
      have_pose = false;
    }
    if (have_pose) {
      const double res = map_.info.resolution;
      const int W = static_cast<int>(map_.info.width);
      const int H = static_cast<int>(map_.info.height);
      const int half = std::max(1, static_cast<int>(
                                       std::ceil(inflated_map_window_m_ / res)));
      const int cx = static_cast<int>(
          (tf.transform.translation.x - map_.info.origin.position.x) / res);
      const int cy = static_cast<int>(
          (tf.transform.translation.y - map_.info.origin.position.y) / res);
      const int x0 = std::max(0, cx - half);
      const int y0 = std::max(0, cy - half);
      const int x1 = std::min(W - 1, cx + half);
      const int y1 = std::min(H - 1, cy + half);
      if (x1 >= x0 && y1 >= y0) {
        nav_msgs::msg::OccupancyGrid win;
        win.header.frame_id = map_.header.frame_id;
        win.header.stamp = node_->now();
        win.info = map_.info;
        win.info.width = static_cast<unsigned int>(x1 - x0 + 1);
        win.info.height = static_cast<unsigned int>(y1 - y0 + 1);
        win.info.origin.position.x =
            map_.info.origin.position.x + x0 * res;
        win.info.origin.position.y =
            map_.info.origin.position.y + y0 * res;
        win.data.resize(static_cast<size_t>(win.info.width) * win.info.height);
        for (int y = y0; y <= y1; ++y) {
          const size_t src_row = static_cast<size_t>(y) * W;
          const size_t dst_row =
              static_cast<size_t>(y - y0) * win.info.width;
          std::copy(src.begin() + src_row + x0,
                    src.begin() + src_row + x1 + 1,
                    win.data.begin() + dst_row);
        }
        inflated_map_pub_->publish(win);
        return;
      }
    }
  }

  nav_msgs::msg::OccupancyGrid inflated = map_;
  inflated.header.stamp = node_->now();
  // F2: publish the MERGED plan_data_ so visualizers (foxglove) show what
  // A* actually sees, including dynamic obstacles.
  inflated.data = src;
  inflated_map_pub_->publish(inflated);
}

void MapPlanner::publishPlanResult(bool success) {
  if (!plan_result_pub_) return;
  std_msgs::msg::Bool msg;
  msg.data = success;
  plan_result_pub_->publish(msg);
}

void MapPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal) {
  if (!map_ready_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Map not ready for planning.");
    publishPlanResult(false);
    return;
  }
  geometry_msgs::msg::PoseStamped start_pose;
  if (!getRobotPose(start_pose)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Unable to get robot pose.");
    publishPlanResult(false);
    return;
  }
  geometry_msgs::msg::PoseStamped goal_in_map = *goal;
  if (goal->header.frame_id != map_.header.frame_id) {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped =
          tf_buffer_->lookupTransform(map_.header.frame_id,   // target frame
                                      goal->header.frame_id,  // source frame
                                      tf2::TimePointZero,     // latest available
                                      std::chrono::milliseconds(100));  // timeout
      tf2::doTransform(*goal, goal_in_map, tf_stamped);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Goal transform failed: %s", ex.what());
      publishPlanResult(false);
      return;
    }
  }
  nav_msgs::msg::Path path;
  std::string fail_reason;
  const bool success = plan(start_pose, goal_in_map, path, fail_reason);
  publishPlanResult(success);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to plan a path: %s", fail_reason.c_str());
    return;
  }
  path_pub_->publish(path);
}

bool MapPlanner::getRobotPose(geometry_msgs::msg::PoseStamped& pose) const {
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        map_.header.frame_id, "base_link", tf2::TimePointZero,
        std::chrono::milliseconds(200));
    const rclcpp::Time stamp(tf.header.stamp);
    const double age = (node_->now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > 1.0)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "Robot pose TF is not fresh (age %.3fs, limit 1.0s)", age);
      return false;
    }
    pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "TF lookup failed: %s", ex.what());
    return false;
  }
}

bool MapPlanner::plan(const geometry_msgs::msg::PoseStamped& start,
                      const geometry_msgs::msg::PoseStamped& goal,
                      nav_msgs::msg::Path& path, std::string& fail_reason) {
  last_path_quality_message_ = "OK";
  if (use_global_memory_) {
    gm_exempt_start_ = start.pose.position;
    gm_exempt_goal_ = goal.pose.position;
    gm_exempt_valid_ = true;
  }
  const auto t_plan_begin = std::chrono::steady_clock::now();
  buildPlanData();
  const auto t_built = std::chrono::steady_clock::now();
  if (require_fresh_sliding_map_ && !dynamic_map_error_.empty()) {
    fail_reason = dynamic_map_error_;
    return false;
  }

  int start_x, start_y, goal_x, goal_y;
  if (!worldToMap(start.pose.position, start_x, start_y) ||
      !worldToMap(goal.pose.position, goal_x, goal_y)) {
    RCLCPP_WARN(node_->get_logger(), "Start or goal outside the map.");
    fail_reason = "OUT_OF_MAP";
    return false;
  }
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const int start_index = toIndex(start_x, start_y);
  const int goal_index = toIndex(goal_x, goal_y);
  const bool start_occupied = !cellHasClearance(start_index, path_min_clear_m_);
  const bool goal_occupied = !cellHasClearance(goal_index, path_min_clear_m_);
  if (start_occupied || goal_occupied) {
    fail_reason = (start_occupied && goal_occupied) ? "START_AND_GOAL_OCCUPIED"
                  : start_occupied                  ? "START_OCCUPIED"
                                                     : "GOAL_OCCUPIED";
    RCLCPP_WARN(node_->get_logger(),
                "Start or goal is blocked by occupancy/clearance "
                "(%s, required clearance %.2fm).",
                fail_reason.c_str(), path_min_clear_m_);
    return false;
  }

  const auto& q = start.pose.orientation;
  const double start_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  auto heuristic = [&](int mx, int my) {
    const double dx = static_cast<double>(mx - goal_x);
    const double dy = static_cast<double>(my - goal_y);
    return std::hypot(dx, dy);
  };

  // Search within a start-to-goal bounding box first, falling back to a full-map search if no path is found.
  const double path_len_m =
      std::hypot(static_cast<double>(goal_x - start_x),
                 static_cast<double>(goal_y - start_y)) * map_.info.resolution;
  const double margin_m = std::min(25.0, std::max(3.0, path_len_m * 0.25));
  const int margin_cells = static_cast<int>(std::ceil(margin_m / map_.info.resolution));
  const int bx0 = std::max(0, std::min(start_x, goal_x) - margin_cells);
  const int by0 = std::max(0, std::min(start_y, goal_y) - margin_cells);
  const int bx1 = std::min(static_cast<int>(map_.info.width) - 1,
                           std::max(start_x, goal_x) + margin_cells);
  const int by1 = std::min(static_cast<int>(map_.info.height) - 1,
                           std::max(start_y, goal_y) + margin_cells);
  RCLCPP_INFO(node_->get_logger(),
              "[map_planner] A* search bbox: (%d,%d)-(%d,%d) = %dx%d cells, "
              "path=%.1fm margin=%.1fm (%d cells), full map = %ux%u",
              bx0, by0, bx1, by1, bx1 - bx0 + 1, by1 - by0 + 1,
              path_len_m, margin_m, margin_cells,
              map_.info.width, map_.info.height);

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double costs[8] = {1.0, 1.0, 1.0, 1.0, SQRT2, SQRT2, SQRT2, SQRT2};

  std::vector<int> index_path;
  auto run_a_star = [&](int search_x0, int search_y0, int search_x1,
                        int search_y1) {
    const int search_width = search_x1 - search_x0 + 1;
    const int search_height = search_y1 - search_y0 + 1;
    const std::size_t search_size =
        static_cast<std::size_t>(search_width) * search_height;
    std::vector<double> g_score(
        search_size, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(search_size, -1);
    auto local_index = [&](int mx, int my) {
      return static_cast<std::size_t>(my - search_y0) * search_width +
             static_cast<std::size_t>(mx - search_x0);
    };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    g_score[local_index(start_x, start_y)] = 0.0;
    open_set.push({start_index, 0.0, heuristic(start_x, start_y)});
    bool found = start_index == goal_index;

    while (!open_set.empty()) {
      Node current = open_set.top();
      open_set.pop();
      const int cx = current.index % width;
      const int cy = current.index / width;
      const std::size_t current_local = local_index(cx, cy);
      if (current.g > g_score[current_local] + 1e-9) continue;
      if (current.index == goal_index) {
        found = true;
        break;
      }

      for (int i = 0; i < 8; ++i) {
        const int nx = cx + dx[i];
        const int ny = cy + dy[i];
        if (nx < search_x0 || ny < search_y0 || nx > search_x1 || ny > search_y1) continue;
        const int n_index = toIndex(nx, ny);
        if (!cellHasClearance(n_index, path_min_clear_m_)) continue;
        if (dx[i] != 0 && dy[i] != 0 &&
            (!cellHasClearance(toIndex(nx, cy), path_min_clear_m_) ||
             !cellHasClearance(toIndex(cx, ny), path_min_clear_m_))) continue;

        double turn_cost = 0.0;
        const int parent_index = came_from[current_local];
        if (parent_index >= 0) {
          turn_cost = map_planner_quality::turnPenaltyCost(
              cx - parent_index % width, cy - parent_index / width,
              dx[i], dy[i], turn_penalty_weight_);
        }
        const double tentative_g =
            g_score[current_local]
            + costs[i]
                  * cellCostMultiplier(n_index)
                  * headingAlignFactor(n_index, start_index, start_yaw)
            + turn_cost;
        const std::size_t next_local = local_index(nx, ny);
        if (tentative_g < g_score[next_local]) {
          came_from[next_local] = current.index;
          g_score[next_local] = tentative_g;
          open_set.push({n_index, tentative_g, tentative_g + heuristic(nx, ny)});
        }
      }
    }
    if (!found) return false;
    index_path.clear();
    for (int current = goal_index; current != -1;) {
      index_path.push_back(current);
      if (current == start_index) break;
      const int cx = current % width;
      const int cy = current / width;
      current = came_from[local_index(cx, cy)];
    }
    if (index_path.empty() || index_path.back() != start_index) return false;
    std::reverse(index_path.begin(), index_path.end());
    return true;
  };

  bool found_path = run_a_star(bx0, by0, bx1, by1);
  if (!found_path) {
    EC_TRACE(ec::ASTAR_FAIL, "plan", "bbox_search_no_path retry_full_map");
    RCLCPP_WARN(node_->get_logger(),
                "[map_planner] bbox search found no path; retrying full map "
                "(%dx%d bbox -> %dx%d map)",
                bx1 - bx0 + 1, by1 - by0 + 1, width, height);
    found_path = run_a_star(0, 0, width - 1, height - 1);
    RCLCPP_INFO(node_->get_logger(), "[map_planner] full-map fallback %s",
                found_path ? "succeeded" : "found no path");
  }

  const auto t_search_end = std::chrono::steady_clock::now();
  if (!found_path) {
    fail_reason = "NO_PATH_FOUND";
    return false;
  }

  const std::vector<int> raw_index_path = index_path;

  const auto t_simplify_done = std::chrono::steady_clock::now();
  bool simplify_requires_alignment = false;
  simplifyPath(index_path, start_x, start_y, start_yaw,
               simplify_requires_alignment);
  const auto t_simplify_after = std::chrono::steady_clock::now();

  path.header.stamp = node_->now();
  path.header.frame_id = map_.header.frame_id;

  // Densify simplified waypoints to approximately one-cell spacing so downstream
  // NeuPAN curve indexing has sufficient pose density.
  const double step = map_.info.resolution;
  path.poses.reserve(index_path.size() * 4);
  for (size_t k = 0; k + 1 < index_path.size(); ++k) {
    const int x0 = index_path[k] % width;
    const int y0 = index_path[k] / width;
    const int x1 = index_path[k + 1] % width;
    const int y1 = index_path[k + 1] / width;
    const geometry_msgs::msg::Point p0 = mapToWorld(x0, y0);
    const geometry_msgs::msg::Point p1 = mapToWorld(x1, y1);
    const double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / step)));
    for (int s = 0; s < n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p0.x + t * (p1.x - p0.x);
      pose.pose.position.y = p0.y + t * (p1.y - p0.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  // Always include the final waypoint exactly.
  {
    const int xf = index_path.back() % width;
    const int yf = index_path.back() / width;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = mapToWorld(xf, yf);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  smoothPath(path.poses);
  if (path_validate_enable_) {
    PathQualityResult quality = validatePath(
        path, start_x, start_y, start_yaw, simplify_requires_alignment);
    if (quality.high_kink && raw_index_path.size() > index_path.size()) {
      nav_msgs::msg::Path alternate = buildDensePath(raw_index_path, path.header);
      PathQualityResult alternate_quality = validatePath(
          alternate, start_x, start_y, start_yaw, simplify_requires_alignment);
      auto path_length = [](const nav_msgs::msg::Path& candidate) {
        double length = 0.0;
        for (std::size_t i = 1; i < candidate.poses.size(); ++i) {
          const auto& a = candidate.poses[i - 1].pose.position;
          const auto& b = candidate.poses[i].pose.position;
          length += std::hypot(b.x - a.x, b.y - a.y);
        }
        return length;
      };
      const bool clearance_ok = quality.min_clear_m < 0.0 ||
          alternate_quality.min_clear_m < 0.0 ||
          alternate_quality.min_clear_m + 1e-6 >= quality.min_clear_m;
      const double original_length = path_length(path);
      const double alternate_length = path_length(alternate);
      if (alternate_quality.max_kink_deg + 1.0 < quality.max_kink_deg &&
          clearance_ok && alternate_length <= original_length * 1.15) {
        RCLCPP_INFO(node_->get_logger(),
                    "[map_planner] high-kink fallback selected: %.1fdeg->%.1fdeg, "
                    "length %.2fm->%.2fm, clearance %.2fm->%.2fm",
                    quality.max_kink_deg, alternate_quality.max_kink_deg,
                    original_length, alternate_length, quality.min_clear_m,
                    alternate_quality.min_clear_m);
        path = std::move(alternate);
        quality = alternate_quality;
      }
    }
    last_path_quality_message_ = pathQualityMessage(quality);
  } else if (simplify_requires_alignment) {
    last_path_quality_message_ = "OK|REQUIRES_ALIGNMENT";
  }
  nav_msgs::msg::Path checked = path;
  checked.poses.insert(checked.poses.begin(), start);
  if (!collisionFreePath(checked, path_min_clear_m_)) {
    fail_reason = "PATH_COLLISION";
    path.poses.clear();
    return false;
  }
  const auto t_end = std::chrono::steady_clock::now();

  auto ms_between = [](const auto& a, const auto& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  EC_TRACE(ec::ASTAR_SLOW, "plan_timing", "total=%.1fms",
           ms_between(t_plan_begin, t_end));
  RCLCPP_INFO(node_->get_logger(),
              "[map_planner] plan timing: buildPlanData=%.1fms A*search=%.1fms "
              "simplify=%.1fms densify+smooth=%.1fms total=%.1fms poses=%zu",
              ms_between(t_plan_begin, t_built),
              ms_between(t_built, t_search_end),
              ms_between(t_simplify_done, t_simplify_after),
              ms_between(t_simplify_after, t_end),
              ms_between(t_plan_begin, t_end),
              path.poses.size());
  return true;
}

nav_msgs::msg::Path MapPlanner::buildDensePath(
    const std::vector<int>& index_path, const std_msgs::msg::Header& header) const {
  nav_msgs::msg::Path result;
  result.header = header;
  if (index_path.empty()) return result;
  const int width = static_cast<int>(map_.info.width);
  const double step = map_.info.resolution;
  result.poses.reserve(index_path.size() * 4);
  for (std::size_t k = 0; k + 1 < index_path.size(); ++k) {
    const geometry_msgs::msg::Point p0 =
        mapToWorld(index_path[k] % width, index_path[k] / width);
    const geometry_msgs::msg::Point p1 =
        mapToWorld(index_path[k + 1] % width, index_path[k + 1] / width);
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(segment_length / step)));
    for (int s = 0; s < steps; ++s) {
      const double ratio = static_cast<double>(s) / steps;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = p0.x + ratio * (p1.x - p0.x);
      pose.pose.position.y = p0.y + ratio * (p1.y - p0.y);
      pose.pose.orientation.w = 1.0;
      result.poses.push_back(pose);
    }
  }
  geometry_msgs::msg::PoseStamped goal;
  goal.header = header;
  goal.pose.position =
      mapToWorld(index_path.back() % width, index_path.back() / width);
  goal.pose.orientation.w = 1.0;
  result.poses.push_back(goal);
  smoothPath(result.poses);
  return result;
}

void MapPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& poses) const {
  // Path B: iterative Laplacian smoothing with three safety constraints:
  //   1. Per-point collision check: never move a point into an occupied
  //      cell (read via isFree on the merged plan_data_, so this respects
  //      both static obstacles and sliding_map dynamic obstacles).
  //   2. Bounded deviation: each point may not drift more than
  //      smoothing_max_deviation_m_ from its original position over all
  //      passes. Keeps smoothing local; prevents pathological pull-toward-
  //      a-shortcut behavior on long paths.
  //   3. Minimum clearance: Laplacian smoothing pulls corner points toward
  //      a straight line between their neighbors, which mathematically
  //      cuts corners closer to obstacles even when that violates zero
  //      collision (finding: this is why turns hugged walls
  //      despite A*'s obstacle_cost_weight_ already biasing the raw path
  //      away from them). A point may never end up with less clearance
  //      (per distance_to_blocked_) than it started with, capped at
  //      obstacle_cost_safe_distance_m_ so already-narrow passages are not
  //      forbidden from smoothing entirely.
  //
  // Endpoints (start, goal) are never moved — they must match the robot
  // pose and the requested goal exactly.
  if (smoothing_iterations_ <= 0) return;
  if (poses.size() < 3) return;

  // Snapshot of original positions for the deviation check.
  std::vector<geometry_msgs::msg::Point> original;
  original.reserve(poses.size());
  for (const auto& p : poses) original.push_back(p.pose.position);

  const double max_dev_sq = smoothing_max_deviation_m_ * smoothing_max_deviation_m_;
  const int n = static_cast<int>(poses.size());

  auto isFreeWorld = [&](double wx, double wy) -> bool {
    geometry_msgs::msg::Point q;
    q.x = wx; q.y = wy; q.z = 0.0;
    int mx, my;
    if (!worldToMap(q, mx, my)) return false;
    return cellHasClearance(toIndex(mx, my), path_min_clear_m_);
  };

  auto clearanceWorld = [&](double wx, double wy) -> double {
    geometry_msgs::msg::Point q;
    q.x = wx; q.y = wy; q.z = 0.0;
    int mx, my;
    if (!worldToMap(q, mx, my)) return 0.0;
    const int idx = toIndex(mx, my);
    if (distance_to_blocked_.empty() || idx < 0 ||
        idx >= static_cast<int>(distance_to_blocked_.size())) {
      return std::numeric_limits<double>::max();
    }
    return static_cast<double>(distance_to_blocked_[idx]);
  };

  std::vector<double> min_clearance(n, 0.0);
  if (obstacle_cost_safe_distance_m_ > 0.0) {
    for (int i = 0; i < n; ++i) {
      min_clearance[i] = std::min(clearanceWorld(original[i].x, original[i].y),
                                   obstacle_cost_safe_distance_m_);
    }
  }

  // Protect initial segment (arc length < min(path_protect_initial_m_, total_length * 0.5)) from smoothing.
  int protected_upto = 0;
  if (path_protect_initial_m_ > 0.0 && n >= 3) {
    std::vector<double> s(n, 0.0);
    for (int i = 1; i < n; ++i) {
      s[i] = s[i - 1] + std::hypot(original[i].x - original[i - 1].x,
                                    original[i].y - original[i - 1].y);
    }
    const double protect_len =
        std::min(path_protect_initial_m_, s[n - 1] * 0.5);
    while (protected_upto < n - 1 && s[protected_upto] < protect_len) {
      ++protected_upto;
    }
  }

  for (int iter = 0; iter < smoothing_iterations_; ++iter) {
    std::vector<geometry_msgs::msg::Point> next(poses.size());
    next.front() = poses.front().pose.position;
    next.back()  = poses.back().pose.position;
    for (int i = 1; i < n - 1; ++i) {
      if (i < protected_upto) {
        next[i] = poses[i].pose.position;
        continue;
      }
      const auto& p_prev = poses[i - 1].pose.position;
      const auto& p_cur  = poses[i].pose.position;
      const auto& p_next = poses[i + 1].pose.position;
      const double cx = 0.25 * p_prev.x + 0.5 * p_cur.x + 0.25 * p_next.x;
      const double cy = 0.25 * p_prev.y + 0.5 * p_cur.y + 0.25 * p_next.y;
      const double odx = cx - original[i].x;
      const double ody = cy - original[i].y;
      if (odx * odx + ody * ody > max_dev_sq) {
        const double scale = smoothing_max_deviation_m_ / std::sqrt(odx * odx + ody * ody);
        next[i].x = original[i].x + odx * scale;
        next[i].y = original[i].y + ody * scale;
        next[i].z = 0.0;
      } else {
        next[i].x = cx;
        next[i].y = cy;
        next[i].z = 0.0;
      }
      if (!isFreeWorld(next[i].x, next[i].y) ||
          (obstacle_cost_safe_distance_m_ > 0.0 &&
           clearanceWorld(next[i].x, next[i].y) < min_clearance[i])) {
        next[i] = p_cur;
      }
    }
    for (int i = 0; i < n; ++i) poses[i].pose.position = next[i];
  }
  nav_msgs::msg::Path candidate;
  candidate.poses = poses;
  if (!collisionFreePath(candidate, path_min_clear_m_)) {
    for (int i = 0; i < n; ++i) poses[i].pose.position = original[i];
    RCLCPP_WARN(node_->get_logger(),
                "[map_planner] smoothing violates occupancy/clearance; "
                "keeping the safe pre-smoothing path");
  }
}

MapPlanner::PathQualityResult MapPlanner::validatePath(
    const nav_msgs::msg::Path& path, int start_x, int start_y, double start_yaw,
    bool simplify_requires_alignment) const {
  PathQualityResult quality;
  quality.requires_alignment = simplify_requires_alignment;
  const size_t n = path.poses.size();
  if (n < 2) return quality;

  std::vector<map_planner_quality::Point2d> points;
  points.reserve(n);
  for (const auto& pose : path.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  const map_planner_quality::GeometryMetrics geometry =
      map_planner_quality::computeGeometryMetrics(points, start_yaw);
  quality.heading_err_deg = geometry.heading_error_deg;
  quality.max_kink_deg = geometry.max_kink_deg;
  quality.requires_alignment = quality.requires_alignment ||
      (path_initial_heading_tol_deg_ > 0.0 &&
       quality.heading_err_deg > path_initial_heading_tol_deg_);
  quality.high_kink = quality.max_kink_deg > 45.0;

  double min_clear = std::numeric_limits<double>::max();
  size_t valid_clearance_samples = 0;
  if (!distance_to_blocked_.empty()) {
    for (const auto& pose : path.poses) {
      int mx, my;
      if (!worldToMap(pose.pose.position, mx, my)) continue;
      const int idx = toIndex(mx, my);
      if (idx >= 0 && idx < static_cast<int>(distance_to_blocked_.size())) {
        min_clear = std::min(
            min_clear, static_cast<double>(distance_to_blocked_[idx]));
        ++valid_clearance_samples;
      }
    }
  }
  if (valid_clearance_samples > 0) {
    quality.min_clear_m = min_clear;
  }
  quality.low_clearance = path_min_clear_m_ > 0.0 &&
      valid_clearance_samples > 0 && quality.min_clear_m + 1e-6 < path_min_clear_m_;

  const std::string message = pathQualityMessage(quality);
  RCLCPP_INFO(node_->get_logger(),
              "[map_planner] path quality: heading_err=%.1fdeg max_kink=%.1fdeg "
              "min_clear=%.2fm poses=%zu flags=%s%s",
              quality.heading_err_deg, quality.max_kink_deg, quality.min_clear_m,
              n, message.c_str(), message == "OK" ? "" : " [WARN]");
  if (quality.requires_alignment) {
    EC_TRACE(ec::PATH_INVALID, "path_validator",
             "heading_err=%.1fdeg max_kink=%.1fdeg min_clear=%.2fm",
             quality.heading_err_deg, quality.max_kink_deg, quality.min_clear_m);
    RCLCPP_WARN(node_->get_logger(),
                "[map_planner] path validator: initial heading err %.1fdeg > %.0fdeg "
                "tol or raw A* first step conflicts (start cell %d,%d)",
                quality.heading_err_deg, path_initial_heading_tol_deg_, start_x,
                start_y);
  }
  if (quality.high_kink) {
    RCLCPP_WARN(node_->get_logger(),
                "[map_planner] path validator: max kink %.1fdeg > 45deg",
                quality.max_kink_deg);
  }
  if (quality.low_clearance) {
    RCLCPP_WARN(node_->get_logger(),
                "[map_planner] path validator: min clearance %.2fm < %.2fm",
                quality.min_clear_m, path_min_clear_m_);
  }
  return quality;
}

std::string MapPlanner::pathQualityMessage(
    const PathQualityResult& quality) const {
  std::string message = "OK";
  if (quality.requires_alignment) message += "|REQUIRES_ALIGNMENT";
  if (quality.high_kink) message += "|HIGH_KINK";
  if (quality.low_clearance) message += "|LOW_CLEARANCE";
  return message;
}

// Path C: 2-pass Chamfer 1-SQRT2 distance transform on plan_data_.
// Linear-time (O(N) per pass). After this call distance_to_blocked_[idx]
// is the approximate Euclidean distance, in METERS, from cell `idx` to
// the nearest cell with !isFree() (i.e. an original obstacle or an
// inflated/dynamic obstacle cell). Cells without a valid plan_data_ map
// get an empty distance_to_blocked_, in which case the cost multiplier
// degenerates to 1.0 (= legacy A*).
void MapPlanner::computeDistanceTransform() {
  if (plan_data_.empty()) {
    distance_to_blocked_.clear();
    return;
  }
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double res = map_.info.resolution;
  const int N = width * height;

  // Sentinel "infinity" that fits comfortably in a float, big enough that
  // it cannot be the winner of any std::min on real maps (a 1000x1000 map
  // at 0.05m res has max chord ~70m).
  constexpr float INF_F = 1.0e9f;
  distance_to_blocked_.assign(N, INF_F);
  for (int idx = 0; idx < N; ++idx) {
    if (!isFree(idx)) distance_to_blocked_[idx] = 0.0f;
  }

  const float d1 = static_cast<float>(res);
  const float d2 = static_cast<float>(res * SQRT2);

  // Forward pass: top-to-bottom, left-to-right. Each cell looks at four
  // neighbors already updated this pass: NW, N, NE, W.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int idx = y * width + x;
      if (distance_to_blocked_[idx] == 0.0f) continue;
      float d = distance_to_blocked_[idx];
      if (x > 0) d = std::min(d, distance_to_blocked_[idx - 1] + d1);
      if (y > 0) {
        if (x > 0)         d = std::min(d, distance_to_blocked_[idx - width - 1] + d2);
        d = std::min(d, distance_to_blocked_[idx - width] + d1);
        if (x < width - 1) d = std::min(d, distance_to_blocked_[idx - width + 1] + d2);
      }
      distance_to_blocked_[idx] = d;
    }
  }
  // Backward pass: bottom-to-top, right-to-left. Looks at SE, S, SW, E.
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      const int idx = y * width + x;
      if (distance_to_blocked_[idx] == 0.0f) continue;
      float d = distance_to_blocked_[idx];
      if (x < width - 1) d = std::min(d, distance_to_blocked_[idx + 1] + d1);
      if (y < height - 1) {
        if (x < width - 1) d = std::min(d, distance_to_blocked_[idx + width + 1] + d2);
        d = std::min(d, distance_to_blocked_[idx + width] + d1);
        if (x > 0)         d = std::min(d, distance_to_blocked_[idx + width - 1] + d2);
      }
      distance_to_blocked_[idx] = d;
    }
  }
}

double MapPlanner::cellCostMultiplier(int index) const {
  // Disabled (weight == 0) or distance map not built: legacy step cost.
  if (obstacle_cost_weight_ <= 0.0 || obstacle_cost_safe_distance_m_ <= 0.0) return 1.0;
  if (distance_to_blocked_.empty()) return 1.0;
  if (index < 0 || index >= static_cast<int>(distance_to_blocked_.size())) return 1.0;
  const float d = distance_to_blocked_[index];
  if (d >= static_cast<float>(obstacle_cost_safe_distance_m_)) return 1.0;
  // Linear from 1+weight at d==0 down to 1.0 at d==safe_distance.
  const double frac = (obstacle_cost_safe_distance_m_ - static_cast<double>(d))
                      / obstacle_cost_safe_distance_m_;
  return 1.0 + obstacle_cost_weight_ * frac;
}

double MapPlanner::headingAlignFactor(int n_index, int start_index, double start_yaw) const {
  // Disabled: weight == 0 -> pure cost-A* (current shipping behavior).
  if (start_heading_penalty_weight_ <= 0.0 || start_heading_penalty_radius_m_ <= 0.0) return 1.0;
  const int width = static_cast<int>(map_.info.width);
  const double res = map_.info.resolution;
  const double dx_cells = static_cast<double>(n_index % width - start_index % width);
  const double dy_cells = static_cast<double>(n_index / width - start_index / width);
  const double dist_m = std::hypot(dx_cells, dy_cells) * res;
  // Outside the bias radius: don't constrain — A* picks freely so honestly
  // shorter detours win.
  if (dist_m >= start_heading_penalty_radius_m_) return 1.0;
  // angle from start to candidate cell (world frame; map x/y aligned with world).
  const double angle_to_n = std::atan2(dy_cells, dx_cells);
  double diff = angle_to_n - start_yaw;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  // 1 - cos(diff): 0 at aligned, 2 at 180°. Smooth, no sharp cutoff.
  // Linearly fade with distance so very near the start the penalty is
  // strongest (forcing aligned exit) and at radius_m it vanishes.
  const double fade = 1.0 - dist_m / start_heading_penalty_radius_m_;
  return 1.0 + start_heading_penalty_weight_ * (1.0 - std::cos(diff)) * fade;
}

// Incremental local distance transform update around dynamic obstacle cells.
void MapPlanner::computeLocalDistanceTransform(const std::vector<int>& dynamic_cells) {
  if (plan_data_.empty() || dynamic_cells.empty()) return;

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double res = map_.info.resolution;
  const double max_range = obstacle_cost_safe_distance_m_ +
                           inflation_radius_ + 1.0;

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const float costs[8] = {static_cast<float>(res), static_cast<float>(res),
                          static_cast<float>(res), static_cast<float>(res),
                          static_cast<float>(res * SQRT2), static_cast<float>(res * SQRT2),
                          static_cast<float>(res * SQRT2), static_cast<float>(res * SQRT2)};

  std::vector<std::pair<int, float>> queue;
  queue.reserve(dynamic_cells.size() * 8);

  for (int idx : dynamic_cells) {
    if (plan_data_[idx] == 100) {
      distance_to_blocked_[idx] = 0.0f;
      queue.emplace_back(idx, 0.0f);
    }
  }

  size_t head = 0;
  while (head < queue.size()) {
    const int cur_idx = queue[head].first;
    const float cur_dist = queue[head].second;
    ++head;

    if (cur_dist >= static_cast<float>(max_range)) continue;

    const int cx = cur_idx % width;
    const int cy = cur_idx / width;

    for (int i = 0; i < 8; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;

      const int n_idx = ny * width + nx;
      const float new_dist = cur_dist + costs[i];

      if (new_dist < distance_to_blocked_[n_idx] &&
          new_dist <= static_cast<float>(max_range)) {
        distance_to_blocked_[n_idx] = new_dist;
        queue.emplace_back(n_idx, new_dist);
      }
    }
  }
}

void MapPlanner::inflateMap() {
  inflated_data_ = map_.data;
  if (inflation_cells_ <= 0) return;

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  std::vector<int8_t> result = inflated_data_;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = toIndex(x, y);
      if (map_.data[index] < obstacle_threshold_ || map_.data[index] < 0) continue;
      for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
        for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
          if (std::hypot(dx, dy) * map_.info.resolution > inflation_radius_) continue;
          result[toIndex(nx, ny)] = 100;
        }
      }
    }
  }
  inflated_data_.swap(result);
}

bool MapPlanner::worldToMap(const geometry_msgs::msg::Point& point, int& mx, int& my) const {
  if (!map_ready_) return false;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution;

  mx = static_cast<int>(std::floor((point.x - origin_x) / resolution));
  my = static_cast<int>(std::floor((point.y - origin_y) / resolution));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(map_.info.width) &&
         my < static_cast<int>(map_.info.height);
}

geometry_msgs::msg::Point MapPlanner::mapToWorld(int mx, int my) const {
  geometry_msgs::msg::Point point;
  point.x = map_.info.origin.position.x + (mx + 0.5) * map_.info.resolution;
  point.y = map_.info.origin.position.y + (my + 0.5) * map_.info.resolution;
  point.z = 0.0;
  return point;
}

bool MapPlanner::isFree(int index) const {
  // F2: A* reads the merged plan_data_ (= inflated_data_ + sliding_map dynamic
  // cells). plan_data_ is rebuilt at the entry of plan() and on every
  // sliding_map / map callback. Falls back to inflated_data_ if plan_data_
  // hasn't been built yet (e.g. very first frame before any plan call).
  const auto& grid = plan_data_.empty() ? inflated_data_ : plan_data_;
  if (index < 0 || index >= static_cast<int>(grid.size())) return false;
  const int8_t value = grid[index];
  if (value < 0) return false;
  return value < obstacle_threshold_;
}

bool MapPlanner::cellHasClearance(int index, double minimum) const {
  return isFree(index) &&
      (minimum <= 0.0 ||
       (index < static_cast<int>(distance_to_blocked_.size()) &&
        distance_to_blocked_[index] + 1e-6 >= minimum));
}

bool MapPlanner::hasLineOfSight(int x0, int y0, int x1, int y1,
                                double min_safe_dist_m) const {
  const double required = std::max(min_safe_dist_m, path_min_clear_m_);
  return map_planner_safety::segmentFree(
      x0 + 0.5, y0 + 0.5, x1 + 0.5, y1 + 0.5, [&](int x, int y) {
    if (x < 0 || y < 0 || x >= static_cast<int>(map_.info.width) ||
        y >= static_cast<int>(map_.info.height)) return false;
    return cellHasClearance(toIndex(x, y), required);
  });
}

void MapPlanner::simplifyPath(std::vector<int>& index_path, int start_x,
                              int start_y, double start_yaw,
                              bool& requires_alignment) const {
  requires_alignment = false;
  if (index_path.size() < 2) return;
  const int width = static_cast<int>(map_.info.width);
  const bool heading_guard = path_initial_heading_tol_deg_ > 0.0;

  auto headingConflict = [&](int index) {
    const double dx = static_cast<double>(index % width - start_x);
    const double dy = static_cast<double>(index / width - start_y);
    return heading_guard && map_planner_quality::headingErrorDeg(
        start_yaw, dx, dy) > path_initial_heading_tol_deg_;
  };

  if (index_path.size() == 2) {
    requires_alignment = headingConflict(index_path[1]);
    return;
  }

  std::vector<int> simplified;
  simplified.reserve(index_path.size());
  simplified.push_back(index_path.front());

  size_t i = 0;
  while (i + 1 < index_path.size()) {
    // Look from the far end backward; the first j with line-of-sight from i
    // becomes the next waypoint. This is the funnel/string-pulling step:
    // collapses long straight runs to a single segment.
    size_t j = index_path.size() - 1;
    while (j > i + 1) {
      const int xi = index_path[i] % width;
      const int yi = index_path[i] / width;
      const int xj = index_path[j] % width;
      const int yj = index_path[j] / width;
      // Path C: require the shortcut to stay at least
      // line_of_sight_safe_distance_m_ away from any obstacle. Without this
      // string-pull would happily collapse a centered cost-A* path back to
      // a wall-adjacent diagonal whenever that diagonal is binary-free.
      if (!hasLineOfSight(xi, yi, xj, yj, line_of_sight_safe_distance_m_)) {
        --j;
        continue;
      }
      if (i == 0 && headingConflict(index_path[j])) {
        --j;
        continue;
      }
      break;
    }
    if (i == 0 && headingConflict(index_path[j])) {
      requires_alignment = true;
    }
    simplified.push_back(index_path[j]);
    i = j;
  }
  index_path.swap(simplified);
}

void MapPlanner::planSequenceService(
    const std::shared_ptr<nav_interfaces::srv::PlanSequence::Request> req,
    std::shared_ptr<nav_interfaces::srv::PlanSequence::Response> res) {
  if (!map_ready_) {
    res->success = false;
    res->message = "Map not ready for sequence planning";
    return;
  }
  if (req->goals.empty()) {
    res->success = false;
    res->message = "Sequence goals are empty";
    return;
  }

  geometry_msgs::msg::PoseStamped start = req->start;
  if (start.header.frame_id.empty()) {
    start.header.frame_id = map_.header.frame_id;
  }
  if (start.header.frame_id != map_.header.frame_id) {
    res->success = false;
    res->message = "Sequence start frame does not match map frame";
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> goals = req->goals;
  if (req->loop) {
    goals.push_back(req->goals.front());
  }

  nav_msgs::msg::Path sequence_path;
  sequence_path.header.frame_id = map_.header.frame_id;
  sequence_path.header.stamp = node_->now();
  for (auto goal : goals) {
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = map_.header.frame_id;
    }
    if (goal.header.frame_id != map_.header.frame_id) {
      res->success = false;
      res->message = "Sequence goal frame does not match map frame";
      return;
    }
    nav_msgs::msg::Path segment;
    std::string fail_reason;
    if (!plan(start, goal, segment, fail_reason) || segment.poses.empty()) {
      res->success = false;
      res->message = "Failed to plan one sequence segment: " + fail_reason;
      return;
    }
    const std::size_t begin = sequence_path.poses.empty() ? 0 : 1;
    sequence_path.poses.insert(sequence_path.poses.end(),
                               segment.poses.begin() + begin,
                               segment.poses.end());
    start = goal;
  }

  res->path = sequence_path;
  res->success = true;
  res->message = "Sequence path planned for visualization";
}

bool MapPlanner::collisionFreePath(const nav_msgs::msg::Path& path,
                                  double min_clearance) const {
  if (path.poses.empty() || !map_ready_) return false;
  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  auto free_cell = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= static_cast<int>(map_.info.width) ||
        y >= static_cast<int>(map_.info.height)) return false;
    return cellHasClearance(toIndex(x, y), min_clearance);
  };
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto& a = path.poses[i == 0 ? 0 : i - 1].pose.position;
    const auto& b = path.poses[i].pose.position;
    if (!map_planner_safety::segmentFree(
            (a.x - ox) / res, (a.y - oy) / res,
            (b.x - ox) / res, (b.y - oy) / res, free_cell)) return false;
  }
  return true;
}

void MapPlanner::validatePathService(
    const std::shared_ptr<nav_interfaces::srv::ValidatePath::Request> req,
    std::shared_ptr<nav_interfaces::srv::ValidatePath::Response> res) {
  res->success = false;
  res->message = "MAP_NOT_READY";
  if (!map_ready_) return;
  buildPlanData();
  if (require_fresh_sliding_map_ && !dynamic_map_error_.empty()) {
    res->message = dynamic_map_error_;
    return;
  }
  res->message = "INVALID_PATH";
  if (req->path.header.frame_id != map_.header.frame_id ||
      req->path.poses.size() < 2 || !std::isfinite(req->max_length_m)) return;
  geometry_msgs::msg::PoseStamped robot;
  if (!getRobotPose(robot)) {
    res->message = "ROBOT_POSE_UNAVAILABLE";
    return;
  }
  std::size_t nearest = 0;
  double distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < req->path.poses.size(); ++i) {
    const auto& p = req->path.poses[i].pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
    const double d = std::hypot(p.x - robot.pose.position.x,
                                p.y - robot.pose.position.y);
    if (d < distance) {
      distance = d;
      nearest = i;
    }
  }
  if (req->check_start_connection &&
      distance > std::max(0.0, path_max_start_offset_m_)) {
    res->message = "PATH_START_TOO_FAR";
    return;
  }
  // Keep the original polyline. Adding robot -> nearest point manufactures
  // the perpendicular connector seen in the bag, and invalidates A* geometry.
  nearest = std::min(nearest, req->path.poses.size() - 2);
  nav_msgs::msg::Path prepared;
  prepared.header = req->path.header;
  prepared.header.stamp = node_->now();
  double arc = 0.0;
  for (std::size_t i = nearest; i < req->path.poses.size(); ++i) {
    auto point = req->path.poses[i];
    point.header = prepared.header;
    if (!prepared.poses.empty()) {
      const auto& a = prepared.poses.back().pose.position;
      const double length = std::hypot(point.pose.position.x - a.x,
                                       point.pose.position.y - a.y);
      if (length < 1e-9) continue;
      if (req->max_length_m > 0.0 && arc + length > req->max_length_m) {
        const double u = (req->max_length_m - arc) / length;
        point.pose.position.x = a.x + u * (point.pose.position.x - a.x);
        point.pose.position.y = a.y + u * (point.pose.position.y - a.y);
        prepared.poses.push_back(point);
        break;
      }
      arc += length;
    }
    prepared.poses.push_back(point);
  }
  nav_msgs::msg::Path checked = prepared;
  if (req->check_start_connection) checked.poses.insert(checked.poses.begin(), robot);
  if (prepared.poses.size() < 2 ||
      !collisionFreePath(checked, path_min_clear_m_)) {
    res->message = "PATH_COLLISION_OR_LOW_CLEARANCE";
    return;
  }
  res->success = true;
  res->message = "OK";
  res->path = std::move(prepared);
}

void MapPlanner::planService(
    const std::shared_ptr<nav_interfaces::srv::PlanPath::Request> req,
    std::shared_ptr<nav_interfaces::srv::PlanPath::Response> res) {
  // Always return true (RPC succeeded) and carry the actual outcome in
  // res.success/res.message, so callers (mission_manager) can distinguish
  // failure reasons instead of only seeing a generic call failure.
  if (!map_ready_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Map not ready for planning.");
    if (!req->shadow_only) publishPlanResult(false);
    res->success = false;
    res->message = "MAP_NOT_READY";
    return;
  }
  geometry_msgs::msg::PoseStamped start_pose;
  if (!getRobotPose(start_pose)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Unable to get robot pose.");
    if (!req->shadow_only) publishPlanResult(false);
    res->success = false;
    res->message = "ROBOT_POSE_UNAVAILABLE";
    return;
  }
  geometry_msgs::msg::PoseStamped goal = req->goal;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_.header.frame_id;
  }
  if (goal.header.frame_id != map_.header.frame_id) {
    RCLCPP_WARN(node_->get_logger(),
                "Goal frame (%s) does not match map frame (%s).",
                goal.header.frame_id.c_str(), map_.header.frame_id.c_str());
    if (!req->shadow_only) publishPlanResult(false);
    res->success = false;
    res->message = "GOAL_FRAME_MISMATCH";
    return;
  }
  nav_msgs::msg::Path path;
  std::string fail_reason;
  const bool success = plan(start_pose, goal, path, fail_reason);
  if (!req->shadow_only) publishPlanResult(success);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to plan a path: %s", fail_reason.c_str());
    res->success = false;
    res->message = fail_reason;
    return;
  }
  res->path = path;
  res->success = true;
  res->message = last_path_quality_message_;
  if (publish_path_ && !req->shadow_only) path_pub_->publish(path);
}

#ifndef MAP_PLANNER_RECORDED_REPLAY
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_planner");
  MapPlanner planner(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
