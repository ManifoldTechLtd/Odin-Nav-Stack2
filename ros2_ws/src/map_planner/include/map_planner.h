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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_interfaces/srv/plan_path.hpp>
#include <nav_interfaces/srv/plan_sequence.hpp>
#include <nav_interfaces/srv/validate_path.hpp>

#include "path_quality.h"
#include <string>
#include <vector>
#include <memory>

class MapPlanner {
public:
  explicit MapPlanner(const rclcpp::Node::SharedPtr& node);

private:
  friend class MapPlannerRecordedReplay;
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
  // Dynamic obstacles from sliding_map's accumulated occupancy grid (in odom frame),
  // merged into plan_data_ before each A* search.
  void slidingMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
  void refreshMapVisualization();
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal);
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose) const;
  // fail_reason is set on failure to one of: OUT_OF_MAP, START_OCCUPIED,
  // GOAL_OCCUPIED, START_AND_GOAL_OCCUPIED, NO_PATH_FOUND.
  bool plan(const geometry_msgs::msg::PoseStamped& start,
            const geometry_msgs::msg::PoseStamped& goal,
            nav_msgs::msg::Path& path, std::string& fail_reason);
  void planService(const std::shared_ptr<nav_interfaces::srv::PlanPath::Request> req,
                   std::shared_ptr<nav_interfaces::srv::PlanPath::Response> res);
  void planSequenceService(
      const std::shared_ptr<nav_interfaces::srv::PlanSequence::Request> req,
      std::shared_ptr<nav_interfaces::srv::PlanSequence::Response> res);
  void validatePathService(
      const std::shared_ptr<nav_interfaces::srv::ValidatePath::Request> req,
      std::shared_ptr<nav_interfaces::srv::ValidatePath::Response> res);
  bool collisionFreePath(const nav_msgs::msg::Path& path,
                         double min_clearance = 0.0) const;
  void inflateMap();
  // Rebuild plan_data_ = inflated_data_ + sliding_map occupied cells (with inflation),
  // transformed from sliding_map's frame into /map's frame.
  void buildPlanData();
  // Global coarse persistent memory (map frame, default 0.2m resolution):
  // records LiDAR hits outside static walls without temporal decay or sliding-window loss.
  void initGlobalMemory();
  void stampGlobalMemory(double wx_map, double wy_map);
  void mergeGlobalMemory();
  void publishInflatedMap();
  void publishPlanResult(bool success);
  bool worldToMap(const geometry_msgs::msg::Point& point, int& mx, int& my) const;
  geometry_msgs::msg::Point mapToWorld(int mx, int my) const;
  inline int toIndex(int mx, int my) const {
    return my * static_cast<int>(map_.info.width) + mx;
  }
  bool isFree(int index) const;
  bool cellHasClearance(int index, double minimum) const;
  // Line-of-sight check for string-pulling on the inflated grid (with optional
  // minimum clearance threshold min_safe_dist_m).
  bool hasLineOfSight(int x0, int y0, int x1, int y1,
                      double min_safe_dist_m = 0.0) const;
  // Simplify A* index path via line-of-sight string-pulling while enforcing
  // path_initial_heading_tol_deg_ on the first segment from start_yaw.
  void simplifyPath(std::vector<int>& index_path, int start_x, int start_y,
                    double start_yaw, bool& requires_alignment) const;
  // Evaluate final path heading error, maximum kink angle, and minimum obstacle clearance.
  struct PathQualityResult {
    double heading_err_deg{0.0};
    double max_kink_deg{0.0};
    double min_clear_m{-1.0};
    bool requires_alignment{false};
    bool high_kink{false};
    bool low_clearance{false};
  };
  PathQualityResult validatePath(const nav_msgs::msg::Path& path, int start_x,
                                 int start_y, double start_yaw,
                                 bool simplify_requires_alignment) const;
  std::string pathQualityMessage(const PathQualityResult& quality) const;
  // Chamfer 1-SQRT2 distance transform from blocked cells into distance_to_blocked_ (in meters).
  void computeDistanceTransform();

  // Incremental distance transform update seeded from dynamic obstacle cells.
  void computeLocalDistanceTransform(const std::vector<int>& dynamic_cells);
  // Linear obstacle proximity cost multiplier on A* edges.
  double cellCostMultiplier(int index) const;
  // Heading-aware A* bias discouraging initial steps opposite to start_yaw.
  double headingAlignFactor(int n_index, int start_index, double start_yaw) const;
  // Laplacian smoothing on the densified polyline with collision and deviation checks.
  void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& poses) const;
  nav_msgs::msg::Path buildDensePath(
      const std::vector<int>& index_path, const std_msgs::msg::Header& header) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sliding_map_sub_;
  rclcpp::CallbackGroup::SharedPtr sliding_map_input_group_;
  rclcpp::TimerBase::SharedPtr map_visualization_timer_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_sliding_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr visualized_sliding_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_result_pub_;
  rclcpp::Service<nav_interfaces::srv::PlanPath>::SharedPtr plan_service_;
  rclcpp::Service<nav_interfaces::srv::PlanSequence>::SharedPtr plan_sequence_service_;
  rclcpp::Service<nav_interfaces::srv::ValidatePath>::SharedPtr validate_path_service_;
  bool publish_path_{true};
  std::string plan_service_name_{"plan"};

  nav_msgs::msg::OccupancyGrid map_;
  std::vector<int8_t> inflated_data_;  // static base: inflated /map
  nav_msgs::msg::OccupancyGrid sliding_map_;
  bool sliding_map_ready_{false};
  bool require_fresh_sliding_map_{true};
  std::string dynamic_map_error_;
  double path_max_start_offset_m_{0.20};
  std::vector<int8_t> plan_data_;
  bool map_ready_{false};
  double inflation_radius_{0.25};
  int obstacle_threshold_{50};
  int inflation_cells_{1};
  double sliding_map_max_age_sec_{2.0};
  // Visualization throttling and window cropping for /inflated_map on large maps
  double inflated_map_publish_period_sec_{1.0};
  double inflated_map_window_m_{15.0};
  int smoothing_iterations_{20};
  double smoothing_max_deviation_m_{0.20};

  double obstacle_cost_weight_{3.0};
  double obstacle_cost_safe_distance_m_{0.5};
  double line_of_sight_safe_distance_m_{0.25};
  std::vector<float> distance_to_blocked_;

  // Static distance transform cache and per-frame dynamic cell indices
  std::vector<float> distance_static_cache_;
  std::vector<int> dynamic_touched_;
  bool static_distance_ready_ = false;
  rclcpp::Time static_distance_built_at_{0, 0, RCL_ROS_TIME};

  // Global coarse persistent memory
  bool use_global_memory_{false};
  double global_memory_res_{0.2};
  double global_memory_inflation_radius_{0.35};
  double global_memory_exempt_radius_m_{0.0};
  std::vector<int8_t> global_memory_;
  int gm_w_{0}, gm_h_{0};
  double gm_ox_{0.0}, gm_oy_{0.0};
  bool gm_ready_{false};
  size_t gm_cell_count_{0};
  geometry_msgs::msg::Point gm_exempt_start_, gm_exempt_goal_;
  bool gm_exempt_valid_{false};

  double start_heading_penalty_weight_{2.0};
  double start_heading_penalty_radius_m_{1.0};
  double turn_penalty_weight_{0.0};

  // Initial segment heading tolerance, smoothing protection length, and path validation
  double path_initial_heading_tol_deg_{45.0};
  double path_protect_initial_m_{1.0};
  bool path_validate_enable_{true};
  double path_min_clear_m_{0.10};
  std::string last_path_quality_message_{"OK"};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
