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
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_interfaces/PlanPath.h>
#include <nav_interfaces/PlanSequence.h>
#include <tf2_ros/buffer.h>

#include "path_quality.h"
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

class MapPlanner {
public:
  MapPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  // Dynamic obstacles from sliding_map's accumulated occupancy grid (in odom frame).
  // Merged into plan_data_ before each A* search.
  void slidingMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;
  // fail_reason is set on failure to one of: OUT_OF_MAP, START_OCCUPIED,
  // GOAL_OCCUPIED, START_AND_GOAL_OCCUPIED, NO_PATH_FOUND.
  bool plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
            nav_msgs::Path& path, std::string& fail_reason);
  bool planService(nav_interfaces::PlanPath::Request& req, nav_interfaces::PlanPath::Response& res);
  bool planSequenceService(nav_interfaces::PlanSequence::Request& req,
                           nav_interfaces::PlanSequence::Response& res);
  void inflateMap();
  // Rebuild plan_data_ = inflated_data_ + sliding_map occupied cells (with inflation),
  // transformed from sliding_map's frame into /map's frame.
  void buildPlanData();
  // Global coarse-grid persistent memory (map frame, default 0.2m resolution).
  // Records LiDAR hits in non-occupied areas of the static prior map without
  // time decay or rolling-window expiration, and merges them into plan_data_
  // with start/goal exemption disks.
  void initGlobalMemory();
  void stampGlobalMemory(double wx_map, double wy_map);
  void mergeGlobalMemory();
  void publishInflatedMap();
  void publishPlanResult(bool success);
  bool worldToMap(const geometry_msgs::Point& point, int& mx, int& my) const;
  geometry_msgs::Point mapToWorld(int mx, int my) const;
  inline int toIndex(int mx, int my) const { return my * static_cast<int>(map_.info.width) + mx; }
  bool isFree(int index) const;
  // Bresenham line-of-sight check across the inflated grid. When min_safe_dist_m > 0,
  // every cell on the segment must also satisfy distance_to_blocked_ >= min_safe_dist_m.
  bool hasLineOfSight(int x0, int y0, int x1, int y1, double min_safe_dist_m = 0.0) const;
  // String-pulling simplification with optional initial heading constraint
  // (path_initial_heading_tol_deg_). Sets requires_alignment if the initial step
  // still exceeds the heading tolerance.
  void simplifyPath(std::vector<int>& index_path, int start_x, int start_y,
                    double start_yaw, bool& requires_alignment) const;
  // Final path validator: evaluates initial heading error, maximum kink angle,
  // and minimum obstacle clearance along the planned path.
  struct PathQualityResult {
    double heading_err_deg{0.0};
    double max_kink_deg{0.0};
    double min_clear_m{-1.0};
    bool requires_alignment{false};
    bool high_kink{false};
    bool low_clearance{false};
  };
  PathQualityResult validatePath(const nav_msgs::Path& path, int start_x,
                                 int start_y, double start_yaw,
                                 bool simplify_requires_alignment) const;
  std::string pathQualityMessage(const PathQualityResult& quality) const;
  // Chamfer 1-SQRT2 2-pass distance transform from blocked cells (in meters).
  void computeDistanceTransform();

  // Incremental local distance transform update around dynamic obstacle cells,
  // starting from the cached static distance transform.
  void computeLocalDistanceTransform(const std::vector<int>& dynamic_cells);
  // Linear obstacle-proximity penalty multiplier on A* edge cost:
  //   d >= obstacle_cost_safe_distance_m_  -> 1.0
  //   d == 0                               -> 1 + obstacle_cost_weight_
  double cellCostMultiplier(int index) const;
  // Heading-aware A* bias within start_heading_penalty_radius_m_ of the start cell:
  //   1 + alpha * (1 - cos(diff)) * (1 - d/radius)
  double headingAlignFactor(int n_index, int start_index, double start_yaw) const;
  // Collision-checked Laplacian smoothing on the densified polyline, bounded by
  // smoothing_max_deviation_m_.
  void smoothPath(std::vector<geometry_msgs::PoseStamped>& poses) const;
  nav_msgs::Path buildDensePath(const std::vector<int>& index_path,
                                const std_msgs::Header& header) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber sliding_map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  ros::Publisher inflated_map_pub_;
  ros::Publisher plan_result_pub_;
  ros::ServiceServer plan_service_;
  ros::ServiceServer plan_sequence_service_;
  bool publish_path_{true};
  std::string plan_service_name_{"/map_planner/plan"};

  nav_msgs::OccupancyGrid map_;
  std::vector<int8_t> inflated_data_;  // static base: inflated /map
  nav_msgs::OccupancyGrid sliding_map_;
  bool sliding_map_ready_{false};
  std::vector<int8_t> plan_data_;
  bool map_ready_{false};
  double inflation_radius_{0.25};
  int obstacle_threshold_{50};
  int inflation_cells_{1};
  double sliding_map_max_age_sec_{2.0};
  // Visualization throttling and windowing for /inflated_map (0 = disabled).
  double inflated_map_publish_period_sec_{1.0};
  double inflated_map_window_m_{15.0};
  ros::Time last_inflated_pub_time_{0};
  int smoothing_iterations_{20};
  double smoothing_max_deviation_m_{0.20};

  double obstacle_cost_weight_{3.0};
  double obstacle_cost_safe_distance_m_{0.5};
  double line_of_sight_safe_distance_m_{0.25};
  std::vector<float> distance_to_blocked_;

  // Cached static distance transform and indices of cells modified by dynamic obstacles.
  std::vector<float> distance_static_cache_;
  std::vector<int> dynamic_touched_;
  bool static_distance_ready_ = false;
  ros::Time static_distance_built_at_;

  // Global coarse-grid persistent memory state
  bool use_global_memory_{false};
  double global_memory_res_{0.2};
  double global_memory_inflation_radius_{0.35};
  double global_memory_exempt_radius_m_{0.0};
  std::vector<int8_t> global_memory_;
  int gm_w_{0}, gm_h_{0};
  double gm_ox_{0.0}, gm_oy_{0.0};
  bool gm_ready_{false};
  size_t gm_cell_count_{0};
  geometry_msgs::Point gm_exempt_start_, gm_exempt_goal_;
  bool gm_exempt_valid_{false};

  double start_heading_penalty_weight_{2.0};
  double start_heading_penalty_radius_m_{1.0};
  double turn_penalty_weight_{0.0};

  // Path executability and validation parameters
  double path_initial_heading_tol_deg_{45.0};
  double path_protect_initial_m_{1.0};
  bool path_validate_enable_{true};
  double path_min_clear_m_{0.10};
  std::string last_path_quality_message_{"OK"};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
