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

#include "mission_manager/mission_manager.h"

#include <chrono>
#include <exception>

bool MissionManager::startInitialRoutePlan() {
  if (current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    return false;
  }
  clearRouteSegment();
  resetInstallGateState("waypoint");
  active_path_.poses.clear();
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_safety_waiting_ = true;
  initial_plan_requested_ = true;
  initial_plan_attempts_ = 0;
  next_initial_plan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING,
               "Global route planning in progress");
  publishEvent("PLANNING", "Waiting for initial global route");
  processInitialRoutePlan();
  return true;
}

void MissionManager::processInitialRoutePlan() {
  if (initial_plan_future_.valid() &&
      initial_plan_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto result = initial_plan_future_.get();
    const bool current = result.mission_id == mission_id_ &&
        result.waypoint_index == current_index_ &&
        result.segment_generation == route_segment_generation_ &&
        result.path_generation == path_goal_generation_.load() &&
        (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME);
    if (current) {
      if (result.planned) {
        installInitialRoute(result.path);
      } else {
        RCLCPP_WARN(node_->get_logger(), "[initial_plan] failed: %s", result.flags.c_str());
        const bool transient = result.flags.find("DYNAMIC_MAP") != std::string::npos ||
            result.flags == "PLAN_SERVICE_UNAVAILABLE" ||
            result.flags == "ROBOT_POSE_UNAVAILABLE";
        const bool occupied = result.flags == "START_OCCUPIED" ||
            result.flags == "START_AND_GOAL_OCCUPIED" || result.flags == "GOAL_OCCUPIED";
        if (transient || (occupied && initial_plan_attempts_ < plan_max_retries_)) {
          if (occupied) ++initial_plan_attempts_;
          initial_plan_requested_ = true;
          next_initial_plan_at_ = node_->now() + rclcpp::Duration::from_seconds(
              occupied ? plan_retry_backoff_sec_ : periodic_replan_sec_);
          publishEvent("PLANNING_RETRY", result.flags);
        } else if (result.flags == "PLAN_RESPONSE_TIMEOUT" ||
                   result.flags == "PLAN_SERVICE_CALL_FAILED" ||
                   result.flags == "PLAN_REQUEST_CANCELLED") {
          // A timed-out ROS service can still be computing on the server.
          // Do not automatically enqueue the same full route again.
          stopNavigation();
          transitionTo(State::FAILED, "Initial global planning failed: " + result.flags);
        } else {
          stopNavigation();
          skipCurrentWaypointOrFail(result.flags);
        }
      }
    }
  }
  if (!initial_plan_requested_ || initial_plan_future_.valid() ||
      (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) ||
      !pathScanFresh() || node_->now() < next_initial_plan_at_ ||
      current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) return;

  const auto goal = waypointPose(waypoints_[current_index_]);
  const auto mission = mission_id_;
  const int waypoint = current_index_;
  const auto segment_generation = route_segment_generation_;
  const auto path_generation = path_goal_generation_.load();
  const auto client = plan_client_;
  const double budget = initial_plan_timeout_sec_, discovery = plan_timeout_sec_;
  initial_plan_requested_ = false;
  initial_plan_future_ = std::async(std::launch::async,
      [this, goal, mission, waypoint, segment_generation, path_generation,
       client, budget, discovery]() {
    PeriodicReplanResult result;
    result.mission_id = mission;
    result.waypoint_index = waypoint;
    result.segment_generation = segment_generation;
    result.path_generation = path_generation;
    result.flags = "PLAN_SERVICE_CALL_FAILED";
    try {
      auto request = std::make_shared<nav_interfaces::srv::PlanPath::Request>();
      request->goal = goal;
      request->shadow_only = true;
      auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanPath>,
                               decltype(request)>(
          client, request, budget, &result.flags, discovery);
      if (response) {
        result.flags = response->message;
        result.planned = response->success && !response->path.poses.empty();
        if (result.planned) result.path = std::move(response->path);
      }
    } catch (const std::exception &ex) {
      RCLCPP_WARN(node_->get_logger(), "[initial_plan] service exception: %s", ex.what());
      result.flags = "PLAN_SERVICE_CALL_FAILED";
    }
    return result;
  });
}

void MissionManager::installInitialRoute(nav_msgs::msg::Path path) {
  global_route_ = path;
  RouteTarget target;
  geometry_msgs::msg::PoseStamped pose;
  std::string reason = "ROBOT_POSE_UNAVAILABLE";
  bool prepared = getRobotPose(pose);
  if (prepared && route_segment_enable_) {
    const auto targets = routeTargets(pose, true);
    prepared = !targets.empty();
    if (prepared) {
      target = targets.front();
      path.poses.resize(target.anchor + 1);
    } else {
      reason = "ROUTE_TARGET_UNAVAILABLE";
    }
  }
  if (!prepared || !validateForInstall(path, reason)) {
    RCLCPP_WARN(node_->get_logger(), "[initial_plan] route cached, awaiting safe segment: %s",
                reason.c_str());
    publishEvent("PATH_SAFETY_WAIT", reason);
    last_periodic_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }
  if (!path_client_ || !path_client_->action_server_is_ready()) {
    publishEvent("PATH_SAFETY_WAIT", "PATH_ACTION_UNAVAILABLE");
    last_periodic_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }
  active_path_ = path;
  path_safety_waiting_ = false;
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  last_feedback_distance_ = -1.0;
  path_started_at_ = node_->now();
  last_path_feedback_at_ = path_started_at_;
  last_feedback_progress_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_periodic_replan_at_ = path_started_at_;
  sendPathGoal(path, target.route ? &target : nullptr);
  publishGlobalPathVisualization();
  RCLCPP_INFO(node_->get_logger(), "[initial_plan] installed poses=%zu", path.poses.size());
}
