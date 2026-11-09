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

bool MissionManager::pathScanFresh() const {
  const auto stamp = path_safety_scan_stamp_ns_.load();
  const auto received = path_safety_scan_received_ns_.load();
  if (stamp == 0 || received == 0) return false;
  const auto now = node_->now().nanoseconds();
  const double age = static_cast<double>(now - stamp) * 1e-9;
  const double receive_age = static_cast<double>(now - received) * 1e-9;
  return age >= -0.1 && age <= path_safety_scan_max_age_sec_ &&
         receive_age >= 0.0 && receive_age <= path_safety_scan_max_age_sec_;
}

bool MissionManager::validateForInstall(nav_msgs::msg::Path &path,
                                       std::string &reason) {
  if (navigation_mode_ != "replan_periodic") return path.poses.size() >= 2;
  if (!pathScanFresh()) {
    reason = "SCAN_STALE";
    return false;
  }
  if (!validate_path_client_ || !validate_path_client_->service_is_ready()) {
    return path.poses.size() >= 2;
  }
  auto request = std::make_shared<nav_interfaces::srv::ValidatePath::Request>();
  request->path = path;
  request->check_start_connection = true;
  auto response = callSync<rclcpp::Client<nav_interfaces::srv::ValidatePath>,
                           decltype(request)>(validate_path_client_, request, 1.0);
  if (!response || !response->success) {
    reason = response ? response->message : "PATH_VALIDATION_UNAVAILABLE";
    return false;
  }
  if (!pathScanFresh()) {
    reason = "SCAN_STALE";
    return false;
  }
  path = std::move(response->path);
  return path.poses.size() >= 2;
}

void MissionManager::holdForPathSafety(const std::string &reason) {
  if (path_safety_waiting_) return;
  stopNavigation();
  active_path_.poses.clear();
  path_safety_waiting_ = true;
  last_periodic_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  RCLCPP_WARN(node_->get_logger(), "[path_safety] stopped; waiting for safe path: %s",
              reason.c_str());
  publishEvent("PATH_SAFETY_WAIT", reason);
}

void MissionManager::processPathSafety() {
  if (local_planner_mode_ != "action" || navigation_mode_ != "replan_periodic") return;
  const bool navigating = state_ == State::NAVIGATING || state_ == State::RETURNING_HOME;
  if (path_safety_future_.valid() &&
      path_safety_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto result = path_safety_future_.get();
    if (navigating && !path_safety_waiting_ && !path_done_.load() &&
        result.generation == path_goal_generation_.load() && !result.valid) {
      holdForPathSafety(result.reason);
    }
  }
  if (!navigating) return;
  if (!pathScanFresh()) {
    holdForPathSafety("SCAN_STALE");
    return;
  }
  if (path_safety_waiting_ || active_path_.poses.size() < 2 ||
      path_done_.load() || path_safety_future_.valid() ||
      !validate_path_client_ || !validate_path_client_->service_is_ready()) return;
  const auto now = node_->now();
  if (last_path_safety_check_.nanoseconds() != 0 &&
      (now - last_path_safety_check_).seconds() < 0.5) return;
  last_path_safety_check_ = now;
  const auto path = active_path_;
  const auto generation = path_goal_generation_.load();
  const auto client = validate_path_client_;
  path_safety_future_ = std::async(std::launch::async, [this, path, generation, client]() {
    PathSafetyResult result;
    result.generation = generation;
    result.reason = "PATH_VALIDATION_UNAVAILABLE";
    try {
      auto request = std::make_shared<nav_interfaces::srv::ValidatePath::Request>();
      request->path = path;
      request->check_start_connection = false;
      auto response = callSync<rclcpp::Client<nav_interfaces::srv::ValidatePath>,
                               decltype(request)>(client, request, 1.0);
      if (response) {
        result.valid = response->success;
        result.reason = response->message;
      }
    } catch (const std::exception &ex) {
      result.reason = ex.what();
    }
    return result;
  });
}
