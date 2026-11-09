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
#include "mission_manager/ec_trace.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <tf2/time.h>

namespace {
inline tf2::Duration tfTimeout(double sec) { return tf2::durationFromSec(sec); }

inline const char* resultCodeName(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:   return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:  return "CANCELED";
    default:                                   return "UNKNOWN";
  }
}
}  // namespace

MissionManager::MissionManager(const rclcpp::Node::SharedPtr &node) : node_(node) {
  // Concurrency model: cb_group_main_ is MutuallyExclusive (serializing services,
  // subscriptions, and timers); cb_group_clients_ is Reentrant (preventing deadlocks
  // when waiting on service/action responses inside process() or worker threads).
  cb_group_main_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_clients_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  map_frame_ = node_->declare_parameter("map_frame", map_frame_);
  base_frame_ = node_->declare_parameter("base_frame", base_frame_);
  odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);
  odom_topic_ = node_->declare_parameter("odom_topic", odom_topic_);
  plan_service_name_ = node_->declare_parameter("plan_service", plan_service_name_);
  nav_path_action_name_ = node_->declare_parameter("nav_path_action", nav_path_action_name_);
  stop_topic_ = node_->declare_parameter("stop_topic", stop_topic_);
  sru_goal_topic_ = node_->declare_parameter("goal_topic", sru_goal_topic_);
  sru_enable_topic_ = node_->declare_parameter("enable_topic", sru_enable_topic_);
  sru_cmd_vel_topic_ =
      node_->declare_parameter("planner_cmd_vel_topic", sru_cmd_vel_topic_);
  plan_timeout_sec_ = node_->declare_parameter("plan_timeout_sec", plan_timeout_sec_);
  initial_plan_timeout_sec_ = std::max(0.1, node_->declare_parameter(
      "initial_plan_timeout_sec", initial_plan_timeout_sec_));
  plan_max_retries_ = node_->declare_parameter("plan_max_retries", plan_max_retries_);
  plan_retry_backoff_sec_ =
      node_->declare_parameter("plan_retry_backoff_sec", plan_retry_backoff_sec_);
  replan_cooldown_sec_ = node_->declare_parameter("replan_cooldown_sec", replan_cooldown_sec_);
  navigation_mode_ = node_->declare_parameter("navigation_mode", navigation_mode_);
  periodic_replan_sec_ = node_->declare_parameter("periodic_replan_sec", periodic_replan_sec_);
  route_segment_enable_ = node_->declare_parameter("route_segment_enable", route_segment_enable_);
  route_segment_length_m_ =
      node_->declare_parameter("route_segment_length_m", route_segment_length_m_);
  route_segment_overlap_m_ =
      node_->declare_parameter("route_segment_overlap_m", route_segment_overlap_m_);
  path_safety_scan_max_age_sec_ = std::max(0.1, node_->declare_parameter(
      "path_safety_scan_max_age_sec", path_safety_scan_max_age_sec_));
  recovery_shadow_enable_ =
      node_->declare_parameter("recovery_shadow_enable", recovery_shadow_enable_);
  recovery_behavior_enable_ =
      node_->declare_parameter("recovery_behavior_enable", recovery_behavior_enable_);
  recovery_candidate_max_age_sec_ = node_->declare_parameter(
      "recovery_candidate_max_age_sec", recovery_candidate_max_age_sec_);
  recovery_candidate_max_start_dist_m_ = node_->declare_parameter(
      "recovery_candidate_max_start_dist_m", recovery_candidate_max_start_dist_m_);
  nav_watchdog_timeout_sec_ =
      node_->declare_parameter("nav_watchdog_timeout_sec", nav_watchdog_timeout_sec_);
  nav_stall_timeout_sec_ =
      node_->declare_parameter("nav_stall_timeout_sec", nav_stall_timeout_sec_);
  alignment_watchdog_timeout_sec_ = node_->declare_parameter(
      "alignment_watchdog_timeout_sec", alignment_watchdog_timeout_sec_);
  handover_settle_sec_ = node_->declare_parameter("handover_settle_sec", handover_settle_sec_);
  enable_alignment_ = node_->declare_parameter("enable_alignment", enable_alignment_);
  enable_dynamic_mode_ = node_->declare_parameter("enable_dynamic_mode", enable_dynamic_mode_);
  dynamic_replan_cooldown_sec_ = node_->declare_parameter(
      "dynamic_replan_cooldown_sec", dynamic_replan_cooldown_sec_);
  dynamic_goal_topic_ = node_->declare_parameter("dynamic_goal_topic", dynamic_goal_topic_);
  enable_follow_search_ = node_->declare_parameter("enable_follow_search", enable_follow_search_);
  follow_search_clearance_m_ = node_->declare_parameter(
      "follow_search_clearance_m", follow_search_clearance_m_);
  follow_search_step_deg_ = node_->declare_parameter(
      "follow_search_step_deg", follow_search_step_deg_);
  follow_search_step_pause_sec_ = node_->declare_parameter(
      "follow_search_step_pause_sec", follow_search_step_pause_sec_);
  follow_search_timeout_sec_ = node_->declare_parameter(
      "follow_search_timeout_sec", follow_search_timeout_sec_);
  follow_search_total_limit_deg_ = node_->declare_parameter(
      "follow_search_total_limit_deg", follow_search_total_limit_deg_);
  follow_search_turn_service_ = node_->declare_parameter(
      "follow_search_turn_service", follow_search_turn_service_);
  follow_search_scan_topic_ = node_->declare_parameter(
      "follow_search_scan_topic", follow_search_scan_topic_);

  // Periodic replan install gate parameters
  install_gate_enable_ = node_->declare_parameter("install_gate_enable", install_gate_enable_);
  install_gate_shadow_ = node_->declare_parameter("install_gate_shadow", install_gate_shadow_);
  install_gate_cfg_.blocked_half_width_m = node_->declare_parameter(
      "install_gate_blocked_half_width_m", install_gate_cfg_.blocked_half_width_m);
  install_gate_cfg_.blocked_forward_m = node_->declare_parameter(
      "install_gate_blocked_forward_m", install_gate_cfg_.blocked_forward_m);
  install_gate_cfg_.blocked_enter_sec = node_->declare_parameter(
      "install_gate_blocked_enter_sec", install_gate_cfg_.blocked_enter_sec);
  install_gate_cfg_.blocked_exit_sec = node_->declare_parameter(
      "install_gate_blocked_exit_sec", install_gate_cfg_.blocked_exit_sec);
  install_gate_cfg_.scan_stride = node_->declare_parameter(
      "install_gate_scan_stride", install_gate_cfg_.scan_stride);
  install_gate_cfg_.side_arc_m = node_->declare_parameter(
      "install_gate_side_arc_m", install_gate_cfg_.side_arc_m);
  install_gate_cfg_.side_deadband_m = node_->declare_parameter(
      "install_gate_side_deadband_m", install_gate_cfg_.side_deadband_m);
  install_gate_cfg_.heading_worsen_deg = node_->declare_parameter(
      "install_gate_heading_worsen_deg", install_gate_cfg_.heading_worsen_deg);
  install_gate_cfg_.improve_length_ratio = node_->declare_parameter(
      "install_gate_improve_length_ratio", install_gate_cfg_.improve_length_ratio);
  install_gate_cfg_.improve_kink_deg = node_->declare_parameter(
      "install_gate_improve_kink_deg", install_gate_cfg_.improve_kink_deg);
  install_gate_cfg_.improve_clearance_m = node_->declare_parameter(
      "install_gate_improve_clearance_m", install_gate_cfg_.improve_clearance_m);
  install_gate_cfg_.changed_lateral_m = node_->declare_parameter(
      "install_gate_changed_lateral_m", install_gate_cfg_.changed_lateral_m);
  install_gate_cfg_.changed_length_ratio = node_->declare_parameter(
      "install_gate_changed_length_ratio", install_gate_cfg_.changed_length_ratio);
  install_gate_cfg_.changed_kink_deg = node_->declare_parameter(
      "install_gate_changed_kink_deg", install_gate_cfg_.changed_kink_deg);
  install_gate_cfg_.metric_window_m = node_->declare_parameter(
      "install_gate_metric_window_m", install_gate_cfg_.metric_window_m);
  install_gate_cfg_.alignment_reject_skip_when_blocked = node_->declare_parameter(
      "install_gate_alignment_skip_when_blocked",
      install_gate_cfg_.alignment_reject_skip_when_blocked);
  if (navigation_mode_ != "escape" && navigation_mode_ != "replan_periodic" &&
      navigation_mode_ != "neupan_only") {
    RCLCPP_ERROR(node_->get_logger(), "Invalid navigation_mode=%s; falling back to escape",
                 navigation_mode_.c_str());
    navigation_mode_ = "escape";
  }
  if (navigation_mode_ != "escape") {
    recovery_shadow_enable_ = false;
    recovery_behavior_enable_ = false;
  }
  // Local planner backend and interface contract (action | goal_pose)
  local_planner_ = node_->declare_parameter("local_planner", local_planner_);
  local_planner_mode_ =
      node_->declare_parameter("local_planner_mode", local_planner_mode_);
  if (local_planner_mode_.empty()) {
    local_planner_mode_ =
        (local_planner_ == "sru" || local_planner_ == "goal_pose") ? "goal_pose" : "action";
  }
  if (local_planner_mode_ != "action" && local_planner_mode_ != "goal_pose") {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid local_planner_mode=%s; falling back to action",
                 local_planner_mode_.c_str());
    local_planner_mode_ = "action";
  }
  sru_arrive_threshold_m_ =
      node_->declare_parameter("sru_arrive_threshold_m", sru_arrive_threshold_m_);
  sru_goal_refresh_sec_ =
      node_->declare_parameter("sru_goal_refresh_sec", sru_goal_refresh_sec_);
  sru_goal_refresh_min_delta_m_ = node_->declare_parameter(
      "sru_goal_refresh_min_delta_m", sru_goal_refresh_min_delta_m_);
  sru_cmd_vel_silence_sec_ =
      node_->declare_parameter("sru_cmd_vel_silence_sec", sru_cmd_vel_silence_sec_);
  sru_stall_progress_epsilon_m_ = node_->declare_parameter(
      "sru_stall_progress_epsilon_m", sru_stall_progress_epsilon_m_);
  sru_robot_frame_ =
      node_->declare_parameter("sru_robot_frame", sru_robot_frame_);
  if (local_planner_mode_ == "goal_pose") {
    recovery_shadow_enable_ = false;
    recovery_behavior_enable_ = false;
    install_gate_enable_ = false;
    install_gate_shadow_ = false;
    route_segment_enable_ = false;
    if (enable_dynamic_mode_) {
      RCLCPP_WARN(node_->get_logger(),
                  "[sru] enable_dynamic_mode=true is incompatible with local_planner=sru; "
                  "VLM follow mode disabled");
      enable_dynamic_mode_ = false;
    }
    if (sru_arrive_threshold_m_ < 0.6) {
      RCLCPP_WARN(node_->get_logger(),
                  "[sru] sru_arrive_threshold_m=%.2f < SRU internal 0.6m threshold; "
                  "forcing to 0.7", sru_arrive_threshold_m_);
      sru_arrive_threshold_m_ = 0.7;
    }
  }
  if (periodic_replan_sec_ <= 0.0) periodic_replan_sec_ = 2.0;
  if (route_segment_length_m_ <= 0.0) route_segment_length_m_ = 10.0;
  route_segment_overlap_m_ = std::max(
      0.0, std::min(route_segment_overlap_m_, route_segment_length_m_ * 0.5));

  rclcpp::QoS latched(1);
  latched.transient_local();
  status_pub_ = node_->create_publisher<nav_interfaces::msg::MissionStatus>(
      "/waypoint_manager/status", latched);
  event_pub_ = node_->create_publisher<nav_interfaces::msg::MissionEvent>(
      "/waypoint_manager/mission_feedback", 10);
  operator_command_pub_ = node_->create_publisher<nav_interfaces::msg::OperatorCommand>(
      "/waypoint_manager/operator_command", 10);
  stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>(stop_topic_, 1);
  global_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/waypoint_mission/global_path_vis", latched);
  recovery_command_pub_ = node_->create_publisher<nav_interfaces::msg::RecoveryCommand>(
      "/neupan/recovery_command", 1);
  hot_path_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/neupan/hot_path_enable", latched);

  rclcpp::SubscriptionOptions main_opts;
  main_opts.callback_group = cb_group_main_;
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(1),
      std::bind(&MissionManager::goalCb, this, std::placeholders::_1), main_opts);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(1),
      std::bind(&MissionManager::odomCb, this, std::placeholders::_1), main_opts);
  replan_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/neupan/replan", rclcpp::QoS(1),
      std::bind(&MissionManager::replanCb, this, std::placeholders::_1), main_opts);
  recovery_request_sub_ = node_->create_subscription<nav_interfaces::msg::RecoveryRequest>(
      "/neupan/recovery_request", rclcpp::QoS(1),
      std::bind(&MissionManager::recoveryRequestCb, this, std::placeholders::_1), main_opts);

  if (local_planner_mode_ == "goal_pose") {
    sru_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        sru_goal_topic_, rclcpp::QoS(1));
    sru_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        sru_enable_topic_, latched);
    sru_cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        sru_cmd_vel_topic_, rclcpp::QoS(1),
        std::bind(&MissionManager::sruCmdVelCb, this, std::placeholders::_1), main_opts);
    std_msgs::msg::Bool sru_enable;
    sru_enable.data = false;
    sru_enable_pub_->publish(sru_enable);
    RCLCPP_WARN(node_->get_logger(),
                "[local_planner] %s (goal_pose mode) ENABLED: global planning is "
                "NOT used; waypoints -> %s, enable -> %s, cmd_vel <- %s; arrive_threshold=%.2fm",
                local_planner_.c_str(), sru_goal_topic_.c_str(),
                sru_enable_topic_.c_str(), sru_cmd_vel_topic_.c_str(),
                sru_arrive_threshold_m_);
  }

  install_gate_blocked_.reset(new mission_manager_gate::BlockedHysteresis(
      install_gate_cfg_.blocked_enter_sec, install_gate_cfg_.blocked_exit_sec));
  if (install_gate_shadow_ || install_gate_enable_) {
    install_gate_shadow_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/neupan/replan_gate_shadow", 10);
    if (!install_gate_scan_subscribed_) {
      scan360_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
          follow_search_scan_topic_, rclcpp::QoS(1),
          std::bind(&MissionManager::scan360Cb, this, std::placeholders::_1), main_opts);
      install_gate_scan_subscribed_ = true;
    }
    RCLCPP_INFO(node_->get_logger(),
                "[install_gate] %s (shadow=%d enable=%d): half_width=%.3fm "
                "fwd=%.1fm enter=%.2fs exit=%.2fs window=%.1fm align_skip_blocked=%d "
                "scan=%s",
                install_gate_enable_ ? "ENABLED (blocking)" : "shadow only (no blocking)",
                install_gate_shadow_ ? 1 : 0, install_gate_enable_ ? 1 : 0,
                install_gate_cfg_.blocked_half_width_m,
                install_gate_cfg_.blocked_forward_m,
                install_gate_cfg_.blocked_enter_sec,
                install_gate_cfg_.blocked_exit_sec,
                install_gate_cfg_.metric_window_m,
                install_gate_cfg_.alignment_reject_skip_when_blocked ? 1 : 0,
                follow_search_scan_topic_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[install_gate] disabled (install_gate_enable=false, "
                "install_gate_shadow=false)");
  }

  start_mission_srv_ = node_->create_service<nav_interfaces::srv::StartMission>(
      "/waypoint_manager/start_mission",
      std::bind(&MissionManager::startMissionCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);
  pause_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/waypoint_manager/pause",
      std::bind(&MissionManager::pauseCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);
  resume_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/waypoint_manager/resume",
      std::bind(&MissionManager::resumeCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);
  return_home_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/waypoint_manager/return_home",
      std::bind(&MissionManager::returnHomeCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);
  cancel_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/waypoint_manager/cancel",
      std::bind(&MissionManager::cancelCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);
  reset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/waypoint_manager/reset",
      std::bind(&MissionManager::resetCb, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_main_);

  std_msgs::msg::Bool hot_path;
  hot_path.data = navigation_mode_ == "replan_periodic";
  hot_path_pub_->publish(hot_path);
  RCLCPP_INFO(node_->get_logger(),
              "[navigation_mode] mode=%s periodic=%.3fs hot_path=%s recovery=%s",
              navigation_mode_.c_str(), periodic_replan_sec_,
              hot_path.data ? "true" : "false",
              recovery_behavior_enable_ ? "true" : "false");
  if (enable_dynamic_mode_) {
    set_dynamic_mode_srv_ = node_->create_service<std_srvs::srv::SetBool>(
        "/waypoint_manager/set_dynamic_mode",
        std::bind(&MissionManager::setDynamicModeCb, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_main_);
    dynamic_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        dynamic_goal_topic_, rclcpp::QoS(1),
        std::bind(&MissionManager::dynamicGoalCb, this, std::placeholders::_1),
        main_opts);
    RCLCPP_INFO(node_->get_logger(),
                "[mission_manager] dynamic (follow) mode ENABLED: goal topic=%s, cooldown=%.2fs",
                dynamic_goal_topic_.c_str(), dynamic_replan_cooldown_sec_);
    if (enable_follow_search_) {
      follow_search_srv_ = node_->create_service<nav_interfaces::srv::FollowSearch>(
          "/waypoint_manager/follow_search",
          std::bind(&MissionManager::followSearchCb, this, std::placeholders::_1,
                    std::placeholders::_2),
          rmw_qos_profile_services_default, cb_group_main_);
      scan360_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
          follow_search_scan_topic_, rclcpp::QoS(1),
          std::bind(&MissionManager::scan360Cb, this, std::placeholders::_1), main_opts);
      install_gate_scan_subscribed_ = true;
      turn_client_ = node_->create_client<atomic_move::srv::Turn>(
          follow_search_turn_service_, rmw_qos_profile_services_default,
          cb_group_clients_);
      RCLCPP_INFO(node_->get_logger(),
                  "[mission_manager] follow search ENABLED: step=%.0fdeg pause=%.2fs "
                  "clearance=%.2fm turn=%s scan=%s (limits: total=%.0fdeg timeout=%.0fs)",
                  follow_search_step_deg_, follow_search_step_pause_sec_,
                  follow_search_clearance_m_, follow_search_turn_service_.c_str(),
                  follow_search_scan_topic_.c_str(), follow_search_total_limit_deg_,
                  follow_search_timeout_sec_);
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "[mission_manager] follow search disabled (enable_follow_search=false)");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[mission_manager] dynamic (follow) mode disabled (enable_dynamic_mode=false)");
  }

  // Keep request/reply endpoints discovered for the lifetime of the node.
  plan_client_ = node_->create_client<nav_interfaces::srv::PlanPath>(
      plan_service_name_, rmw_qos_profile_services_default, cb_group_clients_);
  validate_path_client_ = node_->create_client<nav_interfaces::srv::ValidatePath>(
      "/map_planner/validate_path", rclcpp::ServicesQoS(), cb_group_clients_);
  if (local_planner_mode_ == "action" && navigation_mode_ == "replan_periodic") {
    rclcpp::SubscriptionOptions safety_opts;
    safety_opts.callback_group = cb_group_clients_;
    path_safety_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_360", rclcpp::QoS(1),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          path_safety_scan_stamp_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());
          path_safety_scan_received_ns_.store(node_->now().nanoseconds());
        }, safety_opts);
  }
  sequence_plan_client_ = node_->create_client<nav_interfaces::srv::PlanSequence>(
      "/map_planner/plan_sequence", rmw_qos_profile_services_default, cb_group_clients_);
  align_client_ = node_->create_client<atomic_move::srv::AlignTo>(
      "/atomic_move/align", rmw_qos_profile_services_default, cb_group_clients_);
  path_client_ = rclcpp_action::create_client<NavPathAction>(
      node_, nav_path_action_name_, cb_group_clients_);
  status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MissionManager::statusTimerCb, this),
      cb_group_main_);
  process_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MissionManager::process, this),
      cb_group_main_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, node_, true, tf2_ros::DynamicListenerQoS().best_effort());
  publishStatus();
}

MissionManager::~MissionManager() {
  alignment_cancel_requested_.store(true);
  search_cancel_requested_.store(true);
  stopNavigation();
  if (hot_path_pub_) {
    std_msgs::msg::Bool hot_off;
    hot_off.data = false;
    hot_path_pub_->publish(hot_off);
  }
  joinAlignmentThread();
  joinSearchThread();
  if (recovery_shadow_future_.valid()) {
    recovery_shadow_future_.wait();
  }
  if (periodic_replan_future_.valid()) {
    periodic_replan_future_.wait();
  }
  if (path_safety_future_.valid()) path_safety_future_.wait();
  if (initial_plan_future_.valid()) initial_plan_future_.wait();
}

void MissionManager::sendPathGoal(const nav_msgs::msg::Path &path,
                                 const RouteTarget *target) {
  if (!path_client_) return;
  NavPathAction::Goal goal;
  goal.path = path;
  const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
  pending_route_goal_generation_ = target && target->route ? generation : 0;
  pending_route_sent_at_ = node_->now();
  if (target) pending_route_target_ = *target;
  NavPathClient::SendGoalOptions options;
  options.goal_response_callback =
      [this, generation](const rclcpp_action::ClientGoalHandle<NavPathAction>::SharedPtr &handle) {
        if (generation != path_goal_generation_.load()) {
          if (handle && rclcpp::ok()) path_client_->async_cancel_goal(handle);
          return;
        }
        if (!handle) {
          pathRejectedCb(generation);
          return;
        }
        pathActiveCb(generation);
      };
  options.feedback_callback =
      [this, generation](rclcpp_action::ClientGoalHandle<NavPathAction>::SharedPtr,
                         const std::shared_ptr<const NavPathAction::Feedback> feedback) {
        pathFeedbackCb(generation, feedback);
      };
  options.result_callback =
      [this, generation](
          const rclcpp_action::ClientGoalHandle<NavPathAction>::WrappedResult &result) {
        pathDoneCb(generation, result);
      };
  path_client_->async_send_goal(goal, options);
}

bool MissionManager::waitPathDone(double timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (path_done_.load()) return true;
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  return path_done_.load();
}

void MissionManager::process() {
  {
    const rclcpp::Time now = node_->now();
    const double silent = (now - last_odom_rx_at_).seconds();
    if (last_odom_rx_at_.nanoseconds() != 0 && silent > 2.0) {
      if (!ec_perception_offline_) {
        ec_perception_offline_ = true;
        EC_TRACE(ec::DEVICE_OFFLINE, "perception", "odom silent %.1fs", silent);
      }
    } else if (ec_perception_offline_ && last_odom_rx_at_.nanoseconds() != 0) {
      ec_perception_offline_ = false;
      EC_TRACE(ec::DEVICE_OFFLINE, "perception", "odom recovered");
    }
  }
  processRecoveryShadow();
  if (state_ == State::DYNAMIC || state_ == State::DYNAMIC_SEARCHING) {
    processDynamicMode();
    return;
  }
  processInitialRoutePlan();
  commitRouteTarget();
  processPathSafety();
  maybeAdvanceRouteSegment();
  processPeriodicReplan();
  if (path_safety_waiting_ || pending_route_goal_generation_ != 0) return;

  if (!home_pose_valid_ && state_ == State::IDLE) {
    captureHomePoseIfNeeded();
  }
  if (alignment_result_pending_.exchange(false)) {
    joinAlignmentThread();
    if (state_ == State::ALIGNING) {
      if (alignment_result_.load()) {
        beginDwell();
      } else if (!alignment_cancel_requested_.load()) {
        stopNavigation();
        skipCurrentWaypointOrFail("Final alignment failed");
      }
    }
    return;
  }

  if (replan_requested_.exchange(false) && !recovery_episode_active_ &&
      (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME)) {
    RCLCPP_WARN(node_->get_logger(), "NeuPAN requested a new global path for waypoint %d",
                current_index_);
    if (path_client_) {
      path_client_->async_cancel_all_goals();
      waitPathDone(plan_timeout_sec_);
    }
    stop_pub_->publish(std_msgs::msg::Empty());
    path_done_.store(false);
    path_succeeded_.store(false);
    distance_remaining_.store(-1.0);
    publishEvent("REPLANNING", "NeuPAN requested a fresh path");
    if (!startCurrentWaypoint()) {
      stopNavigation();
    }
    return;
  }

  if (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME) {
    if (local_planner_mode_ == "goal_pose") {
      processSruNavigation();
      return;
    }
    if (recovery_episode_active_) {
      const rclcpp::Time now = node_->now();
      if (nav_watchdog_timeout_sec_ > 0.0 &&
          last_path_feedback_at_.nanoseconds() != 0 &&
          (now - last_path_feedback_at_).seconds() > nav_watchdog_timeout_sec_) {
        clearRecoveryCandidate(true);
        stopNavigation();
        skipCurrentWaypointOrFail("NeuPAN feedback watchdog expired during recovery");
      }
      return;
    }
    if (!path_done_.load()) {
      const rclcpp::Time now = node_->now();
      if (nav_watchdog_timeout_sec_ > 0.0 &&
          last_path_feedback_at_.nanoseconds() != 0 &&
          (now - last_path_feedback_at_).seconds() > nav_watchdog_timeout_sec_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "NeuPAN action watchdog expired in state %s after %.3f s",
                     stateName(state_).c_str(),
                     (now - last_path_feedback_at_).seconds());
        EC_TRACE(ec::NEUPAN_FEEDBACK_FROZEN, "watchdog", "state=%s elapsed=%.3fs cause=action_feedback",
                 stateName(state_).c_str(), (now - last_path_feedback_at_).seconds());
        stopNavigation();
        skipCurrentWaypointOrFail("NeuPAN action feedback watchdog expired");
        return;
      }
      if (nav_stall_timeout_sec_ > 0.0 &&
          last_feedback_progress_at_.nanoseconds() != 0 &&
          (now - last_feedback_progress_at_).seconds() > nav_stall_timeout_sec_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[neupan_node] NeuPAN feedback appears frozen in state %s "
                     "(distance_remaining=%.3f unchanged for %.3f s); the NeuPAN "
                     "control loop may have stalled or crashed internally, "
                     "manual restart of neupan_node may be required",
                     stateName(state_).c_str(), last_feedback_distance_,
                     (now - last_feedback_progress_at_).seconds());
        stopNavigation();
        skipCurrentWaypointOrFail("NeuPAN feedback stalled (neupan_node may need a manual restart)");
        return;
      }
      return;
    }
    if (!path_succeeded_.load()) {
      stopNavigation();
      skipCurrentWaypointOrFail("NeuPAN path action failed");
      return;
    }
    if (route_segment_enable_ && navigation_mode_ == "replan_periodic" &&
        !route_segment_final_) {
      return;
    }
    transitionTo(State::ARRIVED);
  }

  if (state_ == State::ALIGNING && alignment_watchdog_timeout_sec_ > 0.0 &&
      alignment_started_at_.nanoseconds() != 0 &&
      (node_->now() - alignment_started_at_).seconds() > alignment_watchdog_timeout_sec_) {
    RCLCPP_ERROR(node_->get_logger(), "Alignment watchdog expired after %.3f s",
                 (node_->now() - alignment_started_at_).seconds());
    stopNavigation();
    skipCurrentWaypointOrFail("Alignment watchdog expired");
    return;
  }

  if (state_ == State::ARRIVED) {
    if (enable_alignment_) {
      stopNavigation();
      alignment_started_at_ = node_->now();
      transitionTo(State::ALIGNING);
      startAlignment();
      return;
    }
    beginDwell();
    return;
  }

  if (state_ == State::DWELLING) {
    dwell_remaining_ = std::max(0.0, dwell_duration_ -
                                      (node_->now() - dwell_start_).seconds());
    if (dwell_remaining_ > 0.0) {
      return;
    }
    if (returning_home_) {
      completeMission();
      return;
    }
    if (current_index_ + 1 < static_cast<int>(waypoints_.size())) {
      ++current_index_;
      startCurrentWaypoint();
      return;
    }
    if (loop_ && !waypoints_.empty()) {
      current_index_ = 0;
      startCurrentWaypoint();
      return;
    }
    completeMission();
  }
}

void MissionManager::startMissionCb(
    const std::shared_ptr<nav_interfaces::srv::StartMission::Request> req,
    std::shared_ptr<nav_interfaces::srv::StartMission::Response> res) {
  const std::string command_id = newCommandId();
  const std::string payload = "{\"waypoint_count\":" +
                              std::to_string(req->waypoints.size()) +
                              ",\"loop\":" + (req->loop ? "true" : "false") + "}";
  publishOperatorCommand(command_id, "START_MISSION", "REQUESTED", payload, false, "");
  const bool temporary = req->temporary;
  const bool normal_state = state_ == State::IDLE || state_ == State::COMPLETED ||
                            state_ == State::CANCELLED || state_ == State::FAILED;
  if ((!normal_state && !temporary) || (temporary && state_ != State::PAUSED)) {
    res->success = false;
    res->message = temporary ? "Temporary mission requires PAUSED state"
                             : "Mission manager is busy in state " + stateName(state_);
    publishOperatorCommand(command_id, "START_MISSION", "REJECTED", payload, false,
                           res->message);
    return;
  }
  if (req->waypoints.empty()) {
    res->success = false;
    res->message = "Waypoint list is empty";
    publishOperatorCommand(command_id, "START_MISSION", "REJECTED", payload, false,
                           res->message);
    return;
  }

  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  if (temporary) {
    paused_waypoints_ = waypoints_;
    paused_current_index_ = current_index_;
    paused_loop_ = loop_;
    paused_resume_state_ = paused_from_;
    paused_mission_id_ = mission_id_;
    temporary_mission_ = true;
  } else {
    paused_waypoints_.clear();
    paused_current_index_ = -1;
    paused_loop_ = false;
    paused_mission_id_.clear();
    temporary_mission_ = false;
  }

  waypoints_ = req->waypoints;
  loop_ = req->loop;
  returning_home_ = false;
  current_index_ = 0;
  completed_waypoints_ = 0;
  mission_id_ = newMissionId();
  last_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (!temporary) {
    captureHomePoseIfNeeded();
  }

  res->success = startCurrentWaypoint();
  res->mission_id = mission_id_;
  res->message = res->success ? (temporary ? "Temporary mission started" : "Mission started")
                              : "Unable to start first waypoint";
  if (res->success && !temporary) {
    publishGlobalPathVisualization();
  }
  publishOperatorCommand(command_id, "START_MISSION",
                         res->success ? "ACCEPTED" : "REJECTED", payload,
                         res->success, res->message);
}

void MissionManager::pauseCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "PAUSE", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res->success = false;
    res->message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "PAUSE", "REJECTED", "{}", false, res->message);
    return;
  }
  if (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME &&
      state_ != State::DWELLING) {
    res->success = false;
    res->message = "Mission is not active";
    publishOperatorCommand(command_id, "PAUSE", "REJECTED", "{}", false, res->message);
    return;
  }
  clearRecoveryCandidate(true);
  paused_from_ = state_;
  stopNavigation();
  transitionTo(State::PAUSED, "Paused by request");
  res->success = true;
  res->message = "Mission paused";
  publishOperatorCommand(command_id, "PAUSE", "ACCEPTED", "{}", true, res->message);
}

void MissionManager::resumeCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "RESUME", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res->success = false;
    res->message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "RESUME", "REJECTED", "{}", false, res->message);
    return;
  }
  if (state_ != State::PAUSED) {
    res->success = false;
    res->message = "Mission is not paused";
    publishOperatorCommand(command_id, "RESUME", "REJECTED", "{}", false, res->message);
    return;
  }
  if (paused_from_ == State::DWELLING) {
    dwell_start_ = node_->now();
    transitionTo(State::DWELLING, "Resumed dwell");
    res->success = true;
    res->message = "Dwell resumed";
    publishOperatorCommand(command_id, "RESUME", "ACCEPTED", "{}", true, res->message);
    return;
  }
  res->success = startCurrentWaypoint();
  res->message = res->success ? "Mission resumed" : "Unable to resume waypoint";
  publishOperatorCommand(command_id, "RESUME",
                         res->success ? "ACCEPTED" : "REJECTED", "{}",
                         res->success, res->message);
}

void MissionManager::returnHomeCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "RETURN_HOME", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res->success = false;
    res->message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "RETURN_HOME", "REJECTED", "{}", false,
                           res->message);
    return;
  }
  if (!home_pose_valid_) {
    res->success = false;
    res->message = "Home pose is not available";
    publishOperatorCommand(command_id, "RETURN_HOME", "REJECTED", "{}", false,
                           res->message);
    return;
  }
  stopNavigation();
  nav_interfaces::msg::Waypoint home;
  home.x = home_pose_.position.x;
  home.y = home_pose_.position.y;
  home.yaw = tf2::getYaw(home_pose_.orientation);
  home.name = "home";
  home.dwell_sec = 0.0;
  waypoints_.assign(1, home);
  current_index_ = 0;
  returning_home_ = true;
  if (mission_id_.empty()) {
    mission_id_ = newMissionId();
  }
  res->success = startCurrentWaypoint();
  res->message = res->success ? "Returning home" : "Unable to plan home path";
  publishOperatorCommand(command_id, "RETURN_HOME",
                         res->success ? "ACCEPTED" : "REJECTED", "{}",
                         res->success, res->message);
}

void MissionManager::cancelCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "CANCEL", "REQUESTED", "{}", false, "");
  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  if (dynamic_mode_) {
    exitDynamicMode("Dynamic mode exited by cancel");
    transitionTo(State::CANCELLED, "Mission cancelled (dynamic mode exited)");
    res->success = true;
    res->message = "Dynamic mode exited and cancelled";
    publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res->message);
    return;
  }
  if (temporary_mission_) {
    restorePausedMission("Temporary mission cancelled; main mission remains paused");
    res->success = true;
    res->message = "Temporary mission cancelled; main mission remains paused";
    publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res->message);
    return;
  }
  stopNavigation();
  waypoints_.clear();
  current_index_ = -1;
  loop_ = false;
  returning_home_ = false;
  mission_id_.clear();
  last_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  transitionTo(State::CANCELLED, "Mission cancelled");
  res->success = true;
  res->message = "Mission cancelled";
  publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res->message);
}

void MissionManager::resetCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "RESET", "REQUESTED", "{}", false, "");
  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  if (dynamic_mode_) {
    exitDynamicMode("Dynamic mode exited by reset");
  }
  stopNavigation();
  waypoints_.clear();
  current_index_ = -1;
  loop_ = false;
  temporary_mission_ = false;
  paused_waypoints_.clear();
  paused_current_index_ = -1;
  paused_loop_ = false;
  paused_mission_id_.clear();
  returning_home_ = false;
  mission_id_.clear();
  last_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  dwell_duration_ = 0.0;
  dwell_remaining_ = 0.0;
  alignment_started_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  transitionTo(State::IDLE, "Mission manager reset");
  res->success = true;
  res->message = "Mission manager reset";
  publishOperatorCommand(command_id, "RESET", "ACCEPTED", "{}", true, res->message);
}

void MissionManager::goalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  if (state_ != State::IDLE && state_ != State::COMPLETED &&
      state_ != State::CANCELLED && state_ != State::FAILED) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring single goal while mission is active");
    return;
  }
  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  geometry_msgs::msg::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_frame_;
  }
  if (goal.header.frame_id != map_frame_) {
    try {
      geometry_msgs::msg::PoseStamped transformed;
      tf_buffer_->transform(goal, transformed, map_frame_, tfTimeout(0.2));
      goal = transformed;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Single goal transform failed: %s", ex.what());
      return;
    }
  }
  nav_interfaces::msg::Waypoint single;
  single.x = goal.pose.position.x;
  single.y = goal.pose.position.y;
  single.yaw = tf2::getYaw(goal.pose.orientation);
  single.name = "single_goal";
  single.dwell_sec = 0.0;
  waypoints_.assign(1, single);
  loop_ = false;
  returning_home_ = false;
  current_index_ = 0;
  completed_waypoints_ = 0;
  mission_id_ = newMissionId();
  startCurrentWaypoint();
}

// Dynamic target following mode and stepped rotation search

void MissionManager::setDynamicModeCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  const std::string command_id = newCommandId();
  const std::string command = req->data ? "ENTER_DYNAMIC_MODE" : "EXIT_DYNAMIC_MODE";
  publishOperatorCommand(command_id, command, "REQUESTED", "{}", false, "");

  if (req->data) {
    if (dynamic_mode_) {
      res->success = true;
      res->message = "Already in dynamic mode";
      publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res->message);
      return;
    }
    stopNavigation();
    waypoints_.clear();
    current_index_ = -1;
    loop_ = false;
    returning_home_ = false;
    mission_id_.clear();
    temporary_mission_ = false;
    paused_waypoints_.clear();
    paused_current_index_ = -1;
    paused_loop_ = false;
    paused_mission_id_.clear();
    dynamic_mode_ = true;
    dynamic_goal_pending_ = false;
    dynamic_goal_valid_ = false;
    dynamic_nav_active_ = false;
    dynamic_plan_failures_ = 0;
    last_dynamic_plan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std_msgs::msg::Bool hot_on;
    hot_on.data = true;
    hot_path_pub_->publish(hot_on);
    transitionTo(State::DYNAMIC,
                 "Dynamic (follow) mode entered; previous activity discarded");
    res->success = true;
    res->message = "Dynamic mode entered; publish goals to " + dynamic_goal_topic_;
    publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res->message);
    return;
  }

  if (!dynamic_mode_) {
    res->success = true;
    res->message = "Not in dynamic mode";
    publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res->message);
    return;
  }
  exitDynamicMode("Dynamic mode exited by request; holding position in IDLE");
  res->success = true;
  res->message = "Dynamic mode exited; state=IDLE, holding position";
  publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res->message);
}

void MissionManager::exitDynamicMode(const std::string &message) {
  dynamic_mode_ = false;
  dynamic_goal_pending_ = false;
  dynamic_goal_valid_ = false;
  dynamic_nav_active_ = false;
  cancelFollowSearch("exit dynamic mode");
  stopNavigation();
  joinSearchThread();
  if (hot_path_pub_) {
    std_msgs::msg::Bool hot_off;
    hot_off.data = false;
    hot_path_pub_->publish(hot_off);
  }
  transitionTo(State::IDLE, message);
}

void MissionManager::dynamicGoalCb(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  if (!dynamic_mode_ ||
      (state_ != State::DYNAMIC && state_ != State::DYNAMIC_SEARCHING)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring dynamic goal: not in dynamic mode "
                         "(call /waypoint_manager/set_dynamic_mode true first)");
    return;
  }
  geometry_msgs::msg::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_frame_;
  }
  if (goal.header.frame_id != map_frame_) {
    try {
      geometry_msgs::msg::PoseStamped transformed;
      tf_buffer_->transform(goal, transformed, map_frame_, tfTimeout(0.2));
      goal = transformed;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Dynamic goal transform failed (%s); dropping this point",
                           ex.what());
      return;
    }
  }
  if (search_running_.load() || state_ == State::DYNAMIC_SEARCHING) {
    RCLCPP_INFO(node_->get_logger(),
                "[follow_search] target reacquired (new dynamic goal) -> stopping search");
    cancelFollowSearch("dynamic_goal_received");
  }
  dynamic_goal_ = goal;
  dynamic_goal_valid_ = true;
  dynamic_goal_pending_ = true;
}

void MissionManager::processDynamicMode() {
  if (search_result_pending_.exchange(false)) {
    joinSearchThread();
    if (state_ == State::DYNAMIC_SEARCHING) {
      const SearchStop code = static_cast<SearchStop>(search_stop_code_.load());
      const std::string text = searchStopText(code);
      if (code == SearchStop::CANCELLED || code == SearchStop::SUPERSEDED) {
        transitionTo(State::DYNAMIC, "Follow search stopped (" + text + ")");
      } else {
        RCLCPP_WARN(node_->get_logger(), "[follow_search] stopped: %s", text.c_str());
        transitionTo(State::DYNAMIC, "Follow search stopped: " + text);
        publishEvent("FOLLOW_SEARCH_STOPPED", text);
      }
    }
    return;
  }
  if (state_ == State::DYNAMIC_SEARCHING) {
    return;
  }

  if (dynamic_nav_active_ && path_done_.load() && !path_goal_active_.load()) {
    dynamic_nav_active_ = false;
    if (path_succeeded_.load()) {
      publishEvent("DYNAMIC_GOAL_REACHED", "Holding position at last dynamic goal");
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[dynamic] navPath action ended without success; holding "
                           "position and waiting for the next dynamic goal");
    }
  }

  if (!dynamic_goal_pending_) {
    return;
  }
  const rclcpp::Time now = node_->now();
  if (last_dynamic_plan_at_.nanoseconds() != 0 &&
      (now - last_dynamic_plan_at_).seconds() < dynamic_replan_cooldown_sec_) {
    return;
  }
  dynamic_goal_pending_ = false;
  last_dynamic_plan_at_ = now;
  startDynamicGoal();
}

bool MissionManager::startDynamicGoal() {
  auto request = std::make_shared<nav_interfaces::srv::PlanPath::Request>();
  request->goal = dynamic_goal_;
  auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanPath>,
                           std::shared_ptr<nav_interfaces::srv::PlanPath::Request>>(
      plan_client_, request, plan_timeout_sec_);
  if (!response) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[dynamic] plan service unavailable; holding current state");
    return false;
  }
  if (!response->success || response->path.poses.empty()) {
    ++dynamic_plan_failures_;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[dynamic] plan failed (%s); holding (consecutive=%d)",
                         response->message.c_str(), dynamic_plan_failures_);
    return false;
  }
  dynamic_plan_failures_ = 0;

  if (!path_client_->wait_for_action_server(tf2::durationFromSec(plan_timeout_sec_))) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[dynamic] NeuPAN action server unavailable; holding");
    return false;
  }
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = node_->now();
  last_path_feedback_at_ = path_started_at_;
  last_feedback_progress_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_feedback_distance_ = -1.0;
  sendPathGoal(response->path);
  dynamic_nav_active_ = true;
  return true;
}

std::string MissionManager::searchStopText(SearchStop code) {
  switch (code) {
    case SearchStop::CANCELLED: return "cancelled";
    case SearchStop::SUPERSEDED: return "superseded";
    case SearchStop::TIMEOUT: return "timeout";
    case SearchStop::LIMIT: return "total_limit_reached";
    case SearchStop::BLOCKED: return "blocked_clearance";
    case SearchStop::NO_DATA: return "no_scan360_data";
    case SearchStop::TURN_FAILED: return "turn_failed";
  }
  return "unknown";
}

void MissionManager::scan360Cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
  if (!msg || msg->ranges.empty()) return;
  double min_range = std::numeric_limits<double>::infinity();
  for (const float r : msg->ranges) {
    if (!std::isfinite(r)) continue;
    if (r <= msg->range_min || r >= msg->range_max) continue;
    if (r < min_range) min_range = r;
  }
  if (std::isfinite(min_range)) {
    all_round_clearance_.store(min_range);
  } else {
    all_round_clearance_.store(msg->range_max);
  }
  scan360_received_.store(true);

  if (install_gate_shadow_ || install_gate_enable_) {
    install_gate_scan_map_pts_.clear();
    install_gate_scan_ready_ = false;
    geometry_msgs::msg::TransformStamped tf;
    bool tf_ok = false;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
                                       rclcpp::Time(msg->header.stamp), tfTimeout(0.05));
      tf_ok = true;
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
                                         tf2::TimePointZero, tfTimeout(0.05));
        tf_ok = true;
      } catch (const tf2::TransformException &) {
        tf_ok = false;
      }
    }
    if (tf_ok) {
      const double c = std::cos(tf2::getYaw(tf.transform.rotation));
      const double s = std::sin(tf2::getYaw(tf.transform.rotation));
      const double tx = tf.transform.translation.x;
      const double ty = tf.transform.translation.y;
      const int stride = std::max(1, install_gate_cfg_.scan_stride);
      install_gate_scan_map_pts_.reserve(msg->ranges.size() / stride + 1);
      for (std::size_t i = 0; i < msg->ranges.size();
           i += static_cast<std::size_t>(stride)) {
        const float r = msg->ranges[i];
        if (!std::isfinite(r) || r <= msg->range_min || r >= msg->range_max) {
          continue;
        }
        const double a =
            msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const double px = r * std::cos(a);
        const double py = r * std::sin(a);
        mission_manager_gate::Pt p;
        p.x = tx + c * px - s * py;
        p.y = ty + s * px + c * py;
        install_gate_scan_map_pts_.push_back(p);
      }
      install_gate_scan_stamp_ = rclcpp::Time(msg->header.stamp);
      install_gate_scan_ready_ = !install_gate_scan_map_pts_.empty();
    }
    updateInstallGateBlocked();
  }
}

void MissionManager::resetInstallGateState(const std::string &why) {
  if (!install_gate_blocked_) return;
  install_gate_blocked_->reset();
  install_gate_blocked_raw_.store(false);
  install_gate_blocked_gated_.store(false);
  install_gate_min_lateral_m_.store(-1.0);
  install_gate_blocked_dwell_sec_ = 0.0;
  if (install_gate_shadow_ || install_gate_enable_) {
    RCLCPP_INFO(node_->get_logger(), "[install_gate] state reset (%s)", why.c_str());
  }
}

std::vector<mission_manager_gate::Pt> MissionManager::pathToPoints(
    const nav_msgs::msg::Path &path) const {
  std::vector<mission_manager_gate::Pt> out;
  out.reserve(path.poses.size());
  for (const auto &p : path.poses) {
    mission_manager_gate::Pt pt;
    pt.x = p.pose.position.x;
    pt.y = p.pose.position.y;
    out.push_back(pt);
  }
  return out;
}

void MissionManager::updateInstallGateBlocked() {
  if (!(install_gate_shadow_ || install_gate_enable_)) return;
  bool raw = false;
  double min_lateral = -1.0;
  if (install_gate_scan_ready_ && active_path_.poses.size() >= 2) {
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      const mission_manager_gate::Pt robot{pose.pose.position.x,
                                           pose.pose.position.y};
      const std::vector<mission_manager_gate::Pt> active =
          pathToPoints(active_path_);
      const std::vector<mission_manager_gate::Pt> trimmed =
          mission_manager_gate::trimFromNearest(
              active, robot, install_gate_cfg_.blocked_forward_m);
      const mission_manager_gate::CorridorHit hit =
          mission_manager_gate::corridorBlocked(
              trimmed, install_gate_scan_map_pts_,
              install_gate_cfg_.blocked_half_width_m);
      raw = hit.blocked;
      min_lateral = hit.min_lateral_m;
    }
  }
  const double now_sec = std::chrono::duration<double>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
  const bool gated = install_gate_blocked_->update(now_sec, raw);
  install_gate_blocked_raw_.store(raw);
  install_gate_blocked_gated_.store(gated);
  install_gate_min_lateral_m_.store(min_lateral);
  install_gate_blocked_dwell_sec_ = install_gate_blocked_->dwellSec(now_sec);
}

double MissionManager::allRoundClearance() const {
  if (!scan360_received_.load()) return -1.0;
  return all_round_clearance_.load();
}

void MissionManager::followSearchCb(
    const std::shared_ptr<nav_interfaces::srv::FollowSearch::Request> req,
    std::shared_ptr<nav_interfaces::srv::FollowSearch::Response> res) {
  res->accepted = false;
  if (!enable_follow_search_) {
    res->message = "follow search disabled (enable_follow_search=false)";
    return;
  }
  if (!dynamic_mode_ ||
      (state_ != State::DYNAMIC && state_ != State::DYNAMIC_SEARCHING)) {
    res->message = "not in dynamic mode";
    return;
  }
  if (req->direction > -0.5 && req->direction < 0.5) {
    res->message = "direction must be +1 (left/ccw) or -1 (right/cw)";
    return;
  }
  if (!scan360_received_.load()) {
    res->message = "no " + follow_search_scan_topic_ +
                   " data yet; refusing to rotate blind";
    RCLCPP_WARN(node_->get_logger(), "[follow_search] rejected: %s", res->message.c_str());
    return;
  }

  const double direction = req->direction > 0.0 ? 1.0 : -1.0;
  const double step_deg =
      req->step_deg > 0.0 ? req->step_deg : follow_search_step_deg_;
  const double pause_sec = req->step_pause_sec > 0.0 ? req->step_pause_sec
                                                     : follow_search_step_pause_sec_;
  const double total_limit = req->total_limit_deg > 0.0 ? req->total_limit_deg
                                                        : follow_search_total_limit_deg_;
  const double timeout_sec = req->timeout_sec > 0.0 ? req->timeout_sec
                                                    : follow_search_timeout_sec_;
  const double clearance_m = req->min_clearance_m > 0.0 ? req->min_clearance_m
                                                        : follow_search_clearance_m_;

  stopNavigation();
  dynamic_goal_pending_ = false;
  transitionTo(State::DYNAMIC_SEARCHING, "Follow search started");
  startFollowSearch(direction, step_deg, pause_sec, total_limit, timeout_sec,
                    clearance_m);
  RCLCPP_INFO(node_->get_logger(),
              "[follow_search] accepted: dir=%+.0f step=%.0fdeg pause=%.2fs "
              "total_limit=%.0fdeg timeout=%.0fs clearance=%.2fm bearing=%.3frad",
              direction, step_deg, pause_sec, total_limit, timeout_sec, clearance_m,
              req->last_bearing_rad);
  res->accepted = true;
  res->message = "search started";
}

void MissionManager::startFollowSearch(double direction, double step_deg,
                                       double pause_sec, double total_limit_deg,
                                       double timeout_sec, double clearance_m) {
  joinSearchThread();
  search_cancel_requested_.store(false);
  search_result_pending_.store(false);
  search_running_.store(true);
  const std::uint64_t generation = search_generation_.fetch_add(1) + 1;
  search_thread_ = std::thread([this, direction, step_deg, pause_sec,
                                total_limit_deg, timeout_sec, clearance_m,
                                generation]() {
    followSearchLoop(direction, step_deg, pause_sec, total_limit_deg, timeout_sec,
                     clearance_m, generation);
    if (generation != search_generation_.load()) {
      return;
    }
    search_running_.store(false);
    search_result_pending_.store(true);
  });
}

void MissionManager::joinSearchThread() {
  if (search_thread_.joinable()) {
    search_thread_.join();
  }
  search_running_.store(false);
}

void MissionManager::cancelFollowSearch(const std::string &reason) {
  if (!search_running_.load() && state_ != State::DYNAMIC_SEARCHING) {
    return;
  }
  search_cancel_requested_.store(true);
  stop_pub_->publish(std_msgs::msg::Empty());
  RCLCPP_INFO(node_->get_logger(), "[follow_search] cancel requested: %s", reason.c_str());
}

void MissionManager::followSearchLoop(double direction, double step_deg,
                                      double pause_sec, double total_limit_deg,
                                      double timeout_sec, double clearance_m,
                                      std::uint64_t generation) {
  const std::string tag = "[follow_search]";
  SearchStop stop_code = SearchStop::CANCELLED;

  // Let atomic_move consume the /neupan/stop emitted by stopNavigation() before calling /atomic_move/turn.
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(handover_settle_sec_)));
  if (generation != search_generation_.load() || search_cancel_requested_.load()) {
    search_stop_code_.store(static_cast<int>(SearchStop::CANCELLED));
    return;
  }

  double accumulated_deg = 0.0;
  int segments = 0;
  const auto started = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (generation != search_generation_.load()) {
      stop_code = SearchStop::SUPERSEDED;
      break;
    }
    if (search_cancel_requested_.load()) {
      stop_code = SearchStop::CANCELLED;
      break;
    }
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    if (timeout_sec > 0.0 && elapsed > timeout_sec) {
      stop_code = SearchStop::TIMEOUT;
      break;
    }
    if (total_limit_deg > 0.0 && accumulated_deg >= total_limit_deg) {
      stop_code = SearchStop::LIMIT;
      break;
    }

    const double clearance = allRoundClearance();
    if (clearance < 0.0) {
      stop_code = SearchStop::NO_DATA;
      break;
    }
    if (clearance < clearance_m) {
      RCLCPP_WARN(node_->get_logger(),
                  "%s blocked: clearance=%.2fm < %.2fm; stopping search", tag.c_str(),
                  clearance, clearance_m);
      stop_code = SearchStop::BLOCKED;
      break;
    }

    auto turn_request = std::make_shared<atomic_move::srv::Turn::Request>();
    turn_request->angle = static_cast<float>(step_deg * direction);
    auto turn_response = callSync<rclcpp::Client<atomic_move::srv::Turn>,
                                  std::shared_ptr<atomic_move::srv::Turn::Request>>(
        turn_client_, turn_request, 1.0);
    if (!turn_response) {
      RCLCPP_WARN(node_->get_logger(), "%s turn service %s unavailable", tag.c_str(),
                  follow_search_turn_service_.c_str());
      stop_code = SearchStop::TURN_FAILED;
      break;
    }
    if (!turn_response->success) {
      if (turn_response->message == "TURN_CANCELLED") {
        stop_code = SearchStop::CANCELLED;
        break;
      }
      RCLCPP_WARN(node_->get_logger(), "%s turn failed: %s", tag.c_str(),
                  turn_response->message.c_str());
      stop_code = SearchStop::TURN_FAILED;
      break;
    }
    accumulated_deg += step_deg;
    ++segments;
    RCLCPP_INFO(node_->get_logger(),
                "%s segment %d done: this=%.0fdeg total=%.0fdeg clearance=%.2fm",
                tag.c_str(), segments, step_deg, accumulated_deg, clearance);

    const auto pause_end = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(pause_sec));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < pause_end) {
      if (generation != search_generation_.load() ||
          search_cancel_requested_.load()) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
  }

  search_stop_code_.store(static_cast<int>(stop_code));
  const double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - started)
                             .count();
  RCLCPP_INFO(node_->get_logger(),
              "%s loop finished: reason=%s segments=%d total=%.0fdeg elapsed=%.2fs",
              tag.c_str(), searchStopText(stop_code).c_str(), segments, accumulated_deg,
              elapsed);
}

void MissionManager::clearRouteSegment() {
  global_route_.poses.clear();
  route_progress_index_ = 0;
  route_segment_anchor_index_ = 0;
  ++route_segment_generation_;
  route_segment_final_ = true;
  route_advance_requested_ = false;
  pending_route_goal_generation_ = 0;
}

std::vector<MissionManager::RouteTarget> MissionManager::routeTargets(
    const geometry_msgs::msg::PoseStamped &pose, bool advance) const {
  std::vector<RouteTarget> targets;
  if (!route_segment_enable_ || navigation_mode_ != "replan_periodic" ||
      global_route_.poses.size() < 2 ||
      global_route_.header.frame_id != map_frame_) {
    return targets;
  }
  const std::size_t last = global_route_.poses.size() - 1;
  advance = advance || (route_segment_anchor_index_ == 0 && route_segment_final_);
  const std::size_t begin = std::min(route_progress_index_, last);
  std::size_t search_end = begin;
  double search_arc = 0.0;
  while (search_end < last && search_arc < route_segment_length_m_ * 2.0) {
    const auto &a = global_route_.poses[search_end].pose.position;
    const auto &b = global_route_.poses[search_end + 1].pose.position;
    search_arc += std::hypot(b.x - a.x, b.y - a.y);
    ++search_end;
  }
  std::size_t nearest = begin;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = begin; i <= search_end; ++i) {
    const auto &point = global_route_.poses[i].pose.position;
    const double distance = std::hypot(
        point.x - pose.pose.position.x, point.y - pose.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  std::size_t anchor = advance ? nearest : route_segment_anchor_index_;
  double segment_arc = 0.0;
  while (advance && anchor < last && segment_arc < route_segment_length_m_) {
    const auto &a = global_route_.poses[anchor].pose.position;
    const auto &b = global_route_.poses[anchor + 1].pose.position;
    segment_arc += std::hypot(b.x - a.x, b.y - a.y);
    ++anchor;
  }
  if (anchor <= route_segment_anchor_index_ && !route_segment_final_) {
    if (advance) return targets;
  }
  targets.push_back({global_route_.poses[anchor], nearest, anchor, true, anchor == last});
  // Only intermediate targets may move, and only forward by at most half a
  // segment. Each alternative still has to pass real A* and final validation.
  const double limit = route_segment_length_m_ * 0.5;
  double extra = 0.0;
  double next_sample = 1.0;
  while (anchor < last && extra < limit) {
    const auto& a = global_route_.poses[anchor].pose.position;
    const auto& b = global_route_.poses[anchor + 1].pose.position;
    const double step = std::hypot(b.x - a.x, b.y - a.y);
    if (extra + step > limit + 1e-6) break;
    extra += step;
    ++anchor;
    if (extra >= next_sample || anchor == last) {
      targets.push_back({global_route_.poses[anchor], nearest, anchor, true, anchor == last});
      next_sample = extra + 1.0;
    }
  }
  return targets;
}

void MissionManager::commitRouteTarget() {
  if (pending_route_goal_generation_ == 0) return;
  if (pending_route_goal_generation_ != path_goal_generation_.load()) {
    pending_route_goal_generation_ = 0;
    return;
  }
  if (accepted_path_generation_.load() != pending_route_goal_generation_) {
    const bool rejected = path_done_.load() && !path_succeeded_.load();
    if (rejected ||
        (node_->now() - pending_route_sent_at_).seconds() > plan_timeout_sec_) {
      holdForPathSafety(rejected ? "PATH_ACTION_REJECTED" : "PATH_ACTION_ACCEPT_TIMEOUT");
    }
    return;
  }
  const bool changed = route_segment_anchor_index_ != pending_route_target_.anchor;
  route_progress_index_ = pending_route_target_.progress;
  route_segment_anchor_index_ = pending_route_target_.anchor;
  route_segment_final_ = pending_route_target_.final;
  ++route_segment_generation_;
  pending_route_goal_generation_ = 0;
  route_advance_requested_ = false;
  if (changed) {
    resetInstallGateState("route_segment");
    RCLCPP_INFO(node_->get_logger(),
                "[route_segment] accepted planned anchor=%zu/%zu poses=%zu final=%s",
                route_segment_anchor_index_, global_route_.poses.size() - 1,
                active_path_.poses.size(), route_segment_final_ ? "true" : "false");
  }
}

void MissionManager::maybeAdvanceRouteSegment() {
  if (!route_segment_enable_ || navigation_mode_ != "replan_periodic" ||
      route_segment_final_ || path_safety_waiting_ ||
      pending_route_goal_generation_ != 0 || route_advance_requested_ ||
      (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME)) {
    return;
  }
  const double remaining = distance_remaining_.load();
  if (!(remaining >= 0.0 && remaining <= route_segment_overlap_m_) &&
      !(path_done_.load() && path_succeeded_.load())) {
    return;
  }
  route_advance_requested_ = true;
  last_periodic_replan_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void MissionManager::processPeriodicReplan() {
  if (local_planner_mode_ == "goal_pose") return;
  if (navigation_mode_ != "replan_periodic") return;
  if (periodic_replan_running_ && periodic_replan_future_.valid() &&
      periodic_replan_future_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
    PeriodicReplanResult result = periodic_replan_future_.get();
    periodic_replan_running_ = false;
    if (result.mission_id == mission_id_ &&
        result.waypoint_index == current_index_ &&
        result.segment_generation == route_segment_generation_ &&
        result.path_generation == path_goal_generation_.load() &&
        !(route_advance_requested_ && result.target.route &&
          result.target.anchor <= route_segment_anchor_index_) &&
        (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME)) {
      geometry_msgs::msg::PoseStamped pose;
      nav_msgs::msg::Path prepared;
      if (result.planned && getRobotPose(pose)) {
        prepared = preparePeriodicPath(result.path, pose);
        if (!validateForInstall(prepared, result.flags)) {
          prepared.poses.clear();
        }
      }
      if (result.flags.find("DYNAMIC_MAP") != std::string::npos ||
          result.flags == "SCAN_STALE" ||
          result.flags == "PATH_VALIDATION_UNAVAILABLE") {
        holdForPathSafety(result.flags);
      }

      // Evaluate periodic replan install gate
      mission_manager_gate::GateDecision gate;
      gate.install = true;
      gate.reason = "GATE_DISABLED";
      gate.detail = "";
      const bool gate_active = install_gate_shadow_ || install_gate_enable_;
      if (gate_active && prepared.poses.size() >= 2) {
        const std::vector<mission_manager_gate::Pt> active_pts =
            pathToPoints(active_path_);
        const std::vector<mission_manager_gate::Pt> cand_pts =
            pathToPoints(prepared);
        const double yaw = tf2::getYaw(pose.pose.orientation);
        const mission_manager_gate::Pt robot{pose.pose.position.x,
                                             pose.pose.position.y};
        // Trim both active and candidate paths from the robot forward by window_m
        // so metrics compare equal forward horizons regardless of distance traveled.
        const double window_m = install_gate_cfg_.metric_window_m;
        std::vector<mission_manager_gate::Pt> active_metric_pts =
            mission_manager_gate::trimFromNearest(active_pts, robot, window_m);
        std::vector<mission_manager_gate::Pt> cand_metric_pts =
            mission_manager_gate::trimFromNearest(cand_pts, robot, window_m);
        if (active_metric_pts.size() < 2) active_metric_pts = active_pts;
        if (cand_metric_pts.size() < 2) cand_metric_pts = cand_pts;
        mission_manager_gate::PathMetrics act_m =
            mission_manager_gate::computePathMetrics(active_metric_pts, robot,
                                                     yaw, install_gate_cfg_);
        mission_manager_gate::PathMetrics cand_m =
            mission_manager_gate::computePathMetrics(cand_metric_pts, robot,
                                                      yaw, install_gate_cfg_);
        if (install_gate_scan_ready_ && !active_metric_pts.empty() &&
            !cand_metric_pts.empty()) {
          act_m.min_scan_lateral_m =
              mission_manager_gate::corridorBlocked(
                  active_metric_pts, install_gate_scan_map_pts_,
                  install_gate_cfg_.blocked_half_width_m)
                  .min_lateral_m;
          cand_m.min_scan_lateral_m =
              mission_manager_gate::corridorBlocked(
                  cand_metric_pts, install_gate_scan_map_pts_,
                  install_gate_cfg_.blocked_half_width_m)
                  .min_lateral_m;
        }
        mission_manager_gate::GateInput gin;
        gin.active = act_m;
        gin.candidate = cand_m;
        gin.candidate_requires_alignment =
            result.flags.find("REQUIRES_ALIGNMENT") != std::string::npos;
        gin.blocked_gated = install_gate_blocked_gated_.load();
        gin.blocked_raw = install_gate_blocked_raw_.load();
        gin.blocked_dwell_sec = install_gate_blocked_dwell_sec_;
        gin.deviation_m =
            mission_manager_gate::maxDeviation(cand_metric_pts,
                                                active_metric_pts);
        gate = mission_manager_gate::evaluateInstallGate(gin,
                                                         install_gate_cfg_);

        std::ostringstream shadow;
        shadow << std::fixed << std::setprecision(3)
               << "{\"waypoint\":" << current_index_ << ",\"install\":"
               << (gate.install ? "true" : "false") << ",\"reason\":\""
               << gate.reason << "\",\"detail\":\"" << gate.detail
               << "\",\"blocked_raw\":" << (gin.blocked_raw ? "true" : "false")
               << ",\"blocked_gated\":"
               << (gin.blocked_gated ? "true" : "false")
               << ",\"dwell\":" << gin.blocked_dwell_sec
               << ",\"min_lat\":" << install_gate_min_lateral_m_.load()
               << ",\"side_a\":" << act_m.side << ",\"side_c\":"
               << cand_m.side << ",\"heading_a\":"
               << act_m.start_heading_deg << ",\"heading_c\":"
               << cand_m.start_heading_deg << ",\"len_a\":" << act_m.length_m
               << ",\"len_c\":" << cand_m.length_m << ",\"kink_a\":"
               << act_m.max_kink_deg << ",\"kink_c\":" << cand_m.max_kink_deg
               << ",\"dev\":" << gin.deviation_m << ",\"flags\":\""
               << result.flags << "\"}";
        if (install_gate_shadow_pub_) {
          std_msgs::msg::String msg;
          msg.data = shadow.str();
          install_gate_shadow_pub_->publish(msg);
        }
        if (!gate.install) {
          RCLCPP_WARN(node_->get_logger(), "[install_gate] %s (%s) waypoint=%d shadow=%d",
                      gate.reason.c_str(), gate.detail.c_str(), current_index_,
                      install_gate_shadow_ ? 1 : 0);
        } else {
          RCLCPP_INFO(node_->get_logger(), "[install_gate] %s (%s) waypoint=%d",
                      gate.reason.c_str(), gate.detail.c_str(), current_index_);
        }
      }

      const bool gate_block_install = install_gate_enable_ && !gate.install;
      if (!gate_block_install && prepared.poses.size() >= 2 && path_client_ &&
          path_client_->action_server_is_ready()) {
        active_path_ = prepared;
        path_safety_waiting_ = false;
        path_done_.store(false);
        path_succeeded_.store(false);
        distance_remaining_.store(-1.0);
        last_feedback_distance_ = -1.0;
        last_path_feedback_at_ = node_->now();
        sendPathGoal(prepared, result.target.route ? &result.target : nullptr);
        RCLCPP_INFO(node_->get_logger(),
                    "[periodic_replan] hot installed waypoint=%d poses=%zu flags=%s",
                    current_index_, prepared.poses.size(), result.flags.c_str());
        EC_TRACE(ec::PATH_INSTALL, "replan_install", "wp=%d poses=%zu flags=%s",
                 current_index_, prepared.poses.size(), result.flags.c_str());
      } else if (gate_block_install) {
        RCLCPP_WARN(node_->get_logger(),
                    "[periodic_replan] install rejected by gate reason=%s (%s) "
                    "waypoint=%d",
                    gate.reason.c_str(), gate.detail.c_str(), current_index_);
        EC_TRACE(ec::PATH_REJECT, "replan_install", "reason=%s detail=%s wp=%d",
                 gate.reason.c_str(), gate.detail.c_str(), current_index_);
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[periodic_replan] candidate not installed waypoint=%d flags=%s",
                    current_index_, result.flags.c_str());
      }
    }
  }

  if (periodic_replan_running_ || initial_plan_future_.valid() ||
      initial_plan_requested_ || pending_route_goal_generation_ != 0 ||
      (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) ||
      current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    return;
  }
  if (!pathScanFresh() || (route_segment_enable_ && global_route_.poses.empty())) return;
  const rclcpp::Time now = node_->now();
  if (last_periodic_replan_at_.nanoseconds() != 0 &&
      (now - last_periodic_replan_at_).seconds() < periodic_replan_sec_) {
    return;
  }
  last_periodic_replan_at_ = now;
  EC_TRACE(ec::REPLAN_TRIGGER, "periodic_replan", "wp=%d", current_index_);
  std::vector<RouteTarget> targets;
  if (route_segment_enable_ && !global_route_.poses.empty()) {
    geometry_msgs::msg::PoseStamped pose;
    if (!getRobotPose(pose)) return;
    targets = routeTargets(pose, route_advance_requested_);
  } else {
    RouteTarget target;
    target.goal = waypointPose(waypoints_[current_index_]);
    targets.push_back(target);
  }
  if (targets.empty()) return;
  periodic_replan_running_ = true;
  const std::string mission_id = mission_id_;
  const int waypoint_index = current_index_;
  const std::uint64_t segment_generation = route_segment_generation_;
  const std::uint64_t path_generation = path_goal_generation_.load();
  const auto client = plan_client_;
  periodic_replan_future_ = std::async(
      std::launch::async,
      [this, targets, mission_id, waypoint_index, segment_generation,
       path_generation, client]() {
        PeriodicReplanResult result;
        result.mission_id = mission_id;
        result.waypoint_index = waypoint_index;
        result.segment_generation = segment_generation;
        result.path_generation = path_generation;
        auto request = std::make_shared<nav_interfaces::srv::PlanPath::Request>();
        request->shadow_only = true;
        // Retry only bounded forward intermediate anchors; never change the
        // final waypoint or fall back to an unchecked cached-route slice.
        for (const auto& target : targets) {
          result.target = target;
          request->goal = target.goal;
          auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanPath>,
                                   decltype(request)>(client, request, 2.0, &result.flags);
          if (!response) {
            return result;
          }
          result.flags = response->message;
          result.planned = response->success && !response->path.poses.empty();
          if (result.planned) {
            result.path = response->path;
            break;
          }
          if (result.flags != "GOAL_OCCUPIED" && result.flags != "NO_PATH_FOUND") break;
        }
        return result;
      });
}

void MissionManager::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  last_odom_rx_at_ = node_->now();
  linear_velocity_ = msg->twist.twist.linear.x;
  angular_velocity_ = msg->twist.twist.angular.z;
  if (local_planner_mode_ == "goal_pose") {
    last_odom_pose_.header = msg->header;
    last_odom_pose_.pose = msg->pose.pose;
    last_odom_pose_valid_ = true;
  }
}

void MissionManager::statusTimerCb() { publishStatus(); }

void MissionManager::pathDoneCb(
    std::uint64_t generation,
    const rclcpp_action::ClientGoalHandle<NavPathAction>::WrappedResult &result) {
  if (generation != path_goal_generation_.load()) return;
  const bool succeeded =
      result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
      result.result->status == 0;
  path_succeeded_.store(succeeded);
  path_done_.store(true);
  path_goal_active_.store(false);
  last_path_feedback_at_ = node_->now();
  RCLCPP_INFO(node_->get_logger(),
              "[mission_manager] NeuPAN path result state=%s status=%d succeeded=%s",
              resultCodeName(result.code),
              result.result ? result.result->status : -1,
              path_succeeded_.load() ? "true" : "false");
}

void MissionManager::pathActiveCb(std::uint64_t generation) {
  if (generation != path_goal_generation_.load()) return;
  path_done_.store(false);
  path_goal_active_.store(true);
  accepted_path_generation_.store(generation);
  last_path_feedback_at_ = node_->now();
}

void MissionManager::pathRejectedCb(std::uint64_t generation) {
  if (generation != path_goal_generation_.load()) return;
  path_succeeded_.store(false);
  path_done_.store(true);
  path_goal_active_.store(false);
  last_path_feedback_at_ = node_->now();
  RCLCPP_WARN(node_->get_logger(),
              "[mission_manager] NeuPAN path goal rejected by server");
}

void MissionManager::pathFeedbackCb(
    std::uint64_t generation,
    const std::shared_ptr<const NavPathAction::Feedback> &feedback) {
  if (generation != path_goal_generation_.load()) return;
  if (feedback) {
    distance_remaining_.store(feedback->distance_remaining);
    last_path_feedback_at_ = node_->now();
    constexpr double kStallProgressEpsilon = 0.05;  // meters
    if (last_feedback_progress_at_.nanoseconds() == 0 ||
        std::fabs(feedback->distance_remaining - last_feedback_distance_) >
            kStallProgressEpsilon) {
      last_feedback_distance_ = feedback->distance_remaining;
      last_feedback_progress_at_ = last_path_feedback_at_;
    }
  }
}

void MissionManager::replanCb(const std_msgs::msg::Empty::ConstSharedPtr &) {
  if (recovery_episode_active_) return;
  if (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME) {
    const rclcpp::Time now = node_->now();
    if (replan_cooldown_sec_ > 0.0 && last_replan_at_.nanoseconds() != 0 &&
        (now - last_replan_at_).seconds() < replan_cooldown_sec_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Ignoring NeuPAN replan inside %.1f s cooldown",
                           replan_cooldown_sec_);
      return;
    }
    last_replan_at_ = now;
    replan_requested_.store(true);
  }
}

void MissionManager::recoveryRequestCb(
    const nav_interfaces::msg::RecoveryRequest::ConstSharedPtr &msg) {
  if (!recovery_shadow_enable_ || !msg || msg->request_id.empty()) {
    return;
  }
  if (msg->state == nav_interfaces::msg::RecoveryRequest::RECOVERED ||
      msg->state == nav_interfaces::msg::RecoveryRequest::ABORTED) {
    if (msg->request_id == recovery_active_request_id_) {
      publishRecoveryCommand(*msg, nav_interfaces::msg::RecoveryCommand::ABORT, 0,
                             0.0, recovery_candidate_flags_, nav_msgs::msg::Path(),
                             "episode_recovered_or_aborted");
      recovery_active_request_id_.clear();
      clearRecoveryCandidate();
    }
    return;
  }
  if (msg->state == nav_interfaces::msg::RecoveryRequest::ESCAPING) {
    return;
  }
  if (msg->state == nav_interfaces::msg::RecoveryRequest::STILL_BLOCKED) {
    if (msg->request_id == recovery_candidate_id_) {
      publishRecoveryCommand(*msg, nav_interfaces::msg::RecoveryCommand::WAIT,
                             recovery_candidate_side_, 0.0,
                             recovery_candidate_flags_, recovery_candidate_path_,
                             "still_blocked");
    }
    return;
  }
  if (msg->state == nav_interfaces::msg::RecoveryRequest::CLEARED) {
    if (msg->request_id == recovery_candidate_id_) {
      commitRecoveryCandidate(*msg);
    } else if (msg->request_id == recovery_active_request_id_ &&
               recovery_shadow_running_) {
      recovery_commit_pending_ = true;
      recovery_pending_commit_request_ = *msg;
    } else {
      commitRecoveryCandidate(*msg);
    }
    return;
  }
  if (msg->state != nav_interfaces::msg::RecoveryRequest::BLOCKED ||
      msg->request_id == last_recovery_request_id_) {
    return;
  }
  last_recovery_request_id_ = msg->request_id;
  if ((state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) ||
      current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    publishRecoveryCommand(*msg, nav_interfaces::msg::RecoveryCommand::ABORT, 0, 0.0,
                           "", nav_msgs::msg::Path(), "mission_not_navigating");
    return;
  }
  clearRecoveryCandidate(true);
  recovery_active_request_id_ = msg->request_id;
  recovery_episode_active_ = recovery_behavior_enable_;
  if (recovery_shadow_running_) {
    publishRecoveryCommand(*msg, nav_interfaces::msg::RecoveryCommand::WAIT, 0, 0.0,
                           "", nav_msgs::msg::Path(), "shadow_busy");
    return;
  }

  const nav_interfaces::msg::RecoveryRequest request = *msg;
  const geometry_msgs::msg::PoseStamped goal = waypointPose(waypoints_[current_index_]);
  const std::string mission_id = mission_id_;
  const int waypoint_index = current_index_;
  const auto client = plan_client_;
  recovery_shadow_running_ = true;
  recovery_shadow_future_ = std::async(
      std::launch::async,
      [this, request, goal, mission_id, waypoint_index, client]() {
        ShadowResult result;
        result.request = request;
        result.mission_id = mission_id;
        result.waypoint_index = waypoint_index;
        auto request_msg = std::make_shared<nav_interfaces::srv::PlanPath::Request>();
        request_msg->goal = goal;
        request_msg->shadow_only = true;
        auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanPath>,
                                 std::shared_ptr<nav_interfaces::srv::PlanPath::Request>>(
            client, request_msg, 10.0);
        if (!response) {
          result.reason = "PLAN_SERVICE_UNAVAILABLE";
          return result;
        }
        result.flags = response->message;
        result.reason = response->message;
        result.planned = response->success && !response->path.poses.empty();
        if (result.planned) result.path = response->path;
        return result;
      });
  RCLCPP_INFO(node_->get_logger(),
              "[recovery_shadow] started id=%s local_side=%d (no preempt)",
              msg->request_id.c_str(), static_cast<int>(msg->local_side));
}

void MissionManager::processRecoveryShadow() {
  if (!recovery_shadow_running_ || !recovery_shadow_future_.valid() ||
      recovery_shadow_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
    return;
  }
  ShadowResult result = recovery_shadow_future_.get();
  recovery_shadow_running_ = false;
  if (result.request.request_id != recovery_active_request_id_ ||
      result.mission_id != mission_id_ ||
      result.waypoint_index != current_index_) {
    publishRecoveryCommand(result.request, nav_interfaces::msg::RecoveryCommand::ABORT,
                           0, 0.0, result.flags, nav_msgs::msg::Path(),
                           "stale_mission_or_waypoint");
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = result.request.header;
  pose.pose = result.request.robot_pose;
  if (!result.planned && recovery_behavior_enable_) {
    nav_msgs::msg::Path fallback = prepareRecoveryPath(active_path_, pose);
    if (fallback.poses.size() >= 2) {
      const std::string failure = result.reason;
      result.planned = true;
      result.path = fallback;
      result.flags = "OK|FALLBACK_ACTIVE_PATH";
      result.reason = "fallback_active_path:" + failure;
    }
  }
  if (!result.planned) {
    publishRecoveryCommand(result.request, nav_interfaces::msg::RecoveryCommand::WAIT,
                           0, 0.0, result.flags, nav_msgs::msg::Path(), result.reason);
    RCLCPP_WARN(node_->get_logger(), "[recovery_shadow] id=%s no candidate: %s",
                result.request.request_id.c_str(), result.reason.c_str());
    return;
  }

  const double delta = candidateHeadingDelta(result.path, pose);
  constexpr double kSideDeadband = 5.0 * M_PI / 180.0;
  const int8_t side = std::fabs(delta) > kSideDeadband
                          ? (delta > 0.0 ? 1 : -1)
                          : result.request.local_side;
  const bool conflict = result.request.local_side != 0 && side != 0 &&
                        result.request.local_side != side;
  const bool fallback =
      result.flags.find("FALLBACK_ACTIVE_PATH") != std::string::npos;
  uint8_t command = nav_interfaces::msg::RecoveryCommand::SIDE;
  std::string reason = fallback
                           ? result.reason
                           : (conflict ? "direction_conflict"
                                       : "direction_consistent");
  if (recovery_behavior_enable_) {
    recovery_candidate_id_ = result.request.request_id;
    recovery_candidate_mission_id_ = result.mission_id;
    recovery_candidate_waypoint_index_ = result.waypoint_index;
    recovery_candidate_side_ = side;
    recovery_candidate_flags_ = result.flags;
    recovery_candidate_path_ = result.path;
    recovery_candidate_created_at_ = node_->now();
  } else if (conflict) {
    command = nav_interfaces::msg::RecoveryCommand::WAIT;
  }
  publishRecoveryCommand(result.request, command, side, delta, result.flags,
                         result.path, reason);
  RCLCPP_WARN(node_->get_logger(),
              "[recovery_shadow] id=%s local_side=%d candidate_side=%d "
              "heading=%+.1fdeg flags=%s result=%s behavior=%s",
              result.request.request_id.c_str(),
              static_cast<int>(result.request.local_side), static_cast<int>(side),
              delta * 180.0 / M_PI, result.flags.c_str(), reason.c_str(),
              recovery_behavior_enable_ ? "true" : "false");
  if (recovery_commit_pending_ &&
      recovery_pending_commit_request_.request_id == result.request.request_id) {
    const nav_interfaces::msg::RecoveryRequest pending =
        recovery_pending_commit_request_;
    recovery_commit_pending_ = false;
    if (command == nav_interfaces::msg::RecoveryCommand::SIDE) {
      commitRecoveryCandidate(pending);
    }
  }
}

void MissionManager::clearRecoveryCandidate(bool notify_abort) {
  const std::string request_id = recovery_candidate_id_.empty()
                                     ? recovery_active_request_id_
                                     : recovery_candidate_id_;
  if (notify_abort && !request_id.empty()) {
    nav_interfaces::msg::RecoveryCommand response;
    response.header.stamp = node_->now();
    response.header.frame_id = map_frame_;
    response.request_id = request_id;
    response.command = nav_interfaces::msg::RecoveryCommand::ABORT;
    response.reason = "mission_state_changed";
    recovery_command_pub_->publish(response);
  }
  recovery_candidate_id_.clear();
  recovery_candidate_mission_id_.clear();
  recovery_candidate_waypoint_index_ = -1;
  recovery_candidate_side_ = 0;
  recovery_candidate_flags_.clear();
  recovery_candidate_path_.poses.clear();
  recovery_candidate_created_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  recovery_active_request_id_.clear();
  recovery_episode_active_ = false;
  recovery_commit_pending_ = false;
}

nav_msgs::msg::Path MissionManager::prepareRecoveryPath(
    const nav_msgs::msg::Path &candidate,
    const geometry_msgs::msg::PoseStamped &pose) const {
  nav_msgs::msg::Path prepared;
  if (candidate.poses.empty() || candidate.header.frame_id != map_frame_) {
    return prepared;
  }
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < candidate.poses.size(); ++i) {
    const double distance = std::hypot(
        candidate.poses[i].pose.position.x - pose.pose.position.x,
        candidate.poses[i].pose.position.y - pose.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  if (nearest_distance > recovery_candidate_max_start_dist_m_) return prepared;

  prepared.header = candidate.header;
  prepared.header.stamp = node_->now();
  geometry_msgs::msg::PoseStamped start = pose;
  start.header = prepared.header;
  prepared.poses.push_back(start);
  const std::size_t begin =
      nearest_distance < 0.05 ? nearest_index + 1 : nearest_index;
  for (std::size_t i = begin; i < candidate.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped point = candidate.poses[i];
    point.header = prepared.header;
    prepared.poses.push_back(point);
  }
  return prepared;
}

nav_msgs::msg::Path MissionManager::preparePeriodicPath(
    const nav_msgs::msg::Path &candidate,
    const geometry_msgs::msg::PoseStamped &pose) const {
  nav_msgs::msg::Path prepared;
  if (candidate.poses.size() < 2 || candidate.header.frame_id != map_frame_) {
    return prepared;
  }
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < candidate.poses.size(); ++i) {
    const auto &point = candidate.poses[i].pose.position;
    const double distance = std::hypot(
        point.x - pose.pose.position.x, point.y - pose.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  if (nearest_distance > 2.0) return prepared;
  prepared.header = candidate.header;
  prepared.header.stamp = node_->now();
  std::size_t begin = nearest_index;
  begin = std::min(begin, candidate.poses.size() - 2);
  for (std::size_t i = begin; i < candidate.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped point = candidate.poses[i];
    point.header = prepared.header;
    prepared.poses.push_back(point);
  }
  return prepared;
}

void MissionManager::commitRecoveryCandidate(
    const nav_interfaces::msg::RecoveryRequest &request) {
  if (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::ABORT, 0,
                           0.0, recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "mission_state_not_navigating");
    clearRecoveryCandidate();
    return;
  }
  if (!recovery_behavior_enable_ || request.request_id != recovery_candidate_id_ ||
      recovery_candidate_mission_id_ != mission_id_ ||
      recovery_candidate_waypoint_index_ != current_index_ ||
      recovery_candidate_path_.poses.empty()) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::WAIT, 0,
                           0.0, recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "candidate_missing_or_stale");
    return;
  }
  if (recovery_candidate_max_age_sec_ > 0.0 &&
      (node_->now() - recovery_candidate_created_at_).seconds() >
          recovery_candidate_max_age_sec_) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "candidate_expired");
    clearRecoveryCandidate();
    return;
  }

  if (recovery_candidate_path_.header.frame_id != map_frame_) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::ABORT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "candidate_wrong_frame");
    clearRecoveryCandidate();
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(pose)) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "robot_pose_unavailable");
    return;
  }
  nav_msgs::msg::Path prepared = prepareRecoveryPath(recovery_candidate_path_, pose);
  if (prepared.poses.size() < 2) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "candidate_start_too_far");
    clearRecoveryCandidate();
    return;
  }
  if (!path_client_->wait_for_action_server(tf2::durationFromSec(plan_timeout_sec_))) {
    publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::msg::Path(),
                           "nav_path_action_unavailable");
    return;
  }

  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = node_->now();
  last_path_feedback_at_ = path_started_at_;
  last_feedback_progress_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_feedback_distance_ = -1.0;
  active_path_ = prepared;
  sendPathGoal(prepared);
  publishRecoveryCommand(request, nav_interfaces::msg::RecoveryCommand::COMMIT,
                         recovery_candidate_side_, 0.0,
                         recovery_candidate_flags_, prepared,
                         "cached_candidate_committed");
  publishEvent("RECOVERY_COMMIT", "Cached recovery candidate committed");
  clearRecoveryCandidate();
}

void MissionManager::publishRecoveryCommand(
    const nav_interfaces::msg::RecoveryRequest &request, uint8_t command, int8_t side,
    double signed_heading, const std::string &flags,
    const nav_msgs::msg::Path &path, const std::string &reason) {
  nav_interfaces::msg::RecoveryCommand response;
  response.header.stamp = node_->now();
  response.header.frame_id = map_frame_;
  response.request_id = request.request_id;
  response.command = command;
  response.side = side;
  response.signed_heading = signed_heading;
  response.flags = flags;
  response.path = path;
  response.reason = reason;
  recovery_command_pub_->publish(response);
}

double MissionManager::candidateHeadingDelta(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &pose) const {
  if (path.poses.size() < 2) return 0.0;
  const auto &start = path.poses.front().pose.position;
  geometry_msgs::msg::Point target = path.poses.back().pose.position;
  double arc = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    if (arc + segment >= 1.0 && segment > 1e-9) {
      const double ratio = (1.0 - arc) / segment;
      target.x = a.x + ratio * (b.x - a.x);
      target.y = a.y + ratio * (b.y - a.y);
      break;
    }
    arc += segment;
  }
  const double heading = std::atan2(target.y - start.y, target.x - start.x);
  return normalizeAngle(heading - tf2::getYaw(pose.pose.orientation));
}

void MissionManager::skipCurrentWaypointOrFail(const std::string &fail_reason) {
  if (temporary_mission_) {
    restorePausedMission("Temporary " + fail_reason + "; main mission remains paused");
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "Skipping waypoint %d after failure: %s",
              current_index_, fail_reason.c_str());
  publishEvent("WAYPOINT_SKIPPED",
               "Waypoint " + std::to_string(current_index_) + " skipped: " + fail_reason);
  const int total = static_cast<int>(waypoints_.size());
  if (current_index_ + 1 < total) {
    ++current_index_;
  } else if (loop_ && total > 0) {
    current_index_ = 0;
  } else {
    // Report how much of the route actually completed. A route where 4 of 5
    // waypoints succeeded and only the last one failed to align is very
    // different, operationally, from one that never got anywhere -- but both
    // land on FAILED. Putting the counts in the message lets the operator UI
    // show that without needing a new State value (see completed_waypoints_).
    transitionTo(State::FAILED,
                 "All remaining waypoints failed: " + fail_reason +
                     "（共 " + std::to_string(total) + " 个航点，已完成 " +
                     std::to_string(completed_waypoints_) + " 个）");
    return;
  }
  if (!startCurrentWaypoint()) {
    stopNavigation();
  }
}

bool MissionManager::startCurrentWaypoint() {
  if (local_planner_mode_ == "action" && navigation_mode_ == "replan_periodic") {
    return startInitialRoutePlan();
  }
  clearRouteSegment();
  resetInstallGateState("waypoint");
  if (local_planner_mode_ == "goal_pose") {
    return startSruWaypoint();
  }
  if (navigation_mode_ == "replan_periodic" && !pathScanFresh()) {
    holdForPathSafety("SCAN_STALE");
    last_periodic_replan_at_ = node_->now();
    transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
    return true;
  }
  const int total = static_cast<int>(waypoints_.size());
  // Try each remaining waypoint at most once (bounded by `total`); a
  // waypoint whose plan keeps failing after retries is skipped and the
  // next one is attempted instead of failing the whole mission.
  for (int skipped = 0; skipped < total; ++skipped) {
    if (current_index_ < 0 || current_index_ >= total) {
      if (temporary_mission_) {
        restorePausedMission("Temporary waypoint index is invalid; main mission remains paused");
      } else {
        transitionTo(State::FAILED, "Waypoint index is invalid");
      }
      return false;
    }

    nav_msgs::msg::Path path;
    std::string fail_reason;
    bool planned = planCurrentWaypoint(path, fail_reason) && !path.poses.empty();
    for (int attempt = 0;
         !planned && attempt < plan_max_retries_ &&
         (fail_reason == "START_OCCUPIED" || fail_reason == "START_AND_GOAL_OCCUPIED" ||
          fail_reason == "GOAL_OCCUPIED");
         ++attempt) {
      RCLCPP_WARN(node_->get_logger(),
                  "Global planning failed (%s) for waypoint %d, retrying in %.1f s (attempt %d/%d)",
                  fail_reason.c_str(), current_index_, plan_retry_backoff_sec_, attempt + 1,
                  plan_max_retries_);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(plan_retry_backoff_sec_)));
      planned = planCurrentWaypoint(path, fail_reason) && !path.poses.empty();
    }

    if (!planned && navigation_mode_ == "replan_periodic" &&
        (fail_reason.find("DYNAMIC_MAP") != std::string::npos ||
         fail_reason == "ROBOT_POSE_UNAVAILABLE" ||
         fail_reason == "PLAN_SERVICE_UNAVAILABLE")) {
      holdForPathSafety(fail_reason);
      last_periodic_replan_at_ = node_->now();
      transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
      return true;
    }
    if (!planned) {
      if (temporary_mission_) {
        restorePausedMission("Temporary global planning failed; main mission remains paused");
        return false;
      }
      RCLCPP_WARN(node_->get_logger(), "Skipping waypoint %d after planning failure: %s",
                  current_index_, fail_reason.c_str());
      std::string skip_message =
          "Waypoint " + std::to_string(current_index_) + " skipped: " + fail_reason;
      if (fail_reason == "GOAL_OCCUPIED" || fail_reason == "START_AND_GOAL_OCCUPIED") {
        skip_message += "; 第" + std::to_string(current_index_ + 1) + "个航点有障碍物，无法到达";
      }
      publishEvent("WAYPOINT_SKIPPED", skip_message);
      if (current_index_ + 1 < total) {
        ++current_index_;
        continue;
      }
      if (loop_ && total > 0) {
        current_index_ = 0;
        continue;
      }
      break;
    }

    RouteTarget initial_target;
    if (route_segment_enable_ && navigation_mode_ == "replan_periodic") {
      global_route_ = path;
      geometry_msgs::msg::PoseStamped pose;
      if (!getRobotPose(pose)) pose = path.poses.front();
      const auto targets = routeTargets(pose, true);
      if (!targets.empty()) {
        initial_target = targets.front();
        path.poses.resize(initial_target.anchor + 1);
      }
    }
    if (!validateForInstall(path, fail_reason)) {
      holdForPathSafety(fail_reason);
      last_periodic_replan_at_ = node_->now();
      transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
      return true;
    }

    if (!path_client_->wait_for_action_server(tf2::durationFromSec(plan_timeout_sec_))) {
      if (temporary_mission_) {
        restorePausedMission("Temporary NeuPAN action unavailable; main mission remains paused");
      } else {
        transitionTo(State::FAILED, "NeuPAN path action is unavailable");
      }
      return false;
    }
    path_done_.store(false);
    path_succeeded_.store(false);
    distance_remaining_.store(-1.0);
    path_started_at_ = node_->now();
    last_path_feedback_at_ = path_started_at_;
    last_feedback_progress_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_feedback_distance_ = -1.0;
    active_path_ = path;
    path_safety_waiting_ = false;
    if (navigation_mode_ == "replan_periodic") {
      last_periodic_replan_at_ = node_->now();
    }
    sendPathGoal(path, initial_target.route ? &initial_target : nullptr);
    transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
    return true;
  }

  transitionTo(State::FAILED,
               "All remaining waypoints failed to plan（共 " +
                   std::to_string(total) + " 个航点，已完成 " +
                   std::to_string(completed_waypoints_) + " 个）");
  return false;
}

bool MissionManager::planCurrentWaypoint(nav_msgs::msg::Path &path,
                                         std::string &fail_reason) {
  auto request = std::make_shared<nav_interfaces::srv::PlanPath::Request>();
  request->goal = waypointPose(waypoints_[current_index_]);
  request->shadow_only =
      route_segment_enable_ && navigation_mode_ == "replan_periodic";
  auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanPath>,
                           std::shared_ptr<nav_interfaces::srv::PlanPath::Request>>(
      plan_client_, request, plan_timeout_sec_);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "Map planner service is unavailable: %s",
                plan_service_name_.c_str());
    fail_reason = "PLAN_SERVICE_UNAVAILABLE";
    return false;
  }
  if (!response->success) {
    fail_reason = response->message;
    return false;
  }
  path = response->path;
  return true;
}

// SRU local planner backend (local_planner=sru)

bool MissionManager::sruGoalFromWaypoint(const nav_interfaces::msg::Waypoint &waypoint,
                                         geometry_msgs::msg::PoseStamped &out) const {
  geometry_msgs::msg::PoseStamped goal_map = waypointPose(waypoint);
  try {
    // SRU requires goal frame_id to match its odometry frame (default odom).
    tf_buffer_->transform(goal_map, out, odom_frame_, tfTimeout(0.5));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[sru] map->%s transform failed for waypoint %d: %s",
                odom_frame_.c_str(), current_index_, ex.what());
    return false;
  }
  out.header.frame_id = odom_frame_;
  out.header.stamp = node_->now();
  return true;
}

bool MissionManager::startSruWaypoint() {
  const int total = static_cast<int>(waypoints_.size());
  if (current_index_ < 0 || current_index_ >= total) {
    if (temporary_mission_) {
      restorePausedMission("Temporary waypoint index is invalid; main mission remains paused");
    } else {
      transitionTo(State::FAILED, "Waypoint index is invalid");
    }
    return false;
  }

  geometry_msgs::msg::PoseStamped goal_odom;
  if (!sruGoalFromWaypoint(waypoints_[current_index_], goal_odom)) {
    RCLCPP_WARN(node_->get_logger(), "[sru] skipping waypoint %d: map->%s transform failed",
                current_index_, odom_frame_.c_str());
    publishEvent("WAYPOINT_SKIPPED",
                 "Waypoint " + std::to_string(current_index_) +
                     " skipped: SRU goal transform failed");
    if (current_index_ + 1 < total) {
      ++current_index_;
      return startSruWaypoint();
    }
    if (loop_ && total > 0) {
      current_index_ = 0;
      return startSruWaypoint();
    }
    transitionTo(State::FAILED,
                 "All remaining waypoints failed: SRU goal transform failed（共 " +
                     std::to_string(total) + " 个航点，已完成 " +
                     std::to_string(completed_waypoints_) + " 个）");
    return false;
  }

  clearRouteSegment();
  active_path_.poses.clear();
  sru_goal_odom_ = goal_odom;
  sru_goal_valid_ = true;
  sru_last_refresh_at_ = node_->now();
  sru_stall_ref_distance_ = -1.0;
  sru_stall_ref_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  sru_goal_pub_->publish(goal_odom);
  publishSruEnable(true);
  RCLCPP_INFO(node_->get_logger(),
              "[sru] waypoint %d dispatched: map=(%.2f, %.2f, yaw=%.2f) -> odom=(%.2f, %.2f)",
              current_index_, waypoints_[current_index_].x, waypoints_[current_index_].y,
              waypoints_[current_index_].yaw, goal_odom.pose.position.x,
              goal_odom.pose.position.y);
  transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
  return true;
}

void MissionManager::processSruNavigation() {
  const rclcpp::Time now = node_->now();

  if (sru_cmd_vel_seen_ &&
      (now - sru_last_cmd_vel_at_).seconds() > sru_cmd_vel_silence_sec_) {
    if (!sru_silence_warned_) {
      sru_silence_warned_ = true;
      RCLCPP_ERROR(node_->get_logger(),
                   "[sru] no /sru/cmd_vel for %.1fs; SRU node may have stalled or crashed",
                   (now - sru_last_cmd_vel_at_).seconds());
      EC_TRACE(ec::NEUPAN_FEEDBACK_FROZEN, "sru", "cmd_vel silent %.1fs",
               (now - sru_last_cmd_vel_at_).seconds());
    }
  } else if (sru_silence_warned_) {
    sru_silence_warned_ = false;
    RCLCPP_INFO(node_->get_logger(), "[sru] /sru/cmd_vel resumed");
  }

  if (current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    return;
  }

  // Measure distance in sru_robot_frame_ so arrival and stall checks use the same
  // reference frame as SRU's internal arrival check.
  geometry_msgs::msg::PoseStamped current;
  if (!getRobotPoseInFrame(sru_robot_frame_, current)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[sru] robot pose unavailable; arrival judgement skipped");
    return;
  }
  const nav_interfaces::msg::Waypoint &target = waypoints_[current_index_];
  const double remaining =
      std::hypot(target.x - current.pose.position.x, target.y - current.pose.position.y);

  if (remaining <= sru_arrive_threshold_m_) {
    RCLCPP_INFO(node_->get_logger(), "[sru] waypoint %d reached: distance=%.3fm <= %.2fm",
                current_index_, remaining, sru_arrive_threshold_m_);
    transitionTo(State::ARRIVED);
    return;
  }

  if (nav_stall_timeout_sec_ > 0.0) {
    if (sru_stall_ref_distance_ < 0.0 ||
        std::fabs(remaining - sru_stall_ref_distance_) > sru_stall_progress_epsilon_m_) {
      sru_stall_ref_distance_ = remaining;
      sru_stall_ref_at_ = now;
    } else if (sru_stall_ref_at_.nanoseconds() != 0 &&
               (now - sru_stall_ref_at_).seconds() > nav_stall_timeout_sec_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[sru] progress stalled: distance=%.3fm unchanged for %.1fs",
                   remaining, (now - sru_stall_ref_at_).seconds());
      stopNavigation();
      skipCurrentWaypointOrFail("SRU progress stalled");
      return;
    }
  }

  // Refresh odom goal when map->odom drift exceeds threshold.
  if (sru_goal_valid_ && sru_last_refresh_at_.nanoseconds() != 0 &&
      (now - sru_last_refresh_at_).seconds() >= sru_goal_refresh_sec_) {
    sru_last_refresh_at_ = now;
    geometry_msgs::msg::PoseStamped refreshed;
    if (sruGoalFromWaypoint(target, refreshed)) {
      const double drift = std::hypot(
          refreshed.pose.position.x - sru_goal_odom_.pose.position.x,
          refreshed.pose.position.y - sru_goal_odom_.pose.position.y);
      if (drift > sru_goal_refresh_min_delta_m_) {
        sru_goal_odom_ = refreshed;
        sru_goal_pub_->publish(refreshed);
        RCLCPP_INFO(node_->get_logger(), "[sru] goal refreshed: map->odom drifted %.2fm",
                    drift);
      }
    }
  }
}

void MissionManager::publishSruEnable(bool enable) {
  if (!sru_enable_pub_) return;
  std_msgs::msg::Bool msg;
  msg.data = enable;
  sru_enable_pub_->publish(msg);
}

void MissionManager::sruCmdVelCb(const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
  (void)msg;
  sru_last_cmd_vel_at_ = node_->now();
  sru_cmd_vel_seen_ = true;
}

void MissionManager::publishGlobalPathVisualization() {
  if (navigation_mode_ == "replan_periodic" && local_planner_mode_ == "action") {
    if (!global_route_.poses.empty()) global_path_pub_->publish(global_route_);
    return;
  }
  geometry_msgs::msg::PoseStamped start;
  if (!getRobotPose(start)) {
    RCLCPP_WARN(node_->get_logger(),
                "Unable to publish global path visualization without robot pose");
    return;
  }
  auto request = std::make_shared<nav_interfaces::srv::PlanSequence::Request>();
  request->start = start;
  request->loop = loop_;
  request->goals.reserve(waypoints_.size());
  for (const auto &waypoint : waypoints_) {
    request->goals.push_back(waypointPose(waypoint));
  }
  auto response = callSync<rclcpp::Client<nav_interfaces::srv::PlanSequence>,
                           std::shared_ptr<nav_interfaces::srv::PlanSequence::Request>>(
      sequence_plan_client_, request, plan_timeout_sec_);
  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "Sequence planner service is unavailable");
    return;
  }
  if (!response->success || response->path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Global path visualization planning failed: %s",
                response->message.c_str());
    return;
  }
  global_path_pub_->publish(response->path);
}

bool MissionManager::performAlignment(const nav_interfaces::msg::Waypoint &target) {
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  // Settle window so atomic_move consumes the /neupan/stop published by stopNavigation()
  // before the align service request arrives.
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(handover_settle_sec_)));
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
              "[mission_manager][alignment] absolute target frame=%s x=%.3f y=%.3f yaw=%.3f",
              map_frame_.c_str(), target.x, target.y, target.yaw);
  auto align_request = std::make_shared<atomic_move::srv::AlignTo::Request>();
  align_request->x = target.x;
  align_request->y = target.y;
  align_request->yaw = target.yaw;
  auto align_response = callSync<rclcpp::Client<atomic_move::srv::AlignTo>,
                                 std::shared_ptr<atomic_move::srv::AlignTo::Request>>(
      align_client_, align_request, 30.0);
  const bool call_ok = align_response != nullptr;
  const bool call_success = call_ok && align_response->success;
  const std::string call_message = call_ok ? align_response->message : "SERVICE_TIMEOUT_OR_UNAVAILABLE";
  RCLCPP_INFO(node_->get_logger(),
              "[mission_manager][alignment] absolute align response call_ok=%s success=%s message='%s'",
              call_ok ? "true" : "false", call_success ? "true" : "false",
              call_message.c_str());
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  if (call_ok && !align_response->success &&
      align_response->message == "ALIGN_CANCELLED") {
    RCLCPP_ERROR(node_->get_logger(),
                 "[mission_manager][alignment] atomic_move reported ALIGN_CANCELLED even "
                 "after a %.3f s handover settle; a /neupan/stop is still racing the align "
                 "request (see performAlignment comment) -- consider raising "
                 "handover_settle_sec",
                 handover_settle_sec_);
  }
  return call_success;
}

void MissionManager::startAlignment() {
  joinAlignmentThread();
  alignment_cancel_requested_.store(false);
  alignment_result_pending_.store(false);
  alignment_result_.store(false);
  alignment_running_.store(true);
  const uint64_t generation = alignment_generation_.fetch_add(1) + 1;
  const nav_interfaces::msg::Waypoint target = waypoints_[current_index_];
  alignment_thread_ = std::thread([this, target, generation]() {
    const bool result = performAlignment(target);
    if (generation != alignment_generation_.load()) {
      return;
    }
    alignment_result_.store(result);
    alignment_running_.store(false);
    alignment_result_pending_.store(true);
  });
}

void MissionManager::joinAlignmentThread() {
  if (alignment_thread_.joinable()) {
    alignment_thread_.join();
  }
  alignment_running_.store(false);
}

bool MissionManager::captureHomePoseIfNeeded() {
  if (home_pose_valid_) {
    return true;
  }
  geometry_msgs::msg::PoseStamped odom_origin;
  odom_origin.header.frame_id = odom_frame_;
  odom_origin.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom_origin.pose.orientation.w = 1.0;
  try {
    geometry_msgs::msg::PoseStamped map_origin;
    tf_buffer_->transform(odom_origin, map_origin, map_frame_, tfTimeout(0.5));
    home_pose_ = map_origin.pose;
    home_pose_valid_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "Captured fixed home pose from %s origin in %s at x=%.3f y=%.3f yaw=%.3f",
                odom_frame_.c_str(), map_frame_.c_str(), home_pose_.position.x,
                home_pose_.position.y, tf2::getYaw(home_pose_.orientation));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Unable to transform %s origin to %s: %s",
                         odom_frame_.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
}

bool MissionManager::getRobotPose(geometry_msgs::msg::PoseStamped &pose) const {
  return getRobotPoseInFrame(base_frame_, pose);
}

bool MissionManager::getRobotPoseInFrame(
    const std::string &frame, geometry_msgs::msg::PoseStamped &pose) const {
  try {
    const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        map_frame_, frame, tf2::TimePointZero, tfTimeout(0.2));
    const rclcpp::Time stamp(transform.header.stamp);
    const double age = (node_->now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > path_safety_scan_max_age_sec_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "Robot pose TF is not fresh (age %.3fs)", age);
      return false;
    }
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Robot pose lookup failed: %s", ex.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped MissionManager::waypointPose(
    const nav_interfaces::msg::Waypoint &waypoint) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map_frame_;
  pose.header.stamp = node_->now();
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, waypoint.yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

void MissionManager::beginDwell() {
  alignment_started_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ++completed_waypoints_;
  dwell_duration_ = std::max(0.0, waypoints_[current_index_].dwell_sec);
  dwell_remaining_ = dwell_duration_;
  dwell_start_ = node_->now();
  transitionTo(State::DWELLING);
}

void MissionManager::completeMission() {
  if (temporary_mission_) {
    restorePausedMission("Temporary mission completed; main mission remains paused");
    return;
  }
  stopNavigation();
  transitionTo(State::COMPLETED, "Mission complete");
}

void MissionManager::restorePausedMission(const std::string &message) {
  if (!temporary_mission_ || paused_waypoints_.empty()) {
    return;
  }
  stopNavigation();
  waypoints_ = paused_waypoints_;
  current_index_ = paused_current_index_;
  loop_ = paused_loop_;
  mission_id_ = paused_mission_id_;
  temporary_mission_ = false;
  paused_waypoints_.clear();
  paused_current_index_ = -1;
  paused_loop_ = false;
  paused_mission_id_.clear();
  returning_home_ = false;
  transitionTo(State::PAUSED, message);
}

void MissionManager::stopNavigation() {
  alignment_generation_.fetch_add(1);
  path_goal_generation_.fetch_add(1);
  initial_plan_requested_ = false;
  ++route_segment_generation_;
  pending_route_goal_generation_ = 0;
  route_advance_requested_ = false;
  path_safety_waiting_ = false;
  alignment_cancel_requested_.store(true);
  if (path_client_) {
    path_client_->async_cancel_all_goals();
  }
  stop_pub_->publish(std_msgs::msg::Empty());
  if (local_planner_mode_ == "goal_pose") {
    // Disable SRU cmd_vel relay gate and send current pose as goal so SRU enters stopped state.
    publishSruEnable(false);
    if (sru_goal_pub_ && last_odom_pose_valid_) {
      geometry_msgs::msg::PoseStamped self = last_odom_pose_;
      self.header.stamp = node_->now();
      sru_goal_pub_->publish(self);
    }
    sru_goal_valid_ = false;
    sru_stall_ref_distance_ = -1.0;
    sru_stall_ref_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_path_feedback_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_feedback_progress_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_feedback_distance_ = -1.0;
}

void MissionManager::transitionTo(State state, const std::string &message) {
  state_ = state;
  publishEvent(stateName(state), message);
  publishStatus();
}

void MissionManager::publishStatus() const {
  nav_interfaces::msg::MissionStatus status;
  status.state = stateName(state_);
  status.mission_id = mission_id_;
  status.current_waypoint_idx = current_index_ < 0 ? 0 : current_index_;
  status.total_waypoints = waypoints_.size();
  if (current_index_ >= 0 && current_index_ < static_cast<int>(waypoints_.size())) {
    status.current_target = waypointPose(waypoints_[current_index_]).pose;
  } else if (dynamic_mode_ && dynamic_goal_valid_) {
    status.current_target = dynamic_goal_.pose;
  }
  status.dwell_remaining_sec = dwell_remaining_;
  status.progress_percent = waypoints_.empty()
                                ? 0.0
                                : 100.0 * (current_index_ + 1) / waypoints_.size();
  status.distance_to_target_m = distanceToTarget();
  status.linear_vel = linear_velocity_;
  status.angular_vel = angular_velocity_;
  status_pub_->publish(status);
}

void MissionManager::publishEvent(const std::string &event,
                                  const std::string &message) const {
  nav_interfaces::msg::MissionEvent msg;
  msg.event = event;
  msg.waypoint_idx = current_index_ < 0 ? 0 : current_index_;
  if (current_index_ >= 0 && current_index_ < static_cast<int>(waypoints_.size())) {
    msg.waypoint_name = waypoints_[current_index_].name;
  }
  msg.timestamp = node_->now().seconds();
  msg.message = message;
  event_pub_->publish(msg);
}

void MissionManager::publishOperatorCommand(const std::string &command_id,
                                            const std::string &command,
                                            const std::string &phase,
                                            const std::string &payload_json,
                                            bool success,
                                            const std::string &message) const {
  nav_interfaces::msg::OperatorCommand msg;
  msg.header.stamp = node_->now();
  msg.command_id = command_id;
  msg.source = "mission_manager";
  msg.command = command;
  msg.phase = phase;
  msg.payload_json = payload_json;
  msg.success = success;
  msg.message = message;
  operator_command_pub_->publish(msg);
}

std::string MissionManager::stateName(State state) const {
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::NAVIGATING: return "NAVIGATING";
    case State::ARRIVED: return "ARRIVED";
    case State::ALIGNING: return "ALIGNING";
    case State::DWELLING: return "DWELLING";
    case State::PAUSED: return "PAUSED";
    case State::RETURNING_HOME: return "RETURNING_HOME";
    case State::CANCELLED: return "CANCELLED";
    case State::COMPLETED: return "COMPLETED";
    case State::FAILED: return "FAILED";
    case State::DYNAMIC: return "DYNAMIC";
    case State::DYNAMIC_SEARCHING: return "DYNAMIC_SEARCHING";
  }
  return "UNKNOWN";
}

std::string MissionManager::newMissionId() const {
  std::ostringstream stream;
  stream << "mission_" << node_->now().nanoseconds();
  return stream.str();
}

std::string MissionManager::newCommandId() const {
  std::ostringstream stream;
  stream << "command_" << node_->now().nanoseconds();
  return stream.str();
}

double MissionManager::distanceToTarget() const {
  if (current_index_ >= 0 && current_index_ < static_cast<int>(waypoints_.size())) {
    geometry_msgs::msg::PoseStamped current;
    if (getRobotPose(current)) {
      const nav_interfaces::msg::Waypoint &target = waypoints_[current_index_];
      return std::hypot(target.x - current.pose.position.x,
                        target.y - current.pose.position.y);
    }
  }
  const double feedback = distance_remaining_.load();
  return feedback >= 0.0 ? feedback : 0.0;
}

double MissionManager::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::setenv("EC_NODE", "mission_manager", 1);
  EC_TRACE(ec::NODE_READY, "startup", "mission_manager up pid=%d", (int)::getpid());
  auto node = std::make_shared<rclcpp::Node>("mission_manager");
  MissionManager manager(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
