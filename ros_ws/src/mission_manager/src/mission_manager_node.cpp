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
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

MissionManager::MissionManager(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_) {
  private_nh_.param("map_frame", map_frame_, map_frame_);
  private_nh_.param("base_frame", base_frame_, base_frame_);
  private_nh_.param("odom_frame", odom_frame_, odom_frame_);
  private_nh_.param("odom_topic", odom_topic_, odom_topic_);
  private_nh_.param("plan_service", plan_service_name_, plan_service_name_);
  private_nh_.param("nav_path_action", nav_path_action_name_, nav_path_action_name_);
  private_nh_.param("stop_topic", stop_topic_, stop_topic_);
  private_nh_.param("goal_topic", sru_goal_topic_, sru_goal_topic_);
  private_nh_.param("enable_topic", sru_enable_topic_, sru_enable_topic_);
  private_nh_.param("planner_cmd_vel_topic", sru_cmd_vel_topic_, sru_cmd_vel_topic_);
  private_nh_.param("plan_timeout_sec", plan_timeout_sec_, plan_timeout_sec_);
  private_nh_.param("plan_max_retries", plan_max_retries_, plan_max_retries_);
  private_nh_.param("plan_retry_backoff_sec", plan_retry_backoff_sec_, plan_retry_backoff_sec_);
  private_nh_.param("replan_cooldown_sec", replan_cooldown_sec_, replan_cooldown_sec_);
  private_nh_.param("navigation_mode", navigation_mode_, navigation_mode_);
  private_nh_.param("periodic_replan_sec", periodic_replan_sec_, periodic_replan_sec_);
  private_nh_.param("route_segment_enable", route_segment_enable_, route_segment_enable_);
  private_nh_.param("route_segment_length_m", route_segment_length_m_, route_segment_length_m_);
  private_nh_.param("route_segment_overlap_m", route_segment_overlap_m_, route_segment_overlap_m_);
  private_nh_.param("recovery_shadow_enable", recovery_shadow_enable_, recovery_shadow_enable_);
  private_nh_.param("recovery_behavior_enable", recovery_behavior_enable_, recovery_behavior_enable_);
  private_nh_.param("recovery_candidate_max_age_sec", recovery_candidate_max_age_sec_, recovery_candidate_max_age_sec_);
  private_nh_.param("recovery_candidate_max_start_dist_m", recovery_candidate_max_start_dist_m_, recovery_candidate_max_start_dist_m_);
  private_nh_.param("nav_watchdog_timeout_sec", nav_watchdog_timeout_sec_, nav_watchdog_timeout_sec_);
  private_nh_.param("nav_stall_timeout_sec", nav_stall_timeout_sec_, nav_stall_timeout_sec_);
  private_nh_.param("alignment_watchdog_timeout_sec", alignment_watchdog_timeout_sec_, alignment_watchdog_timeout_sec_);
  private_nh_.param("handover_settle_sec", handover_settle_sec_, handover_settle_sec_);
  private_nh_.param("enable_alignment", enable_alignment_, enable_alignment_);
  // Dynamic target following mode (disabled by default).
  private_nh_.param("enable_dynamic_mode", enable_dynamic_mode_, enable_dynamic_mode_);
  private_nh_.param("dynamic_replan_cooldown_sec", dynamic_replan_cooldown_sec_,
                    dynamic_replan_cooldown_sec_);
  private_nh_.param("dynamic_goal_topic", dynamic_goal_topic_, dynamic_goal_topic_);
  // Target-loss in-place rotation search (substate of DYNAMIC, disabled by default).
  private_nh_.param("enable_follow_search", enable_follow_search_, enable_follow_search_);
  private_nh_.param("follow_search_clearance_m", follow_search_clearance_m_,
                    follow_search_clearance_m_);
  private_nh_.param("follow_search_step_deg", follow_search_step_deg_,
                    follow_search_step_deg_);
  private_nh_.param("follow_search_step_pause_sec", follow_search_step_pause_sec_,
                    follow_search_step_pause_sec_);
  private_nh_.param("follow_search_timeout_sec", follow_search_timeout_sec_,
                    follow_search_timeout_sec_);
  private_nh_.param("follow_search_total_limit_deg", follow_search_total_limit_deg_,
                    follow_search_total_limit_deg_);
  private_nh_.param("follow_search_turn_service", follow_search_turn_service_,
                    follow_search_turn_service_);
  private_nh_.param("follow_search_scan_topic", follow_search_scan_topic_,
                    follow_search_scan_topic_);

  // Periodic replan install gate parameters.
  private_nh_.param("install_gate_enable", install_gate_enable_, install_gate_enable_);
  private_nh_.param("install_gate_shadow", install_gate_shadow_, install_gate_shadow_);
  private_nh_.param("install_gate_blocked_half_width_m",
                    install_gate_cfg_.blocked_half_width_m,
                    install_gate_cfg_.blocked_half_width_m);
  private_nh_.param("install_gate_blocked_forward_m",
                    install_gate_cfg_.blocked_forward_m,
                    install_gate_cfg_.blocked_forward_m);
  private_nh_.param("install_gate_blocked_enter_sec",
                    install_gate_cfg_.blocked_enter_sec,
                    install_gate_cfg_.blocked_enter_sec);
  private_nh_.param("install_gate_blocked_exit_sec",
                    install_gate_cfg_.blocked_exit_sec,
                    install_gate_cfg_.blocked_exit_sec);
  private_nh_.param("install_gate_scan_stride", install_gate_cfg_.scan_stride,
                    install_gate_cfg_.scan_stride);
  private_nh_.param("install_gate_side_arc_m", install_gate_cfg_.side_arc_m,
                    install_gate_cfg_.side_arc_m);
  private_nh_.param("install_gate_side_deadband_m",
                    install_gate_cfg_.side_deadband_m,
                    install_gate_cfg_.side_deadband_m);
  private_nh_.param("install_gate_heading_worsen_deg",
                    install_gate_cfg_.heading_worsen_deg,
                    install_gate_cfg_.heading_worsen_deg);
  private_nh_.param("install_gate_improve_length_ratio",
                    install_gate_cfg_.improve_length_ratio,
                    install_gate_cfg_.improve_length_ratio);
  private_nh_.param("install_gate_improve_kink_deg",
                    install_gate_cfg_.improve_kink_deg,
                    install_gate_cfg_.improve_kink_deg);
  private_nh_.param("install_gate_improve_clearance_m",
                    install_gate_cfg_.improve_clearance_m,
                    install_gate_cfg_.improve_clearance_m);
  private_nh_.param("install_gate_changed_lateral_m",
                    install_gate_cfg_.changed_lateral_m,
                    install_gate_cfg_.changed_lateral_m);
  private_nh_.param("install_gate_changed_length_ratio",
                    install_gate_cfg_.changed_length_ratio,
                    install_gate_cfg_.changed_length_ratio);
  private_nh_.param("install_gate_changed_kink_deg",
                    install_gate_cfg_.changed_kink_deg,
                    install_gate_cfg_.changed_kink_deg);
  private_nh_.param("install_gate_metric_window_m",
                    install_gate_cfg_.metric_window_m,
                    install_gate_cfg_.metric_window_m);
  private_nh_.param("install_gate_alignment_skip_when_blocked",
                    install_gate_cfg_.alignment_reject_skip_when_blocked,
                    install_gate_cfg_.alignment_reject_skip_when_blocked);
  if (navigation_mode_ != "escape" && navigation_mode_ != "replan_periodic" &&
      navigation_mode_ != "neupan_only") {
    ROS_ERROR("Invalid navigation_mode=%s; falling back to escape",
              navigation_mode_.c_str());
    navigation_mode_ = "escape";
  }
  if (navigation_mode_ != "escape") {
    recovery_shadow_enable_ = false;
    recovery_behavior_enable_ = false;
  }
  // Local planner backend and interface contract (action | goal_pose)
  private_nh_.param("local_planner", local_planner_, local_planner_);
  private_nh_.param("local_planner_mode", local_planner_mode_, local_planner_mode_);
  if (local_planner_mode_.empty()) {
    local_planner_mode_ =
        (local_planner_ == "sru" || local_planner_ == "goal_pose") ? "goal_pose" : "action";
  }
  if (local_planner_mode_ != "action" && local_planner_mode_ != "goal_pose") {
    ROS_ERROR("Invalid local_planner_mode=%s; falling back to action",
              local_planner_mode_.c_str());
    local_planner_mode_ = "action";
  }
  private_nh_.param("sru_arrive_threshold_m", sru_arrive_threshold_m_,
                    sru_arrive_threshold_m_);
  private_nh_.param("sru_goal_refresh_sec", sru_goal_refresh_sec_,
                    sru_goal_refresh_sec_);
  private_nh_.param("sru_goal_refresh_min_delta_m", sru_goal_refresh_min_delta_m_,
                    sru_goal_refresh_min_delta_m_);
  private_nh_.param("sru_cmd_vel_silence_sec", sru_cmd_vel_silence_sec_,
                    sru_cmd_vel_silence_sec_);
  private_nh_.param("sru_stall_progress_epsilon_m", sru_stall_progress_epsilon_m_,
                    sru_stall_progress_epsilon_m_);
  private_nh_.param("sru_robot_frame", sru_robot_frame_, sru_robot_frame_);
  if (local_planner_mode_ == "goal_pose") {
    recovery_shadow_enable_ = false;
    recovery_behavior_enable_ = false;
    install_gate_enable_ = false;
    install_gate_shadow_ = false;
    route_segment_enable_ = false;
    if (enable_dynamic_mode_) {
      ROS_WARN("[sru] enable_dynamic_mode=true is incompatible with goal_pose planner (%s); "
               "VLM follow mode disabled", local_planner_.c_str());
      enable_dynamic_mode_ = false;
    }
    if (sru_arrive_threshold_m_ < 0.6) {
      ROS_WARN("[sru] sru_arrive_threshold_m=%.2f < SRU internal 0.6m threshold; "
               "forcing to 0.7", sru_arrive_threshold_m_);
      sru_arrive_threshold_m_ = 0.7;
    }
  }
  if (periodic_replan_sec_ <= 0.0) periodic_replan_sec_ = 2.0;
  if (route_segment_length_m_ <= 0.0) route_segment_length_m_ = 10.0;
  route_segment_overlap_m_ = std::max(
      0.0, std::min(route_segment_overlap_m_, route_segment_length_m_ * 0.5));

  status_pub_ = nh_.advertise<nav_interfaces::MissionStatus>(
      "/waypoint_manager/status", 1, true);
  event_pub_ = nh_.advertise<nav_interfaces::MissionEvent>(
      "/waypoint_manager/mission_feedback", 10);
  operator_command_pub_ = nh_.advertise<nav_interfaces::OperatorCommand>(
      "/waypoint_manager/operator_command", 10);
  stop_pub_ = nh_.advertise<std_msgs::Empty>(stop_topic_, 1);
  global_path_pub_ = nh_.advertise<nav_msgs::Path>(
      "/waypoint_mission/global_path_vis", 1, true);
  recovery_command_pub_ = nh_.advertise<nav_interfaces::RecoveryCommand>(
      "/neupan/recovery_command", 1);
  hot_path_pub_ = nh_.advertise<std_msgs::Bool>("/neupan/hot_path_enable", 1, true);

  // Goal-pose backend publishers and subscribers (created when local_planner_mode=goal_pose)
  if (local_planner_mode_ == "goal_pose") {
    sru_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(sru_goal_topic_, 1);
    sru_enable_pub_ = nh_.advertise<std_msgs::Bool>(sru_enable_topic_, 1, true);
    sru_cmd_vel_sub_ = nh_.subscribe(sru_cmd_vel_topic_, 1,
                                     &MissionManager::sruCmdVelCb, this);
    std_msgs::Bool sru_enable;
    sru_enable.data = false;
    sru_enable_pub_.publish(sru_enable);
    ROS_WARN("[local_planner] %s (goal_pose mode) ENABLED: global planning is "
             "NOT used; waypoints -> %s, enable -> %s, cmd_vel <- %s; arrive_threshold=%.2fm",
             local_planner_.c_str(), sru_goal_topic_.c_str(),
             sru_enable_topic_.c_str(), sru_cmd_vel_topic_.c_str(),
             sru_arrive_threshold_m_);
  }

  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                            &MissionManager::goalCb, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 1, &MissionManager::odomCb, this);
  replan_sub_ = nh_.subscribe("/neupan/replan", 1,
                              &MissionManager::replanCb, this);
  recovery_request_sub_ = nh_.subscribe(
      "/neupan/recovery_request", 1, &MissionManager::recoveryRequestCb, this);

  // Install gate hysteresis, shadow publisher, and scan subscriber
  install_gate_blocked_.reset(new mission_manager_gate::BlockedHysteresis(
      install_gate_cfg_.blocked_enter_sec, install_gate_cfg_.blocked_exit_sec));
  if (install_gate_shadow_ || install_gate_enable_) {
    install_gate_shadow_pub_ = nh_.advertise<std_msgs::String>(
        "/neupan/replan_gate_shadow", 10);
    if (!install_gate_scan_subscribed_) {
      scan360_sub_ = nh_.subscribe(follow_search_scan_topic_, 1,
                                   &MissionManager::scan360Cb, this);
      install_gate_scan_subscribed_ = true;
    }
    ROS_INFO("[install_gate] %s (shadow=%d enable=%d): half_width=%.3fm "
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
    ROS_INFO("[install_gate] disabled (install_gate_enable=false, "
             "install_gate_shadow=false)");
  }

  start_mission_srv_ = nh_.advertiseService(
      "/waypoint_manager/start_mission", &MissionManager::startMissionCb, this);
  pause_srv_ = nh_.advertiseService("/waypoint_manager/pause",
                                    &MissionManager::pauseCb, this);
  resume_srv_ = nh_.advertiseService("/waypoint_manager/resume",
                                     &MissionManager::resumeCb, this);
  return_home_srv_ = nh_.advertiseService(
      "/waypoint_manager/return_home", &MissionManager::returnHomeCb, this);
  cancel_srv_ = nh_.advertiseService("/waypoint_manager/cancel",
                                     &MissionManager::cancelCb, this);
  reset_srv_ = nh_.advertiseService("/waypoint_manager/reset",
                                    &MissionManager::resetCb, this);

  std_msgs::Bool hot_path;
  hot_path.data = navigation_mode_ == "replan_periodic";
  hot_path_pub_.publish(hot_path);
  ROS_INFO("[navigation_mode] mode=%s periodic=%.3fs hot_path=%s recovery=%s",
           navigation_mode_.c_str(), periodic_replan_sec_,
           hot_path.data ? "true" : "false",
           recovery_behavior_enable_ ? "true" : "false");
  if (enable_dynamic_mode_) {
    set_dynamic_mode_srv_ = nh_.advertiseService(
        "/waypoint_manager/set_dynamic_mode", &MissionManager::setDynamicModeCb, this);
    dynamic_goal_sub_ = nh_.subscribe(dynamic_goal_topic_, 1,
                                      &MissionManager::dynamicGoalCb, this);
    ROS_INFO("[mission_manager] dynamic (follow) mode ENABLED: goal topic=%s, cooldown=%.2fs",
             dynamic_goal_topic_.c_str(), dynamic_replan_cooldown_sec_);
    if (enable_follow_search_) {
      follow_search_srv_ = nh_.advertiseService(
          "/waypoint_manager/follow_search", &MissionManager::followSearchCb, this);
      scan360_sub_ = nh_.subscribe(follow_search_scan_topic_, 1,
                                   &MissionManager::scan360Cb, this);
      install_gate_scan_subscribed_ = true;
      turn_client_ = nh_.serviceClient<atomic_move::Turn>(follow_search_turn_service_);
      ROS_INFO("[mission_manager] follow search ENABLED: step=%.0fdeg pause=%.2fs "
               "clearance=%.2fm turn=%s scan=%s (limits: total=%.0fdeg timeout=%.0fs)",
               follow_search_step_deg_, follow_search_step_pause_sec_,
               follow_search_clearance_m_, follow_search_turn_service_.c_str(),
               follow_search_scan_topic_.c_str(), follow_search_total_limit_deg_,
               follow_search_timeout_sec_);
    } else {
      ROS_INFO("[mission_manager] follow search disabled (enable_follow_search=false)");
    }
  } else {
    ROS_INFO("[mission_manager] dynamic (follow) mode disabled (enable_dynamic_mode=false)");
  }

  plan_client_ = nh_.serviceClient<nav_interfaces::PlanPath>(plan_service_name_);
  sequence_plan_client_ = nh_.serviceClient<nav_interfaces::PlanSequence>(
      "/map_planner/plan_sequence");
  align_client_ = nh_.serviceClient<atomic_move::AlignTo>("/atomic_move/align");
  path_client_.reset(new NavPathClient(nav_path_action_name_, true));
  status_timer_ = nh_.createTimer(ros::Duration(0.5),
                                  &MissionManager::statusTimerCb, this);
  publishStatus();
}

MissionManager::~MissionManager() {
  alignment_cancel_requested_.store(true);
  search_cancel_requested_.store(true);
  stopNavigation();
  if (hot_path_pub_) {
    std_msgs::Bool hot_off;
    hot_off.data = false;
    hot_path_pub_.publish(hot_off);
  }
  joinAlignmentThread();
  joinSearchThread();
  if (recovery_shadow_future_.valid()) {
    recovery_shadow_future_.wait();
  }
  if (periodic_replan_future_.valid()) {
    periodic_replan_future_.wait();
  }
}

void MissionManager::process() {
  // Report perception/odometry silence (>2s) via EC_TRACE
  {
    const ros::Time now = ros::Time::now();
    if (!last_odom_rx_at_.isZero() && (now - last_odom_rx_at_).toSec() > 2.0) {
      if (!ec_perception_offline_) {
        ec_perception_offline_ = true;
        EC_TRACE(ec::DEVICE_OFFLINE, "perception", "odom silent %.1fs (last_rx=%.3f)",
                 (now - last_odom_rx_at_).toSec(), last_odom_rx_at_.toSec());
      }
    } else if (ec_perception_offline_ && !last_odom_rx_at_.isZero()) {
      ec_perception_offline_ = false;
      EC_TRACE(ec::DEVICE_OFFLINE, "perception", "odom recovered");
    }
  }
  processRecoveryShadow();
  if (state_ == State::DYNAMIC || state_ == State::DYNAMIC_SEARCHING) {
    processDynamicMode();
    return;
  }
  maybeAdvanceRouteSegment();
  processPeriodicReplan();

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
    ROS_WARN("NeuPAN requested a new global path for waypoint %d", current_index_);
    if (path_client_) {
      path_client_->cancelGoal();
      path_client_->waitForResult(ros::Duration(plan_timeout_sec_));
    }
    stop_pub_.publish(std_msgs::Empty());
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
      const ros::Time now = ros::Time::now();
      if (nav_watchdog_timeout_sec_ > 0.0 &&
          !last_path_feedback_at_.isZero() &&
          (now - last_path_feedback_at_).toSec() > nav_watchdog_timeout_sec_) {
        clearRecoveryCandidate(true);
        stopNavigation();
        skipCurrentWaypointOrFail("NeuPAN feedback watchdog expired during recovery");
      }
      return;
    }
    if (!path_done_.load()) {
      const ros::Time now = ros::Time::now();
      if (nav_watchdog_timeout_sec_ > 0.0 &&
          !last_path_feedback_at_.isZero() &&
          (now - last_path_feedback_at_).toSec() > nav_watchdog_timeout_sec_) {
        ROS_ERROR("NeuPAN action watchdog expired in state %s after %.3f s",
                  stateName(state_).c_str(),
                  (now - last_path_feedback_at_).toSec());
        EC_TRACE(ec::NEUPAN_FEEDBACK_FROZEN, "watchdog", "state=%s elapsed=%.3fs cause=action_feedback",
                 stateName(state_).c_str(), (now - last_path_feedback_at_).toSec());
        stopNavigation();
        skipCurrentWaypointOrFail("NeuPAN action feedback watchdog expired");
        return;
      }
      if (nav_stall_timeout_sec_ > 0.0 &&
          !last_feedback_progress_at_.isZero() &&
          (now - last_feedback_progress_at_).toSec() > nav_stall_timeout_sec_) {
        ROS_ERROR("[neupan_node] NeuPAN feedback appears frozen in state %s "
                  "(distance_remaining=%.3f unchanged for %.3f s); the NeuPAN "
                  "control loop may have stalled or crashed internally, "
                  "manual restart of neupan_node may be required",
                  stateName(state_).c_str(), last_feedback_distance_,
                  (now - last_feedback_progress_at_).toSec());
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
      !alignment_started_at_.isZero() &&
      (ros::Time::now() - alignment_started_at_).toSec() > alignment_watchdog_timeout_sec_) {
    ROS_ERROR("Alignment watchdog expired after %.3f s",
              (ros::Time::now() - alignment_started_at_).toSec());
    stopNavigation();
    skipCurrentWaypointOrFail("Alignment watchdog expired");
    return;
  }

  if (state_ == State::ARRIVED) {
    if (enable_alignment_) {
      stopNavigation();
      alignment_started_at_ = ros::Time::now();
      transitionTo(State::ALIGNING);
      startAlignment();
      return;
    }
    beginDwell();
    return;
  }

  if (state_ == State::DWELLING) {
    dwell_remaining_ = std::max(0.0, dwell_duration_ -
                                      (ros::Time::now() - dwell_start_).toSec());
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

bool MissionManager::startMissionCb(nav_interfaces::StartMission::Request &req,
                                     nav_interfaces::StartMission::Response &res) {
  const std::string command_id = newCommandId();
  const std::string payload = "{\"waypoint_count\":" +
                              std::to_string(req.waypoints.size()) +
                              ",\"loop\":" + (req.loop ? "true" : "false") + "}";
  publishOperatorCommand(command_id, "START_MISSION", "REQUESTED", payload, false, "");
  const bool temporary = req.temporary;
  const bool normal_state = state_ == State::IDLE || state_ == State::COMPLETED ||
                            state_ == State::CANCELLED || state_ == State::FAILED;
  if ((!normal_state && !temporary) || (temporary && state_ != State::PAUSED)) {
    res.success = false;
    res.message = temporary ? "Temporary mission requires PAUSED state"
                            : "Mission manager is busy in state " + stateName(state_);
    publishOperatorCommand(command_id, "START_MISSION", "REJECTED", payload, false,
                           res.message);
    return true;
  }
  if (req.waypoints.empty()) {
    res.success = false;
    res.message = "Waypoint list is empty";
    publishOperatorCommand(command_id, "START_MISSION", "REJECTED", payload, false,
                           res.message);
    return true;
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

  waypoints_ = req.waypoints;
  loop_ = req.loop;
  returning_home_ = false;
  current_index_ = 0;
  completed_waypoints_ = 0;
  mission_id_ = newMissionId();
  last_replan_at_ = ros::Time(0);
  if (!temporary) {
    captureHomePoseIfNeeded();
  }

  res.success = startCurrentWaypoint();
  res.mission_id = mission_id_;
  res.message = res.success ? (temporary ? "Temporary mission started" : "Mission started")
                            : "Unable to start first waypoint";
  if (res.success && !temporary) {
    publishGlobalPathVisualization();
  }
  publishOperatorCommand(command_id, "START_MISSION",
                         res.success ? "ACCEPTED" : "REJECTED", payload,
                         res.success, res.message);
  return true;
}

bool MissionManager::pauseCb(std_srvs::Trigger::Request &,
                             std_srvs::Trigger::Response &res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "PAUSE", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res.success = false;
    res.message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "PAUSE", "REJECTED", "{}", false, res.message);
    return true;
  }
  if (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME &&
      state_ != State::DWELLING) {
    res.success = false;
    res.message = "Mission is not active";
    publishOperatorCommand(command_id, "PAUSE", "REJECTED", "{}", false, res.message);
    return true;
  }
  clearRecoveryCandidate(true);
  paused_from_ = state_;
  stopNavigation();
  transitionTo(State::PAUSED, "Paused by request");
  res.success = true;
  res.message = "Mission paused";
  publishOperatorCommand(command_id, "PAUSE", "ACCEPTED", "{}", true, res.message);
  return true;
}

bool MissionManager::resumeCb(std_srvs::Trigger::Request &,
                              std_srvs::Trigger::Response &res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "RESUME", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res.success = false;
    res.message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "RESUME", "REJECTED", "{}", false, res.message);
    return true;
  }
  if (state_ != State::PAUSED) {
    res.success = false;
    res.message = "Mission is not paused";
    publishOperatorCommand(command_id, "RESUME", "REJECTED", "{}", false, res.message);
    return true;
  }
  if (paused_from_ == State::DWELLING) {
    dwell_start_ = ros::Time::now();
    transitionTo(State::DWELLING, "Resumed dwell");
    res.success = true;
    res.message = "Dwell resumed";
    publishOperatorCommand(command_id, "RESUME", "ACCEPTED", "{}", true, res.message);
    return true;
  }
  res.success = startCurrentWaypoint();
  res.message = res.success ? "Mission resumed" : "Unable to resume waypoint";
  publishOperatorCommand(command_id, "RESUME",
                         res.success ? "ACCEPTED" : "REJECTED", "{}",
                         res.success, res.message);
  return true;
}

bool MissionManager::returnHomeCb(std_srvs::Trigger::Request &,
                                  std_srvs::Trigger::Response &res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "RETURN_HOME", "REQUESTED", "{}", false, "");
  if (dynamic_mode_) {
    res.success = false;
    res.message = "In dynamic (follow) mode; exit via set_dynamic_mode(false) first";
    publishOperatorCommand(command_id, "RETURN_HOME", "REJECTED", "{}", false,
                           res.message);
    return true;
  }
  if (!home_pose_valid_) {
    res.success = false;
    res.message = "Home pose is not available";
    publishOperatorCommand(command_id, "RETURN_HOME", "REJECTED", "{}", false,
                           res.message);
    return true;
  }
  stopNavigation();
  nav_interfaces::Waypoint home;
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
  res.success = startCurrentWaypoint();
  res.message = res.success ? "Returning home" : "Unable to plan home path";
  publishOperatorCommand(command_id, "RETURN_HOME",
                         res.success ? "ACCEPTED" : "REJECTED", "{}",
                         res.success, res.message);
  return true;
}

bool MissionManager::cancelCb(std_srvs::Trigger::Request &,
                              std_srvs::Trigger::Response &res) {
  const std::string command_id = newCommandId();
  publishOperatorCommand(command_id, "CANCEL", "REQUESTED", "{}", false, "");
  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  if (dynamic_mode_) {
    exitDynamicMode("Dynamic mode exited by cancel");
    transitionTo(State::CANCELLED, "Mission cancelled (dynamic mode exited)");
    res.success = true;
    res.message = "Dynamic mode exited and cancelled";
    publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res.message);
    return true;
  }
  if (temporary_mission_) {
    restorePausedMission("Temporary mission cancelled; main mission remains paused");
    res.success = true;
    res.message = "Temporary mission cancelled; main mission remains paused";
    publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res.message);
    return true;
  }
  stopNavigation();
  waypoints_.clear();
  current_index_ = -1;
  loop_ = false;
  returning_home_ = false;
  mission_id_.clear();
  last_replan_at_ = ros::Time(0);
  transitionTo(State::CANCELLED, "Mission cancelled");
  res.success = true;
  res.message = "Mission cancelled";
  publishOperatorCommand(command_id, "CANCEL", "ACCEPTED", "{}", true, res.message);
  return true;
}

bool MissionManager::resetCb(std_srvs::Trigger::Request &,
                             std_srvs::Trigger::Response &res) {
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
  last_replan_at_ = ros::Time(0);
  dwell_duration_ = 0.0;
  dwell_remaining_ = 0.0;
  alignment_started_at_ = ros::Time(0);
  transitionTo(State::IDLE, "Mission manager reset");
  res.success = true;
  res.message = "Mission manager reset";
  publishOperatorCommand(command_id, "RESET", "ACCEPTED", "{}", true, res.message);
  return true;
}

void MissionManager::goalCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  if (state_ != State::IDLE && state_ != State::COMPLETED &&
      state_ != State::CANCELLED && state_ != State::FAILED) {
    ROS_WARN_THROTTLE(2.0, "Ignoring single goal while mission is active");
    return;
  }
  clearRecoveryCandidate(true);
  active_path_.poses.clear();
  clearRouteSegment();
  geometry_msgs::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_frame_;
  }
  if (goal.header.frame_id != map_frame_) {
    try {
      geometry_msgs::PoseStamped transformed;
      tf_buffer_.transform(goal, transformed, map_frame_, ros::Duration(0.2));
      goal = transformed;
    } catch (const tf2::TransformException &ex) {
      ROS_WARN("Single goal transform failed: %s", ex.what());
      return;
    }
  }
  nav_interfaces::Waypoint single;
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

// Dynamic target following mode

bool MissionManager::setDynamicModeCb(std_srvs::SetBool::Request &req,
                                      std_srvs::SetBool::Response &res) {
  const std::string command_id = newCommandId();
  const std::string command = req.data ? "ENTER_DYNAMIC_MODE" : "EXIT_DYNAMIC_MODE";
  publishOperatorCommand(command_id, command, "REQUESTED", "{}", false, "");

  if (req.data) {
    if (dynamic_mode_) {
      res.success = true;
      res.message = "Already in dynamic mode";
      publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res.message);
      return true;
    }
    // Entering dynamic mode preempts and discards any active or paused mission.
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
    last_dynamic_plan_at_ = ros::Time(0);
    std_msgs::Bool hot_on;
    hot_on.data = true;
    hot_path_pub_.publish(hot_on);
    transitionTo(State::DYNAMIC,
                 "Dynamic (follow) mode entered; previous activity discarded");
    res.success = true;
    res.message = "Dynamic mode entered; publish goals to " + dynamic_goal_topic_;
    publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res.message);
    return true;
  }

  if (!dynamic_mode_) {
    res.success = true;
    res.message = "Not in dynamic mode";
    publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res.message);
    return true;
  }
  exitDynamicMode("Dynamic mode exited by request; holding position in IDLE");
  res.success = true;
  res.message = "Dynamic mode exited; state=IDLE, holding position";
  publishOperatorCommand(command_id, command, "ACCEPTED", "{}", true, res.message);
  return true;
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
    std_msgs::Bool hot_off;
    hot_off.data = false;
    hot_path_pub_.publish(hot_off);
  }
  transitionTo(State::IDLE, message);
}

void MissionManager::dynamicGoalCb(const geometry_msgs::PoseStampedConstPtr &msg) {
  // Receiving a dynamic goal while in DYNAMIC_SEARCHING interrupts the rotation search.
  if (!dynamic_mode_ ||
      (state_ != State::DYNAMIC && state_ != State::DYNAMIC_SEARCHING)) {
    ROS_WARN_THROTTLE(2.0, "Ignoring dynamic goal: not in dynamic mode "
                           "(call /waypoint_manager/set_dynamic_mode true first)");
    return;
  }
  geometry_msgs::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_frame_;
  }
  if (goal.header.frame_id != map_frame_) {
    try {
      geometry_msgs::PoseStamped transformed;
      tf_buffer_.transform(goal, transformed, map_frame_, ros::Duration(0.2));
      goal = transformed;
    } catch (const tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(2.0, "Dynamic goal transform failed (%s); dropping this point",
                        ex.what());
      return;
    }
  }
  if (search_running_.load() || state_ == State::DYNAMIC_SEARCHING) {
    ROS_INFO("[follow_search] target reacquired (new dynamic goal) -> stopping search");
    cancelFollowSearch("dynamic_goal_received");
  }
  dynamic_goal_ = goal;
  dynamic_goal_valid_ = true;
  dynamic_goal_pending_ = true;  // latest-wins
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
        ROS_WARN("[follow_search] stopped: %s", text.c_str());
        transitionTo(State::DYNAMIC, "Follow search stopped: " + text);
        publishEvent("FOLLOW_SEARCH_STOPPED", text);
      }
    }
    return;
  }
  if (state_ == State::DYNAMIC_SEARCHING) {
    return;
  }

  if (dynamic_nav_active_ && path_done_.load() &&
      path_client_->getState().isDone()) {
    dynamic_nav_active_ = false;
    if (path_succeeded_.load()) {
      publishEvent("DYNAMIC_GOAL_REACHED", "Holding position at last dynamic goal");
    } else {
      ROS_WARN_THROTTLE(2.0,
                        "[dynamic] navPath action ended without success; holding "
                        "position and waiting for the next dynamic goal");
    }
  }

  if (!dynamic_goal_pending_) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if (!last_dynamic_plan_at_.isZero() &&
      (now - last_dynamic_plan_at_).toSec() < dynamic_replan_cooldown_sec_) {
    return;
  }
  dynamic_goal_pending_ = false;
  last_dynamic_plan_at_ = now;
  startDynamicGoal();
}

bool MissionManager::startDynamicGoal() {
  if (!plan_client_.waitForExistence(ros::Duration(plan_timeout_sec_))) {
    ROS_WARN_THROTTLE(2.0, "[dynamic] plan service unavailable; holding current state");
    return false;
  }
  nav_interfaces::PlanPath service;
  service.request.goal = dynamic_goal_;
  if (!plan_client_.call(service) || !service.response.success ||
      service.response.path.poses.empty()) {
    ++dynamic_plan_failures_;
    ROS_WARN_THROTTLE(2.0, "[dynamic] plan failed (%s); holding (consecutive=%d)",
                      service.response.message.c_str(), dynamic_plan_failures_);
    return false;
  }
  dynamic_plan_failures_ = 0;

  if (!path_client_->waitForServer(ros::Duration(plan_timeout_sec_))) {
    ROS_WARN_THROTTLE(2.0, "[dynamic] NeuPAN action server unavailable; holding");
    return false;
  }
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = ros::Time::now();
  last_path_feedback_at_ = path_started_at_;
  last_feedback_progress_at_ = ros::Time();
  last_feedback_distance_ = -1.0;
  nav_interfaces::navPathGoal action_goal;
  action_goal.path = service.response.path;
  const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
  path_client_->sendGoal(action_goal,
                         boost::bind(&MissionManager::pathDoneCb, this, generation,
                                     boost::placeholders::_1, boost::placeholders::_2),
                         boost::bind(&MissionManager::pathActiveCb, this, generation),
                         boost::bind(&MissionManager::pathFeedbackCb, this, generation,
                                     boost::placeholders::_1));
  dynamic_nav_active_ = true;
  return true;
}

// Target-loss rotation search (substate of DYNAMIC)

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

void MissionManager::scan360Cb(const sensor_msgs::LaserScanConstPtr &msg) {
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

  // Transform subsampled scan points into map_frame_ for the install gate
  if (install_gate_shadow_ || install_gate_enable_) {
    install_gate_scan_map_pts_.clear();
    install_gate_scan_ready_ = false;
    geometry_msgs::TransformStamped tf;
    bool tf_ok = false;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                      msg->header.stamp, ros::Duration(0.05));
      tf_ok = true;
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                        ros::Time(0), ros::Duration(0.05));
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
      install_gate_scan_stamp_ = msg->header.stamp;
      install_gate_scan_ready_ = !install_gate_scan_map_pts_.empty();
    }
    updateInstallGateBlocked();
  }
}

// Install gate state reset and blocked evaluation

void MissionManager::resetInstallGateState(const std::string &why) {
  if (!install_gate_blocked_) return;
  install_gate_blocked_->reset();
  install_gate_blocked_raw_.store(false);
  install_gate_blocked_gated_.store(false);
  install_gate_min_lateral_m_.store(-1.0);
  install_gate_blocked_dwell_sec_ = 0.0;
  if (install_gate_shadow_ || install_gate_enable_) {
    ROS_INFO("[install_gate] state reset (%s)", why.c_str());
  }
}

std::vector<mission_manager_gate::Pt> MissionManager::pathToPoints(
    const nav_msgs::Path &path) const {
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
    geometry_msgs::PoseStamped pose;
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

bool MissionManager::followSearchCb(nav_interfaces::FollowSearch::Request &req,
                                    nav_interfaces::FollowSearch::Response &res) {
  res.accepted = false;
  if (!enable_follow_search_) {
    res.message = "follow search disabled (enable_follow_search=false)";
    return true;
  }
  if (!dynamic_mode_ ||
      (state_ != State::DYNAMIC && state_ != State::DYNAMIC_SEARCHING)) {
    res.message = "not in dynamic mode";
    return true;
  }
  if (req.direction > -0.5 && req.direction < 0.5) {
    res.message = "direction must be +1 (left/ccw) or -1 (right/cw)";
    return true;
  }
  if (!scan360_received_.load()) {
    res.message = "no " + follow_search_scan_topic_ +
                  " data yet; refusing to rotate blind";
    ROS_WARN("[follow_search] rejected: %s", res.message.c_str());
    return true;
  }

  const double direction = req.direction > 0.0 ? 1.0 : -1.0;
  const double step_deg =
      req.step_deg > 0.0 ? req.step_deg : follow_search_step_deg_;
  const double pause_sec = req.step_pause_sec > 0.0 ? req.step_pause_sec
                                                    : follow_search_step_pause_sec_;
  const double total_limit = req.total_limit_deg > 0.0 ? req.total_limit_deg
                                                       : follow_search_total_limit_deg_;
  const double timeout_sec = req.timeout_sec > 0.0 ? req.timeout_sec
                                                   : follow_search_timeout_sec_;
  const double clearance_m = req.min_clearance_m > 0.0 ? req.min_clearance_m
                                                       : follow_search_clearance_m_;

  stopNavigation();
  dynamic_goal_pending_ = false;
  transitionTo(State::DYNAMIC_SEARCHING, "Follow search started");
  startFollowSearch(direction, step_deg, pause_sec, total_limit, timeout_sec,
                    clearance_m);
  ROS_INFO("[follow_search] accepted: dir=%+.0f step=%.0fdeg pause=%.2fs "
           "total_limit=%.0fdeg timeout=%.0fs clearance=%.2fm bearing=%.3frad",
           direction, step_deg, pause_sec, total_limit, timeout_sec, clearance_m,
           req.last_bearing_rad);
  res.accepted = true;
  res.message = "search started";
  return true;
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
  stop_pub_.publish(std_msgs::Empty());
  ROS_INFO("[follow_search] cancel requested: %s", reason.c_str());
}

void MissionManager::followSearchLoop(double direction, double step_deg,
                                      double pause_sec, double total_limit_deg,
                                      double timeout_sec, double clearance_m,
                                      std::uint64_t generation) {
  const std::string tag = "[follow_search]";
  SearchStop stop_code = SearchStop::CANCELLED;

  // Allow atomic_move to consume the /neupan/stop published by stopNavigation()
  // before issuing the first /atomic_move/turn request.
  ros::Duration(handover_settle_sec_).sleep();
  if (generation != search_generation_.load() || search_cancel_requested_.load()) {
    search_stop_code_.store(static_cast<int>(SearchStop::CANCELLED));
    return;
  }

  double accumulated_deg = 0.0;
  int segments = 0;
  const ros::WallTime started = ros::WallTime::now();

  while (ros::ok()) {
    if (generation != search_generation_.load()) {
      stop_code = SearchStop::SUPERSEDED;
      break;
    }
    if (search_cancel_requested_.load()) {
      stop_code = SearchStop::CANCELLED;
      break;
    }
    if (timeout_sec > 0.0 && (ros::WallTime::now() - started).toSec() > timeout_sec) {
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
      ROS_WARN("%s blocked: clearance=%.2fm < %.2fm; stopping search", tag.c_str(),
               clearance, clearance_m);
      stop_code = SearchStop::BLOCKED;
      break;
    }

    if (!turn_client_.waitForExistence(ros::Duration(1.0))) {
      ROS_WARN("%s turn service %s unavailable", tag.c_str(),
               follow_search_turn_service_.c_str());
      stop_code = SearchStop::TURN_FAILED;
      break;
    }
    atomic_move::Turn turn;
    turn.request.angle = static_cast<float>(step_deg * direction);
    if (!turn_client_.call(turn)) {
      ROS_WARN("%s turn call failed", tag.c_str());
      stop_code = SearchStop::TURN_FAILED;
      break;
    }
    if (!turn.response.success) {
      if (turn.response.message == "TURN_CANCELLED") {
        stop_code = SearchStop::CANCELLED;
        break;
      }
      ROS_WARN("%s turn failed: %s", tag.c_str(), turn.response.message.c_str());
      stop_code = SearchStop::TURN_FAILED;
      break;
    }
    accumulated_deg += step_deg;
    ++segments;
    ROS_INFO("%s segment %d done: this=%.0fdeg total=%.0fdeg clearance=%.2fm",
             tag.c_str(), segments, step_deg, accumulated_deg, clearance);

    const ros::WallTime pause_end =
        ros::WallTime::now() + ros::WallDuration(pause_sec);
    while (ros::ok() && ros::WallTime::now() < pause_end) {
      if (generation != search_generation_.load() ||
          search_cancel_requested_.load()) {
        break;
      }
      ros::Duration(0.05).sleep();
    }
  }

  search_stop_code_.store(static_cast<int>(stop_code));
  ROS_INFO("%s loop finished: reason=%s segments=%d total=%.0fdeg elapsed=%.2fs",
           tag.c_str(), searchStopText(stop_code).c_str(), segments, accumulated_deg,
           (ros::WallTime::now() - started).toSec());
}

void MissionManager::clearRouteSegment() {
  global_route_.poses.clear();
  route_progress_index_ = 0;
  route_segment_anchor_index_ = 0;
  ++route_segment_generation_;
  route_segment_final_ = true;
}

nav_msgs::Path MissionManager::buildRouteSegment(
    const geometry_msgs::PoseStamped &pose) {
  nav_msgs::Path segment;
  if (!route_segment_enable_ || navigation_mode_ != "replan_periodic" ||
      global_route_.poses.size() < 2 ||
      global_route_.header.frame_id != map_frame_) {
    return segment;
  }
  const std::size_t last = global_route_.poses.size() - 1;
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
  std::size_t anchor = nearest;
  double segment_arc = 0.0;
  while (anchor < last && segment_arc < route_segment_length_m_) {
    const auto &a = global_route_.poses[anchor].pose.position;
    const auto &b = global_route_.poses[anchor + 1].pose.position;
    segment_arc += std::hypot(b.x - a.x, b.y - a.y);
    ++anchor;
  }
  if (anchor <= route_segment_anchor_index_ && !route_segment_final_) {
    return segment;
  }
  segment.header = global_route_.header;
  segment.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped start = pose;
  start.header = segment.header;
  segment.poses.push_back(start);
  const std::size_t route_begin = nearest_distance < 0.05 ? nearest + 1 : nearest;
  for (std::size_t i = route_begin; i <= anchor && i < global_route_.poses.size(); ++i) {
    geometry_msgs::PoseStamped point = global_route_.poses[i];
    point.header = segment.header;
    segment.poses.push_back(point);
  }
  if (segment.poses.size() < 2) return nav_msgs::Path();
  route_progress_index_ = nearest;
  route_segment_anchor_index_ = anchor;
  route_segment_final_ = anchor == last;
  ++route_segment_generation_;
  return segment;
}

void MissionManager::maybeAdvanceRouteSegment() {
  if (!route_segment_enable_ || navigation_mode_ != "replan_periodic" ||
      route_segment_final_ ||
      (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME)) {
    return;
  }
  const double remaining = distance_remaining_.load();
  if (!(remaining >= 0.0 && remaining <= route_segment_overlap_m_) &&
      !(path_done_.load() && path_succeeded_.load())) {
    return;
  }
  geometry_msgs::PoseStamped pose;
  if (!getRobotPose(pose)) return;
  nav_msgs::Path segment = buildRouteSegment(pose);
  if (segment.poses.size() < 2 || !path_client_ ||
      !path_client_->isServerConnected()) {
    return;
  }
  nav_interfaces::navPathGoal action_goal;
  action_goal.path = segment;
  active_path_ = segment;
  resetInstallGateState("route_segment");
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  last_periodic_replan_at_ = ros::Time::now();
  const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
  path_client_->sendGoal(
      action_goal,
      boost::bind(&MissionManager::pathDoneCb, this, generation,
                  boost::placeholders::_1, boost::placeholders::_2),
      boost::bind(&MissionManager::pathActiveCb, this, generation),
      boost::bind(&MissionManager::pathFeedbackCb, this, generation,
                  boost::placeholders::_1));
  ROS_INFO("[route_segment] advanced anchor=%zu/%zu poses=%zu final=%s",
           route_segment_anchor_index_, global_route_.poses.size() - 1,
           segment.poses.size(), route_segment_final_ ? "true" : "false");
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
        (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME)) {
      geometry_msgs::PoseStamped pose;
      nav_msgs::Path prepared;
      if (result.planned && getRobotPose(pose)) {
        prepared = preparePeriodicPath(result.path, pose);
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
        // Trim both active and candidate paths from the robot's current pose
        // forward by metric_window_m so forward motion does not skew length/deviation comparisons.
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
          std_msgs::String msg;
          msg.data = shadow.str();
          install_gate_shadow_pub_.publish(msg);
        }
        if (!gate.install) {
          ROS_WARN("[install_gate] %s (%s) waypoint=%d shadow=%d",
                   gate.reason.c_str(), gate.detail.c_str(), current_index_,
                   install_gate_shadow_ ? 1 : 0);
        } else {
          ROS_INFO("[install_gate] %s (%s) waypoint=%d", gate.reason.c_str(),
                   gate.detail.c_str(), current_index_);
        }
      }

      const bool gate_block_install = install_gate_enable_ && !gate.install;
      if (!gate_block_install && prepared.poses.size() >= 2 && path_client_ &&
          path_client_->isServerConnected()) {
        nav_interfaces::navPathGoal action_goal;
        action_goal.path = prepared;
        active_path_ = prepared;
        path_done_.store(false);
        path_succeeded_.store(false);
        distance_remaining_.store(-1.0);
        last_feedback_distance_ = -1.0;
        last_path_feedback_at_ = ros::Time::now();
        const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
        path_client_->sendGoal(
            action_goal,
            boost::bind(&MissionManager::pathDoneCb, this, generation,
                        boost::placeholders::_1, boost::placeholders::_2),
            boost::bind(&MissionManager::pathActiveCb, this, generation),
            boost::bind(&MissionManager::pathFeedbackCb, this, generation,
                        boost::placeholders::_1));
        ROS_INFO("[periodic_replan] hot installed waypoint=%d poses=%zu flags=%s",
                 current_index_, prepared.poses.size(), result.flags.c_str());
        EC_TRACE(ec::PATH_INSTALL, "replan_install", "wp=%d poses=%zu flags=%s",
                 current_index_, prepared.poses.size(), result.flags.c_str());
      } else if (gate_block_install) {
        ROS_WARN("[periodic_replan] install rejected by gate reason=%s (%s) "
                 "waypoint=%d",
                 gate.reason.c_str(), gate.detail.c_str(), current_index_);
        EC_TRACE(ec::PATH_REJECT, "replan_install", "reason=%s detail=%s wp=%d",
                 gate.reason.c_str(), gate.detail.c_str(), current_index_);
      } else {
        ROS_WARN("[periodic_replan] candidate not installed waypoint=%d flags=%s",
                 current_index_, result.flags.c_str());
      }
    }
  }

  if (periodic_replan_running_ ||
      (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) ||
      current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if (!last_periodic_replan_at_.isZero() &&
      (now - last_periodic_replan_at_).toSec() < periodic_replan_sec_) {
    return;
  }
  last_periodic_replan_at_ = now;
  EC_TRACE(ec::REPLAN_TRIGGER, "periodic_replan", "wp=%d", current_index_);
  periodic_replan_running_ = true;
  geometry_msgs::PoseStamped goal = waypointPose(waypoints_[current_index_]);
  if (route_segment_enable_ && !global_route_.poses.empty()) {
    goal = global_route_.poses[route_segment_anchor_index_];
  }
  const std::string mission_id = mission_id_;
  const int waypoint_index = current_index_;
  const std::uint64_t segment_generation = route_segment_generation_;
  const std::string service_name = plan_service_name_;
  periodic_replan_future_ = std::async(
      std::launch::async,
      [goal, mission_id, waypoint_index, segment_generation, service_name]() {
        PeriodicReplanResult result;
        result.mission_id = mission_id;
        result.waypoint_index = waypoint_index;
        result.segment_generation = segment_generation;
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<nav_interfaces::PlanPath>(service_name);
        nav_interfaces::PlanPath service;
        service.request.goal = goal;
        service.request.shadow_only = true;
        if (!client.call(service)) {
          result.flags = "PLAN_SERVICE_CALL_FAILED";
          return result;
        }
        result.flags = service.response.message;
        result.planned = service.response.success &&
                         !service.response.path.poses.empty();
        if (result.planned) result.path = service.response.path;
        return result;
      });
}

void MissionManager::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  last_odom_rx_at_ = ros::Time::now();
  linear_velocity_ = msg->twist.twist.linear.x;
  angular_velocity_ = msg->twist.twist.angular.z;
  if (local_planner_mode_ == "goal_pose") {
    last_odom_pose_.header = msg->header;
    last_odom_pose_.pose = msg->pose.pose;
    last_odom_pose_valid_ = true;
  }
}

void MissionManager::statusTimerCb(const ros::TimerEvent &) { publishStatus(); }

void MissionManager::pathDoneCb(
    std::uint64_t generation,
    const actionlib::SimpleClientGoalState &state,
    const nav_interfaces::navPathResultConstPtr &result) {
  if (generation != path_goal_generation_.load()) return;
  path_succeeded_.store(state == actionlib::SimpleClientGoalState::SUCCEEDED &&
                        result && result->status == 0);
  path_done_.store(true);
  last_path_feedback_at_ = ros::Time::now();
  ROS_INFO("[mission_manager] NeuPAN path result state=%s status=%d succeeded=%s",
           state.toString().c_str(), result ? result->status : -1,
           path_succeeded_.load() ? "true" : "false");
}

void MissionManager::pathActiveCb(std::uint64_t generation) {
  if (generation != path_goal_generation_.load()) return;
  path_done_.store(false);
  last_path_feedback_at_ = ros::Time::now();
}

void MissionManager::pathFeedbackCb(
    std::uint64_t generation,
    const nav_interfaces::navPathFeedbackConstPtr &feedback) {
  if (generation != path_goal_generation_.load()) return;
  if (feedback) {
    distance_remaining_.store(feedback->distance_remaining);
    last_path_feedback_at_ = ros::Time::now();
    constexpr double kStallProgressEpsilon = 0.05;  // meters
    if (last_feedback_progress_at_.isZero() ||
        std::fabs(feedback->distance_remaining - last_feedback_distance_) >
            kStallProgressEpsilon) {
      last_feedback_distance_ = feedback->distance_remaining;
      last_feedback_progress_at_ = last_path_feedback_at_;
    }
  }
}

void MissionManager::replanCb(const std_msgs::EmptyConstPtr &) {
  if (recovery_episode_active_) return;
  if (state_ == State::NAVIGATING || state_ == State::RETURNING_HOME) {
    const ros::Time now = ros::Time::now();
    if (replan_cooldown_sec_ > 0.0 && !last_replan_at_.isZero() &&
        (now - last_replan_at_).toSec() < replan_cooldown_sec_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring NeuPAN replan inside %.1f s cooldown",
                        replan_cooldown_sec_);
      return;
    }
    last_replan_at_ = now;
    replan_requested_.store(true);
  }
}

void MissionManager::recoveryRequestCb(
    const nav_interfaces::RecoveryRequestConstPtr &msg) {
  if (!recovery_shadow_enable_ || !msg || msg->request_id.empty()) {
    return;
  }
  if (msg->state == nav_interfaces::RecoveryRequest::RECOVERED ||
      msg->state == nav_interfaces::RecoveryRequest::ABORTED) {
    if (msg->request_id == recovery_active_request_id_) {
      publishRecoveryCommand(*msg, nav_interfaces::RecoveryCommand::ABORT, 0,
                             0.0, recovery_candidate_flags_, nav_msgs::Path(),
                             "episode_recovered_or_aborted");
      recovery_active_request_id_.clear();
      clearRecoveryCandidate();
    }
    return;
  }
  if (msg->state == nav_interfaces::RecoveryRequest::ESCAPING) {
    return;
  }
  if (msg->state == nav_interfaces::RecoveryRequest::STILL_BLOCKED) {
    if (msg->request_id == recovery_candidate_id_) {
      publishRecoveryCommand(*msg, nav_interfaces::RecoveryCommand::WAIT,
                             recovery_candidate_side_, 0.0,
                             recovery_candidate_flags_, recovery_candidate_path_,
                             "still_blocked");
    }
    return;
  }
  if (msg->state == nav_interfaces::RecoveryRequest::CLEARED) {
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
  if (msg->state != nav_interfaces::RecoveryRequest::BLOCKED ||
      msg->request_id == last_recovery_request_id_) {
    return;
  }
  last_recovery_request_id_ = msg->request_id;
  if ((state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) ||
      current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    publishRecoveryCommand(*msg, nav_interfaces::RecoveryCommand::ABORT, 0, 0.0,
                           "", nav_msgs::Path(), "mission_not_navigating");
    return;
  }
  clearRecoveryCandidate(true);
  recovery_active_request_id_ = msg->request_id;
  recovery_episode_active_ = recovery_behavior_enable_;
  if (recovery_shadow_running_) {
    publishRecoveryCommand(*msg, nav_interfaces::RecoveryCommand::WAIT, 0, 0.0,
                           "", nav_msgs::Path(), "shadow_busy");
    return;
  }

  const nav_interfaces::RecoveryRequest request = *msg;
  const geometry_msgs::PoseStamped goal = waypointPose(waypoints_[current_index_]);
  const std::string mission_id = mission_id_;
  const int waypoint_index = current_index_;
  const std::string service_name = plan_service_name_;
  ros::NodeHandle nh = nh_;
  recovery_shadow_running_ = true;
  recovery_shadow_future_ = std::async(
      std::launch::async,
      [request, goal, mission_id, waypoint_index, service_name, nh]() mutable {
        ShadowResult result;
        result.request = request;
        result.mission_id = mission_id;
        result.waypoint_index = waypoint_index;
        ros::ServiceClient client =
            nh.serviceClient<nav_interfaces::PlanPath>(service_name);
        if (!client.waitForExistence(ros::Duration(2.0))) {
          result.reason = "PLAN_SERVICE_UNAVAILABLE";
          return result;
        }
        nav_interfaces::PlanPath service;
        service.request.goal = goal;
        service.request.shadow_only = true;
        if (!client.call(service)) {
          result.reason = "PLAN_SERVICE_CALL_FAILED";
          return result;
        }
        result.flags = service.response.message;
        result.reason = service.response.message;
        result.planned = service.response.success &&
                         !service.response.path.poses.empty();
        if (result.planned) result.path = service.response.path;
        return result;
      });
  ROS_INFO("[recovery_shadow] started id=%s local_side=%d (no preempt)",
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
    publishRecoveryCommand(result.request, nav_interfaces::RecoveryCommand::ABORT,
                           0, 0.0, result.flags, nav_msgs::Path(),
                           "stale_mission_or_waypoint");
    return;
  }
  geometry_msgs::PoseStamped pose;
  pose.header = result.request.header;
  pose.pose = result.request.robot_pose;
  if (!result.planned && recovery_behavior_enable_) {
    nav_msgs::Path fallback = prepareRecoveryPath(active_path_, pose);
    if (fallback.poses.size() >= 2) {
      const std::string failure = result.reason;
      result.planned = true;
      result.path = fallback;
      result.flags = "OK|FALLBACK_ACTIVE_PATH";
      result.reason = "fallback_active_path:" + failure;
    }
  }
  if (!result.planned) {
    publishRecoveryCommand(result.request, nav_interfaces::RecoveryCommand::WAIT,
                           0, 0.0, result.flags, nav_msgs::Path(), result.reason);
    ROS_WARN("[recovery_shadow] id=%s no candidate: %s",
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
  uint8_t command = nav_interfaces::RecoveryCommand::SIDE;
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
    recovery_candidate_created_at_ = ros::Time::now();
  } else if (conflict) {
    command = nav_interfaces::RecoveryCommand::WAIT;
  }
  publishRecoveryCommand(result.request, command, side, delta, result.flags,
                         result.path, reason);
  ROS_WARN("[recovery_shadow] id=%s local_side=%d candidate_side=%d "
           "heading=%+.1fdeg flags=%s result=%s behavior=%s",
           result.request.request_id.c_str(),
           static_cast<int>(result.request.local_side), static_cast<int>(side),
           delta * 180.0 / M_PI, result.flags.c_str(), reason.c_str(),
           recovery_behavior_enable_ ? "true" : "false");
  if (recovery_commit_pending_ &&
      recovery_pending_commit_request_.request_id == result.request.request_id) {
    const nav_interfaces::RecoveryRequest pending =
        recovery_pending_commit_request_;
    recovery_commit_pending_ = false;
    if (command == nav_interfaces::RecoveryCommand::SIDE) {
      commitRecoveryCandidate(pending);
    }
  }
}

void MissionManager::clearRecoveryCandidate(bool notify_abort) {
  const std::string request_id = recovery_candidate_id_.empty()
                                     ? recovery_active_request_id_
                                     : recovery_candidate_id_;
  if (notify_abort && !request_id.empty()) {
    nav_interfaces::RecoveryCommand response;
    response.header.stamp = ros::Time::now();
    response.header.frame_id = map_frame_;
    response.request_id = request_id;
    response.command = nav_interfaces::RecoveryCommand::ABORT;
    response.reason = "mission_state_changed";
    recovery_command_pub_.publish(response);
  }
  recovery_candidate_id_.clear();
  recovery_candidate_mission_id_.clear();
  recovery_candidate_waypoint_index_ = -1;
  recovery_candidate_side_ = 0;
  recovery_candidate_flags_.clear();
  recovery_candidate_path_.poses.clear();
  recovery_candidate_created_at_ = ros::Time();
  recovery_active_request_id_.clear();
  recovery_episode_active_ = false;
  recovery_commit_pending_ = false;
}

nav_msgs::Path MissionManager::prepareRecoveryPath(
    const nav_msgs::Path &candidate,
    const geometry_msgs::PoseStamped &pose) const {
  nav_msgs::Path prepared;
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
  prepared.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped start = pose;
  start.header = prepared.header;
  prepared.poses.push_back(start);
  const std::size_t begin =
      nearest_distance < 0.05 ? nearest_index + 1 : nearest_index;
  for (std::size_t i = begin; i < candidate.poses.size(); ++i) {
    geometry_msgs::PoseStamped point = candidate.poses[i];
    point.header = prepared.header;
    prepared.poses.push_back(point);
  }
  return prepared;
}

nav_msgs::Path MissionManager::preparePeriodicPath(
    const nav_msgs::Path &candidate,
    const geometry_msgs::PoseStamped &pose) const {
  nav_msgs::Path prepared;
  if (candidate.poses.empty() || candidate.header.frame_id != map_frame_) {
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
  prepared.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped start = pose;
  start.header = prepared.header;
  prepared.poses.push_back(start);
  std::size_t begin = nearest_index;
  while (begin < candidate.poses.size()) {
    const auto &point = candidate.poses[begin].pose.position;
    if (std::hypot(point.x - pose.pose.position.x,
                   point.y - pose.pose.position.y) >= 0.30) {
      break;
    }
    ++begin;
  }
  for (std::size_t i = begin; i < candidate.poses.size(); ++i) {
    geometry_msgs::PoseStamped point = candidate.poses[i];
    point.header = prepared.header;
    prepared.poses.push_back(point);
  }
  return prepared;
}

void MissionManager::commitRecoveryCandidate(
    const nav_interfaces::RecoveryRequest &request) {
  if (state_ != State::NAVIGATING && state_ != State::RETURNING_HOME) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::ABORT, 0,
                           0.0, recovery_candidate_flags_, nav_msgs::Path(),
                           "mission_state_not_navigating");
    clearRecoveryCandidate();
    return;
  }
  if (!recovery_behavior_enable_ || request.request_id != recovery_candidate_id_ ||
      recovery_candidate_mission_id_ != mission_id_ ||
      recovery_candidate_waypoint_index_ != current_index_ ||
      recovery_candidate_path_.poses.empty()) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::WAIT, 0,
                           0.0, recovery_candidate_flags_, nav_msgs::Path(),
                           "candidate_missing_or_stale");
    return;
  }
  if (recovery_candidate_max_age_sec_ > 0.0 &&
      (ros::Time::now() - recovery_candidate_created_at_).toSec() >
          recovery_candidate_max_age_sec_) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::Path(),
                           "candidate_expired");
    clearRecoveryCandidate();
    return;
  }

  if (recovery_candidate_path_.header.frame_id != map_frame_) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::ABORT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::Path(),
                           "candidate_wrong_frame");
    clearRecoveryCandidate();
    return;
  }
  geometry_msgs::PoseStamped pose;
  if (!getRobotPose(pose)) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::Path(),
                           "robot_pose_unavailable");
    return;
  }
  nav_msgs::Path prepared = prepareRecoveryPath(recovery_candidate_path_, pose);
  if (prepared.poses.size() < 2) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::Path(),
                           "candidate_start_too_far");
    clearRecoveryCandidate();
    return;
  }
  if (!path_client_->waitForServer(ros::Duration(plan_timeout_sec_))) {
    publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::WAIT,
                           recovery_candidate_side_, 0.0,
                           recovery_candidate_flags_, nav_msgs::Path(),
                           "nav_path_action_unavailable");
    return;
  }

  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = ros::Time::now();
  last_path_feedback_at_ = path_started_at_;
  last_feedback_progress_at_ = ros::Time();
  last_feedback_distance_ = -1.0;
  nav_interfaces::navPathGoal action_goal;
  action_goal.path = prepared;
  active_path_ = prepared;
  const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
  path_client_->sendGoal(action_goal,
                         boost::bind(&MissionManager::pathDoneCb, this, generation,
                                     boost::placeholders::_1,
                                     boost::placeholders::_2),
                         boost::bind(&MissionManager::pathActiveCb, this, generation),
                         boost::bind(&MissionManager::pathFeedbackCb, this, generation,
                                     boost::placeholders::_1));
  publishRecoveryCommand(request, nav_interfaces::RecoveryCommand::COMMIT,
                         recovery_candidate_side_, 0.0,
                         recovery_candidate_flags_, prepared,
                         "cached_candidate_committed");
  publishEvent("RECOVERY_COMMIT", "Cached recovery candidate committed");
  clearRecoveryCandidate();
}

void MissionManager::publishRecoveryCommand(
    const nav_interfaces::RecoveryRequest &request, uint8_t command, int8_t side,
    double signed_heading, const std::string &flags,
    const nav_msgs::Path &path, const std::string &reason) {
  nav_interfaces::RecoveryCommand response;
  response.header.stamp = ros::Time::now();
  response.header.frame_id = map_frame_;
  response.request_id = request.request_id;
  response.command = command;
  response.side = side;
  response.signed_heading = signed_heading;
  response.flags = flags;
  response.path = path;
  response.reason = reason;
  recovery_command_pub_.publish(response);
}

double MissionManager::candidateHeadingDelta(
    const nav_msgs::Path &path,
    const geometry_msgs::PoseStamped &pose) const {
  if (path.poses.size() < 2) return 0.0;
  const auto &start = path.poses.front().pose.position;
  geometry_msgs::Point target = path.poses.back().pose.position;
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
  ROS_WARN("Skipping waypoint %d after failure: %s", current_index_, fail_reason.c_str());
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
  clearRouteSegment();
  resetInstallGateState("waypoint");
  if (local_planner_mode_ == "goal_pose") {
    return startSruWaypoint();
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

    nav_msgs::Path path;
    std::string fail_reason;
    bool planned = planCurrentWaypoint(path, fail_reason) && !path.poses.empty();
    for (int attempt = 0;
         !planned && attempt < plan_max_retries_ &&
         (fail_reason == "START_OCCUPIED" || fail_reason == "START_AND_GOAL_OCCUPIED" ||
          fail_reason == "GOAL_OCCUPIED");
         ++attempt) {
      ROS_WARN("Global planning failed (%s) for waypoint %d, retrying in %.1f s (attempt %d/%d)",
               fail_reason.c_str(), current_index_, plan_retry_backoff_sec_, attempt + 1,
               plan_max_retries_);
      ros::Duration(plan_retry_backoff_sec_).sleep();
      planned = planCurrentWaypoint(path, fail_reason) && !path.poses.empty();
    }

    if (!planned) {
      if (temporary_mission_) {
        restorePausedMission("Temporary global planning failed; main mission remains paused");
        return false;
      }
      ROS_WARN("Skipping waypoint %d after planning failure: %s", current_index_,
               fail_reason.c_str());
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

    if (route_segment_enable_ && navigation_mode_ == "replan_periodic") {
      global_route_ = path;
      geometry_msgs::PoseStamped pose;
      if (!getRobotPose(pose)) pose = path.poses.front();
      nav_msgs::Path segment = buildRouteSegment(pose);
      if (segment.poses.size() >= 2) path = segment;
    }

    if (!path_client_->waitForServer(ros::Duration(plan_timeout_sec_))) {
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
    path_started_at_ = ros::Time::now();
    last_path_feedback_at_ = path_started_at_;
    last_feedback_progress_at_ = ros::Time();
    last_feedback_distance_ = -1.0;
    nav_interfaces::navPathGoal action_goal;
    action_goal.path = path;
    active_path_ = path;
    if (navigation_mode_ == "replan_periodic") {
      last_periodic_replan_at_ = ros::Time::now();
    }
    const std::uint64_t generation = path_goal_generation_.fetch_add(1) + 1;
    path_client_->sendGoal(action_goal,
                           boost::bind(&MissionManager::pathDoneCb, this, generation,
                                       boost::placeholders::_1, boost::placeholders::_2),
                           boost::bind(&MissionManager::pathActiveCb, this, generation),
                           boost::bind(&MissionManager::pathFeedbackCb, this, generation,
                                       boost::placeholders::_1));
    transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
    return true;
  }

  transitionTo(State::FAILED,
               "All remaining waypoints failed to plan（共 " +
                   std::to_string(total) + " 个航点，已完成 " +
                   std::to_string(completed_waypoints_) + " 个）");
  return false;
}

bool MissionManager::planCurrentWaypoint(nav_msgs::Path &path, std::string &fail_reason) {
  if (!plan_client_.waitForExistence(ros::Duration(plan_timeout_sec_))) {
    ROS_WARN("Map planner service is unavailable: %s", plan_service_name_.c_str());
    fail_reason = "PLAN_SERVICE_UNAVAILABLE";
    return false;
  }
  nav_interfaces::PlanPath service;
  service.request.goal = waypointPose(waypoints_[current_index_]);
  service.request.shadow_only =
      route_segment_enable_ && navigation_mode_ == "replan_periodic";
  if (!plan_client_.call(service)) {
    fail_reason = "PLAN_SERVICE_CALL_FAILED";
    return false;
  }
  if (!service.response.success) {
    fail_reason = service.response.message;
    return false;
  }
  path = service.response.path;
  return true;
}

// SRU backend (local_planner=sru):
// Dispatches map-frame waypoints transformed into odom_frame_ (/sru/goal_pose),
// monitors arrival and stall progress, refreshes goals under map->odom drift,
// and gates /sru/cmd_vel via /sru/cmd_vel_enable.

bool MissionManager::sruGoalFromWaypoint(const nav_interfaces::Waypoint &waypoint,
                                         geometry_msgs::PoseStamped &out) const {
  geometry_msgs::PoseStamped goal_map = waypointPose(waypoint);
  try {
    tf_buffer_.transform(goal_map, out, odom_frame_, ros::Duration(0.5));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN("[sru] map->%s transform failed for waypoint %d: %s",
             odom_frame_.c_str(), current_index_, ex.what());
    return false;
  }
  out.header.frame_id = odom_frame_;
  out.header.stamp = ros::Time::now();
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

  geometry_msgs::PoseStamped goal_odom;
  if (!sruGoalFromWaypoint(waypoints_[current_index_], goal_odom)) {
    ROS_WARN("[sru] skipping waypoint %d: map->%s transform failed",
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
  sru_last_refresh_at_ = ros::Time::now();
  sru_stall_ref_distance_ = -1.0;
  sru_stall_ref_at_ = ros::Time(0);
  sru_goal_pub_.publish(goal_odom);
  publishSruEnable(true);
  ROS_INFO("[sru] waypoint %d dispatched: map=(%.2f, %.2f, yaw=%.2f) -> odom=(%.2f, %.2f)",
           current_index_, waypoints_[current_index_].x, waypoints_[current_index_].y,
           waypoints_[current_index_].yaw, goal_odom.pose.position.x,
           goal_odom.pose.position.y);
  transitionTo(returning_home_ ? State::RETURNING_HOME : State::NAVIGATING);
  return true;
}

void MissionManager::processSruNavigation() {
  const ros::Time now = ros::Time::now();

  if (sru_cmd_vel_seen_ &&
      (now - sru_last_cmd_vel_at_).toSec() > sru_cmd_vel_silence_sec_) {
    if (!sru_silence_warned_) {
      sru_silence_warned_ = true;
      ROS_ERROR("[sru] no /sru/cmd_vel for %.1fs; SRU node may have stalled or crashed",
                (now - sru_last_cmd_vel_at_).toSec());
      EC_TRACE(ec::NEUPAN_FEEDBACK_FROZEN, "sru", "cmd_vel silent %.1fs",
               (now - sru_last_cmd_vel_at_).toSec());
    }
  } else if (sru_silence_warned_) {
    sru_silence_warned_ = false;
    ROS_INFO("[sru] /sru/cmd_vel resumed");
  }

  if (current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
    return;
  }

  // Measure distance in sru_robot_frame_ (matching SRU's odometry child_frame).
  geometry_msgs::PoseStamped current;
  if (!getRobotPoseInFrame(sru_robot_frame_, current)) {
    ROS_WARN_THROTTLE(2.0, "[sru] robot pose unavailable; arrival judgement skipped");
    return;
  }
  const nav_interfaces::Waypoint &target = waypoints_[current_index_];
  const double remaining =
      std::hypot(target.x - current.pose.position.x, target.y - current.pose.position.y);

  if (remaining <= sru_arrive_threshold_m_) {
    ROS_INFO("[sru] waypoint %d reached: distance=%.3fm <= %.2fm",
             current_index_, remaining, sru_arrive_threshold_m_);
    transitionTo(State::ARRIVED);
    return;
  }

  // Stall check based on distance progress
  if (nav_stall_timeout_sec_ > 0.0) {
    if (sru_stall_ref_distance_ < 0.0 ||
        std::fabs(remaining - sru_stall_ref_distance_) > sru_stall_progress_epsilon_m_) {
      sru_stall_ref_distance_ = remaining;
      sru_stall_ref_at_ = now;
    } else if (!sru_stall_ref_at_.isZero() &&
               (now - sru_stall_ref_at_).toSec() > nav_stall_timeout_sec_) {
      ROS_ERROR("[sru] progress stalled: distance=%.3fm unchanged for %.1fs",
                remaining, (now - sru_stall_ref_at_).toSec());
      stopNavigation();
      skipCurrentWaypointOrFail("SRU progress stalled");
      return;
    }
  }

  // Periodically refresh the odom-frame goal when map->odom drift exceeds threshold
  if (sru_goal_valid_ && !sru_last_refresh_at_.isZero() &&
      (now - sru_last_refresh_at_).toSec() >= sru_goal_refresh_sec_) {
    sru_last_refresh_at_ = now;
    geometry_msgs::PoseStamped refreshed;
    if (sruGoalFromWaypoint(target, refreshed)) {
      const double drift = std::hypot(
          refreshed.pose.position.x - sru_goal_odom_.pose.position.x,
          refreshed.pose.position.y - sru_goal_odom_.pose.position.y);
      if (drift > sru_goal_refresh_min_delta_m_) {
        sru_goal_odom_ = refreshed;
        sru_goal_pub_.publish(refreshed);
        ROS_INFO("[sru] goal refreshed: map->odom drifted %.2fm", drift);
      }
    }
  }
}

void MissionManager::publishSruEnable(bool enable) {
  if (!sru_enable_pub_) return;
  std_msgs::Bool msg;
  msg.data = enable;
  sru_enable_pub_.publish(msg);
}

void MissionManager::sruCmdVelCb(const geometry_msgs::TwistConstPtr &msg) {
  (void)msg;
  sru_last_cmd_vel_at_ = ros::Time::now();
  sru_cmd_vel_seen_ = true;
}

void MissionManager::publishGlobalPathVisualization() {
  if (route_segment_enable_ && navigation_mode_ == "replan_periodic" &&
      !global_route_.poses.empty()) {
    global_path_pub_.publish(global_route_);
    return;
  }
  geometry_msgs::PoseStamped start;
  if (!getRobotPose(start)) {
    ROS_WARN("Unable to publish global path visualization without robot pose");
    return;
  }
  if (!sequence_plan_client_.waitForExistence(ros::Duration(plan_timeout_sec_))) {
    ROS_WARN("Sequence planner service is unavailable");
    return;
  }

  nav_interfaces::PlanSequence service;
  service.request.start = start;
  service.request.loop = loop_;
  service.request.goals.reserve(waypoints_.size());
  for (const auto &waypoint : waypoints_) {
    service.request.goals.push_back(waypointPose(waypoint));
  }
  if (!sequence_plan_client_.call(service) || !service.response.success ||
      service.response.path.poses.empty()) {
    ROS_WARN("Global path visualization planning failed: %s",
             service.response.message.c_str());
    return;
  }
  global_path_pub_.publish(service.response.path);
}

bool MissionManager::performAlignment(const nav_interfaces::Waypoint &target) {
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  if (!align_client_.waitForExistence(ros::Duration(3.0))) {
    ROS_ERROR("[mission_manager][alignment] absolute align service unavailable");
    return false;
  }
  // Handover settle window: sleep briefly so atomic_move consumes the
  // /neupan/stop published by stopNavigation() before /atomic_move/align arrives.
  ros::Duration(handover_settle_sec_).sleep();
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  atomic_move::AlignTo align;
  align.request.x = target.x;
  align.request.y = target.y;
  align.request.yaw = target.yaw;
  ROS_INFO("[mission_manager][alignment] absolute target frame=%s x=%.3f y=%.3f yaw=%.3f",
           map_frame_.c_str(), target.x, target.y, target.yaw);
  const bool call_ok = align_client_.call(align);
  ROS_INFO("[mission_manager][alignment] absolute align response call_ok=%s success=%s message='%s'",
           call_ok ? "true" : "false", align.response.success ? "true" : "false",
           align.response.message.c_str());
  if (alignment_cancel_requested_.load()) {
    return false;
  }
  if (call_ok && !align.response.success &&
      align.response.message == "ALIGN_CANCELLED") {
    ROS_ERROR("[mission_manager][alignment] atomic_move reported ALIGN_CANCELLED even "
              "after a %.3f s handover settle; a /neupan/stop is still racing the align "
              "request (see performAlignment comment) -- consider raising "
              "handover_settle_sec",
              handover_settle_sec_);
  }
  return call_ok && align.response.success;
}

void MissionManager::startAlignment() {
  joinAlignmentThread();
  alignment_cancel_requested_.store(false);
  alignment_result_pending_.store(false);
  alignment_result_.store(false);
  alignment_running_.store(true);
  const uint64_t generation = alignment_generation_.fetch_add(1) + 1;
  const nav_interfaces::Waypoint target = waypoints_[current_index_];
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
  geometry_msgs::PoseStamped odom_origin;
  odom_origin.header.frame_id = odom_frame_;
  odom_origin.header.stamp = ros::Time(0);
  odom_origin.pose.orientation.w = 1.0;
  try {
    geometry_msgs::PoseStamped map_origin;
    tf_buffer_.transform(odom_origin, map_origin, map_frame_, ros::Duration(0.5));
    home_pose_ = map_origin.pose;
    home_pose_valid_ = true;
    ROS_INFO("Captured fixed home pose from %s origin in %s at x=%.3f y=%.3f yaw=%.3f",
             odom_frame_.c_str(), map_frame_.c_str(), home_pose_.position.x,
             home_pose_.position.y, tf2::getYaw(home_pose_.orientation));
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(2.0, "Unable to transform %s origin to %s: %s",
                      odom_frame_.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
}

bool MissionManager::getRobotPose(geometry_msgs::PoseStamped &pose) const {
  return getRobotPoseInFrame(base_frame_, pose);
}

bool MissionManager::getRobotPoseInFrame(
    const std::string &frame, geometry_msgs::PoseStamped &pose) const {
  try {
    const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        map_frame_, frame, ros::Time(0), ros::Duration(0.2));
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(2.0, "Robot pose lookup failed: %s", ex.what());
    return false;
  }
}

geometry_msgs::PoseStamped MissionManager::waypointPose(
    const nav_interfaces::Waypoint &waypoint) const {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = map_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, waypoint.yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

void MissionManager::beginDwell() {
  alignment_started_at_ = ros::Time(0);
  ++completed_waypoints_;
  dwell_duration_ = std::max(0.0, waypoints_[current_index_].dwell_sec);
  dwell_remaining_ = dwell_duration_;
  dwell_start_ = ros::Time::now();
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
  alignment_cancel_requested_.store(true);
  if (path_client_) {
    path_client_->cancelAllGoals();
  }
  stop_pub_.publish(std_msgs::Empty());
  if (local_planner_mode_ == "goal_pose") {
    // Disable SRU velocity relay gate before alignment so atomic_move has exclusive /cmd_vel access.
    publishSruEnable(false);
    if (sru_goal_pub_ && last_odom_pose_valid_) {
      geometry_msgs::PoseStamped self = last_odom_pose_;
      self.header.stamp = ros::Time::now();
      sru_goal_pub_.publish(self);
    }
    sru_goal_valid_ = false;
    sru_stall_ref_distance_ = -1.0;
    sru_stall_ref_at_ = ros::Time(0);
  }
  path_done_.store(false);
  path_succeeded_.store(false);
  distance_remaining_.store(-1.0);
  path_started_at_ = ros::Time(0);
  last_path_feedback_at_ = ros::Time(0);
  last_feedback_progress_at_ = ros::Time(0);
  last_feedback_distance_ = -1.0;
}

void MissionManager::transitionTo(State state, const std::string &message) {
  state_ = state;
  publishEvent(stateName(state), message);
  publishStatus();
}

void MissionManager::publishStatus() const {
  nav_interfaces::MissionStatus status;
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
  status_pub_.publish(status);
}

void MissionManager::publishEvent(const std::string &event,
                                  const std::string &message) const {
  nav_interfaces::MissionEvent msg;
  msg.event = event;
  msg.waypoint_idx = current_index_ < 0 ? 0 : current_index_;
  if (current_index_ >= 0 && current_index_ < static_cast<int>(waypoints_.size())) {
    msg.waypoint_name = waypoints_[current_index_].name;
  }
  msg.timestamp = ros::Time::now().toSec();
  msg.message = message;
  event_pub_.publish(msg);
}

void MissionManager::publishOperatorCommand(const std::string &command_id,
                                             const std::string &command,
                                             const std::string &phase,
                                             const std::string &payload_json,
                                             bool success,
                                             const std::string &message) const {
  nav_interfaces::OperatorCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.command_id = command_id;
  msg.source = "mission_manager";
  msg.command = command;
  msg.phase = phase;
  msg.payload_json = payload_json;
  msg.success = success;
  msg.message = message;
  operator_command_pub_.publish(msg);
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
  stream << "mission_" << ros::Time::now().toNSec();
  return stream.str();
}

std::string MissionManager::newCommandId() const {
  std::ostringstream stream;
  stream << "command_" << ros::Time::now().toNSec();
  return stream.str();
}

double MissionManager::distanceToTarget() const {
  if (current_index_ >= 0 && current_index_ < static_cast<int>(waypoints_.size())) {
    geometry_msgs::PoseStamped current;
    if (getRobotPose(current)) {
      const nav_interfaces::Waypoint &target = waypoints_[current_index_];
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
  ros::init(argc, argv, "mission_manager");
  ::setenv("EC_NODE", "mission_manager", 1);
  EC_TRACE(ec::NODE_READY, "startup", "mission_manager up pid=%d", (int)::getpid());
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  MissionManager manager(nh, private_nh);
  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();
    manager.process();
    rate.sleep();
  }
  return 0;
}
