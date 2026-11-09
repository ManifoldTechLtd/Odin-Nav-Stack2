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

#ifndef MISSION_MANAGER_MISSION_MANAGER_H
#define MISSION_MANAGER_MISSION_MANAGER_H

#include <actionlib/client/simple_action_client.h>
#include <atomic>
#include <atomic_move/AlignTo.h>
#include <atomic_move/Turn.h>
#include <cstdint>
#include <geometry_msgs/PoseStamped.h>
#include <future>
#include <nav_interfaces/PlanPath.h>
#include <nav_interfaces/PlanSequence.h>
#include <memory>
#include <nav_interfaces/FollowSearch.h>
#include <nav_interfaces/MissionEvent.h>
#include <nav_interfaces/MissionStatus.h>
#include <nav_interfaces/OperatorCommand.h>
#include <nav_interfaces/RecoveryCommand.h>
#include <nav_interfaces/RecoveryRequest.h>
#include <nav_interfaces/StartMission.h>
#include <nav_interfaces/Waypoint.h>
#include <nav_msgs/Odometry.h>
#include <nav_interfaces/navPathAction.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mission_manager/replan_install_gate.h"

#include <string>
#include <thread>
#include <vector>

class MissionManager {
public:
  MissionManager(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~MissionManager();
  void process();

private:
  enum class State {
    IDLE,
    NAVIGATING,
    ARRIVED,
    ALIGNING,
    DWELLING,
    PAUSED,
    RETURNING_HOME,
    CANCELLED,
    COMPLETED,
    FAILED,
    // Continuous dynamic target following mode (entered via set_dynamic_mode(true)
    // when enable_dynamic_mode=true; holds position at goal without watchdog/stall/alignment).
    DYNAMIC,
    // Substate of DYNAMIC: in-place stepped rotation search triggered via follow_search
    // when the target is lost, until a new dynamic_goal is received or limits are reached.
    DYNAMIC_SEARCHING
  };

  // Search termination reason passed atomically from worker thread to main loop
  enum class SearchStop {
    CANCELLED = 0,   // Target reacquired, exited dynamic mode, or stopNavigation
    SUPERSEDED = 1,  // Replaced by a newer search request
    TIMEOUT = 2,     // Reached timeout_sec
    LIMIT = 3,       // Reached total_limit_deg
    BLOCKED = 4,     // Insufficient all-round obstacle clearance
    NO_DATA = 5,     // No /scan_360 data available
    TURN_FAILED = 6  // Turn service unavailable or returned failure
  };

  using NavPathClient = actionlib::SimpleActionClient<nav_interfaces::navPathAction>;
  struct ShadowResult {
    nav_interfaces::RecoveryRequest request;
    nav_msgs::Path path;
    std::string flags;
    std::string reason;
    std::string mission_id;
    int waypoint_index{-1};
    bool planned{false};
  };
  struct PeriodicReplanResult {
    nav_msgs::Path path;
    std::string flags;
    std::string mission_id;
    int waypoint_index{-1};
    std::uint64_t segment_generation{0};
    bool planned{false};
  };

  bool startMissionCb(nav_interfaces::StartMission::Request &req,
                      nav_interfaces::StartMission::Response &res);
  bool pauseCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool resumeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool returnHomeCb(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
  bool cancelCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool resetCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void goalCb(const geometry_msgs::PoseStampedConstPtr &msg);
  bool setDynamicModeCb(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
  void dynamicGoalCb(const geometry_msgs::PoseStampedConstPtr &msg);
  void processDynamicMode();
  bool startDynamicGoal();
  void exitDynamicMode(const std::string &message);
  // Target-loss rotation search (substate of DYNAMIC)
  bool followSearchCb(nav_interfaces::FollowSearch::Request &req,
                      nav_interfaces::FollowSearch::Response &res);
  void scan360Cb(const sensor_msgs::LaserScanConstPtr &msg);
  // Minimum all-round clearance (m) from robot center; < 0 if no /scan_360 received yet.
  double allRoundClearance() const;
  void startFollowSearch(double direction, double step_deg, double pause_sec,
                         double total_limit_deg, double timeout_sec,
                         double clearance_m);
  void followSearchLoop(double direction, double step_deg, double pause_sec,
                        double total_limit_deg, double timeout_sec,
                        double clearance_m, std::uint64_t generation);
  void joinSearchThread();
  // Set cancellation flag and publish /neupan/stop to interrupt any active atomic_move turn.
  void cancelFollowSearch(const std::string &reason);
  static std::string searchStopText(SearchStop code);
  void odomCb(const nav_msgs::OdometryConstPtr &msg);
  void statusTimerCb(const ros::TimerEvent &event);
  void pathDoneCb(std::uint64_t generation,
                  const actionlib::SimpleClientGoalState &state,
                  const nav_interfaces::navPathResultConstPtr &result);
  void pathActiveCb(std::uint64_t generation);
  void pathFeedbackCb(std::uint64_t generation,
                      const nav_interfaces::navPathFeedbackConstPtr &feedback);
  void processPeriodicReplan();
  void maybeAdvanceRouteSegment();
  nav_msgs::Path buildRouteSegment(const geometry_msgs::PoseStamped &pose);
  void clearRouteSegment();
  void replanCb(const std_msgs::EmptyConstPtr &msg);
  void recoveryRequestCb(const nav_interfaces::RecoveryRequestConstPtr &msg);
  void processRecoveryShadow();
  void clearRecoveryCandidate(bool notify_abort = false);
  void commitRecoveryCandidate(const nav_interfaces::RecoveryRequest &request);
  nav_msgs::Path prepareRecoveryPath(const nav_msgs::Path &candidate,
                                     const geometry_msgs::PoseStamped &pose) const;
  nav_msgs::Path preparePeriodicPath(const nav_msgs::Path &candidate,
                                     const geometry_msgs::PoseStamped &pose) const;
  void publishRecoveryCommand(const nav_interfaces::RecoveryRequest &request,
                              uint8_t command, int8_t side,
                              double signed_heading,
                              const std::string &flags,
                              const nav_msgs::Path &path,
                              const std::string &reason);
  double candidateHeadingDelta(const nav_msgs::Path &path,
                               const geometry_msgs::PoseStamped &pose) const;

  // Periodic replan install gate
  void resetInstallGateState(const std::string &why);
  void updateInstallGateBlocked();
  std::vector<mission_manager_gate::Pt> pathToPoints(
      const nav_msgs::Path &path) const;

  bool startCurrentWaypoint();
  // Shared per-waypoint failure recovery: skips the current waypoint and
  // starts the next one (or loops back to 0), instead of failing the whole
  // mission. Falls back to FAILED only once no waypoints remain.
  void skipCurrentWaypointOrFail(const std::string &fail_reason);
  bool planCurrentWaypoint(nav_msgs::Path &path, std::string &fail_reason);
  // SRU local planner backend (local_planner="sru"):
  // Transforms map-frame waypoints to odom frame (/sru/goal_pose), monitors
  // arrival by odometry distance, and gates /sru/cmd_vel via /sru/cmd_vel_enable.
  bool startSruWaypoint();
  void processSruNavigation();
  void publishSruEnable(bool enable);
  void sruCmdVelCb(const geometry_msgs::TwistConstPtr &msg);
  bool sruGoalFromWaypoint(const nav_interfaces::Waypoint &waypoint,
                           geometry_msgs::PoseStamped &out) const;
  void publishGlobalPathVisualization();
  bool performAlignment(const nav_interfaces::Waypoint &target);
  void startAlignment();
  void joinAlignmentThread();
  bool getRobotPose(geometry_msgs::PoseStamped &pose) const;
  bool getRobotPoseInFrame(const std::string &frame,
                           geometry_msgs::PoseStamped &pose) const;
  bool captureHomePoseIfNeeded();
  geometry_msgs::PoseStamped waypointPose(const nav_interfaces::Waypoint &waypoint) const;
  void beginDwell();
  void completeMission();
  void restorePausedMission(const std::string &message);
  void stopNavigation();
  void transitionTo(State state, const std::string &message = "");
  void publishStatus() const;
  void publishEvent(const std::string &event, const std::string &message = "") const;
  void publishOperatorCommand(const std::string &command_id,
                              const std::string &command,
                              const std::string &phase,
                              const std::string &payload_json,
                              bool success,
                              const std::string &message) const;
  std::string stateName(State state) const;
  std::string newMissionId() const;
  std::string newCommandId() const;
  double distanceToTarget() const;
  static double normalizeAngle(double angle);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer start_mission_srv_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer return_home_srv_;
  ros::ServiceServer cancel_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer set_dynamic_mode_srv_;
  ros::Subscriber dynamic_goal_sub_;
  ros::Publisher hot_path_pub_;
  ros::ServiceClient plan_client_;
  ros::ServiceClient sequence_plan_client_;
  ros::ServiceClient align_client_;
  ros::Publisher status_pub_;
  ros::Publisher event_pub_;
  ros::Publisher operator_command_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher recovery_command_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber replan_sub_;
  ros::Subscriber recovery_request_sub_;
  ros::Timer status_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<NavPathClient> path_client_;
  nav_msgs::Path active_path_;

  State state_{State::IDLE};
  State paused_from_{State::IDLE};
  std::vector<nav_interfaces::Waypoint> waypoints_;
  int current_index_{-1};
  bool loop_{false};
  bool temporary_mission_{false};
  std::vector<nav_interfaces::Waypoint> paused_waypoints_;
  int paused_current_index_{-1};
  bool paused_loop_{false};
  State paused_resume_state_{State::NAVIGATING};
  std::string paused_mission_id_;
  bool returning_home_{false};
  std::string mission_id_;
  geometry_msgs::Pose home_pose_;
  bool home_pose_valid_{false};
  ros::Time dwell_start_;
  double dwell_duration_{0.0};
  double dwell_remaining_{0.0};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  bool enable_alignment_{false};
  double plan_timeout_sec_{2.0};
  // Global planning failure retry/skip policy
  int plan_max_retries_{2};
  double plan_retry_backoff_sec_{3.0};
  double replan_cooldown_sec_{5.0};
  bool recovery_shadow_enable_{false};
  bool recovery_behavior_enable_{false};
  double recovery_candidate_max_age_sec_{5.0};
  double recovery_candidate_max_start_dist_m_{0.5};
  std::string last_recovery_request_id_;
  std::string recovery_active_request_id_;
  std::future<ShadowResult> recovery_shadow_future_;
  bool recovery_shadow_running_{false};
  bool recovery_episode_active_{false};
  bool recovery_commit_pending_{false};
  nav_interfaces::RecoveryRequest recovery_pending_commit_request_;
  std::string recovery_candidate_id_;
  std::string recovery_candidate_mission_id_;
  int recovery_candidate_waypoint_index_{-1};
  int8_t recovery_candidate_side_{0};
  std::string recovery_candidate_flags_;
  nav_msgs::Path recovery_candidate_path_;
  ros::Time recovery_candidate_created_at_;
  double nav_watchdog_timeout_sec_{15.0};
  double nav_stall_timeout_sec_{15.0};
  double alignment_watchdog_timeout_sec_{25.0};
  // Settle window between publishing /neupan/stop and calling /atomic_move/align.
  double handover_settle_sec_{0.2};
  int completed_waypoints_{0};
  ros::Time path_started_at_;
  ros::Time last_path_feedback_at_;
  ros::Time last_odom_rx_at_;
  bool ec_perception_offline_{false};
  ros::Time last_feedback_progress_at_;
  double last_feedback_distance_{-1.0};
  ros::Time alignment_started_at_;
  ros::Time last_replan_at_;
  std::thread alignment_thread_;
  std::atomic<bool> alignment_cancel_requested_{false};
  std::atomic<bool> alignment_running_{false};
  std::atomic<bool> alignment_result_pending_{false};
  std::atomic<bool> alignment_result_{false};
  std::atomic<std::uint64_t> alignment_generation_{0};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  std::string odom_topic_{"/odin1/odometry"};
  std::string plan_service_name_{"/map_planner/plan"};
  std::string nav_path_action_name_{"/neupan/nav_path"};
  std::string stop_topic_{"/neupan/stop"};
  std::string sru_goal_topic_{"/sru/goal_pose"};
  std::string sru_enable_topic_{"/sru/cmd_vel_enable"};
  std::string sru_cmd_vel_topic_{"/sru/cmd_vel"};
  std::string local_planner_{"neupan"};
  std::string local_planner_mode_{""};
  std::atomic<bool> path_done_{false};
  std::atomic<bool> path_succeeded_{false};
  std::atomic<double> distance_remaining_{-1.0};
  std::atomic<bool> replan_requested_{false};
  std::atomic<std::uint64_t> path_goal_generation_{0};
  std::string navigation_mode_{"escape"};
  double periodic_replan_sec_{2.0};
  ros::Time last_periodic_replan_at_;
  std::future<PeriodicReplanResult> periodic_replan_future_;
  bool periodic_replan_running_{false};
  bool route_segment_enable_{true};
  double route_segment_length_m_{10.0};
  double route_segment_overlap_m_{3.0};
  nav_msgs::Path global_route_;
  std::size_t route_progress_index_{0};
  std::size_t route_segment_anchor_index_{0};
  std::uint64_t route_segment_generation_{0};
  bool route_segment_final_{true};

  // Dynamic following mode state
  bool enable_dynamic_mode_{false};
  bool dynamic_mode_{false};
  geometry_msgs::PoseStamped dynamic_goal_;
  bool dynamic_goal_pending_{false};
  bool dynamic_goal_valid_{false};
  bool dynamic_nav_active_{false};
  ros::Time last_dynamic_plan_at_;
  double dynamic_replan_cooldown_sec_{0.5};
  std::string dynamic_goal_topic_{"/waypoint_manager/dynamic_goal"};
  int dynamic_plan_failures_{0};

  // Target-loss rotation search state (substate of DYNAMIC)
  ros::ServiceServer follow_search_srv_;
  ros::Subscriber scan360_sub_;
  ros::ServiceClient turn_client_;
  std::thread search_thread_;
  std::atomic<bool> search_cancel_requested_{false};
  std::atomic<bool> search_running_{false};
  std::atomic<bool> search_result_pending_{false};
  std::atomic<std::uint64_t> search_generation_{0};
  std::atomic<int> search_stop_code_{0};
  std::atomic<double> all_round_clearance_{-1.0};
  std::atomic<bool> scan360_received_{false};
  bool enable_follow_search_{false};
  double follow_search_clearance_m_{0.60};
  double follow_search_step_deg_{90.0};
  double follow_search_step_pause_sec_{0.3};
  double follow_search_timeout_sec_{0.0};
  double follow_search_total_limit_deg_{0.0};
  std::string follow_search_turn_service_{"/atomic_move/turn"};
  std::string follow_search_scan_topic_{"/scan_360"};

  // Periodic replan install gate state
  ros::Publisher install_gate_shadow_pub_;  // /neupan/replan_gate_shadow
  bool install_gate_enable_{false};
  bool install_gate_shadow_{false};
  bool install_gate_scan_subscribed_{false};
  mission_manager_gate::GateConfig install_gate_cfg_;
  std::unique_ptr<mission_manager_gate::BlockedHysteresis> install_gate_blocked_;
  std::vector<mission_manager_gate::Pt> install_gate_scan_map_pts_;
  ros::Time install_gate_scan_stamp_;
  bool install_gate_scan_ready_{false};
  std::atomic<bool> install_gate_blocked_raw_{false};
  std::atomic<bool> install_gate_blocked_gated_{false};
  std::atomic<double> install_gate_min_lateral_m_{-1.0};
  double install_gate_blocked_dwell_sec_{0.0};

  // SRU backend runtime state
  ros::Publisher sru_goal_pub_;
  ros::Publisher sru_enable_pub_;
  ros::Subscriber sru_cmd_vel_sub_;
  double sru_arrive_threshold_m_{0.7};
  double sru_goal_refresh_sec_{1.0};
  double sru_goal_refresh_min_delta_m_{0.30};
  double sru_cmd_vel_silence_sec_{2.0};
  double sru_stall_progress_epsilon_m_{0.05};
  std::string sru_robot_frame_{"odin1_base_link"};
  geometry_msgs::PoseStamped sru_goal_odom_;
  bool sru_goal_valid_{false};
  ros::Time sru_last_refresh_at_;
  ros::Time sru_last_cmd_vel_at_;
  bool sru_cmd_vel_seen_{false};
  bool sru_silence_warned_{false};
  double sru_stall_ref_distance_{-1.0};
  ros::Time sru_stall_ref_at_;
  geometry_msgs::PoseStamped last_odom_pose_;
  bool last_odom_pose_valid_{false};
};

#endif
