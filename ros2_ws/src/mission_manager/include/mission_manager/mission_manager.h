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

// ROS 2 Jazzy port of mission_manager.
// Concurrency model:
//   - cb_group_main_ (MutuallyExclusive): serializes all service servers, subscriptions,
//     status timer, and the 100ms process() timer.
//   - cb_group_clients_ (Reentrant): used by all ServiceClients and the NavPath action client
//     so synchronous waits inside process() or worker threads do not deadlock under
//     MultiThreadedExecutor(4).

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic_move/srv/align_to.hpp>
#include <atomic_move/srv/turn.hpp>
#include <nav_interfaces/srv/plan_path.hpp>
#include <nav_interfaces/srv/plan_sequence.hpp>
#include <nav_interfaces/srv/validate_path.hpp>
#include <nav_interfaces/msg/mission_event.hpp>
#include <nav_interfaces/msg/mission_status.hpp>
#include <nav_interfaces/msg/operator_command.hpp>
#include <nav_interfaces/msg/recovery_command.hpp>
#include <nav_interfaces/msg/recovery_request.hpp>
#include <nav_interfaces/msg/waypoint.hpp>
#include <nav_interfaces/srv/follow_search.hpp>
#include <nav_interfaces/srv/start_mission.hpp>
#include <nav_interfaces/action/nav_path.hpp>

#include "mission_manager/replan_install_gate.h"

class MissionManager {
public:
  explicit MissionManager(const rclcpp::Node::SharedPtr &node);
  ~MissionManager();
  void process();

private:
  using NavPathAction = nav_interfaces::action::NavPath;
  using NavPathClient = rclcpp_action::Client<NavPathAction>;

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
    // when enable_dynamic_mode=true; holds position at target without watchdog/alignment).
    DYNAMIC,
    // Sub-state of DYNAMIC: executes stepped in-place turns via /atomic_move/turn
    // until a new dynamic_goal is received or a safety/limit condition stops the search.
    DYNAMIC_SEARCHING
  };

  // Search termination reason passed from worker thread to main thread via atomic code
  enum class SearchStop {
    CANCELLED = 0,   // Target reacquired, dynamic mode exited, or stopNavigation called
    SUPERSEDED = 1,  // Replaced by a newer search request
    TIMEOUT = 2,     // Reached timeout_sec
    LIMIT = 3,       // Reached total_limit_deg
    BLOCKED = 4,     // Insufficient all-round clearance
    NO_DATA = 5,     // No /scan_360 data available
    TURN_FAILED = 6  // Turn service unavailable or returned failure
  };

  struct ShadowResult {
    nav_interfaces::msg::RecoveryRequest request;
    nav_msgs::msg::Path path;
    std::string flags;
    std::string reason;
    std::string mission_id;
    int waypoint_index{-1};
    bool planned{false};
  };
  struct RouteTarget {
    geometry_msgs::msg::PoseStamped goal;
    std::size_t progress{0};
    std::size_t anchor{0};
    bool route{false};
    bool final{true};
  };
  struct PeriodicReplanResult {
    nav_msgs::msg::Path path;
    std::string flags;
    std::string mission_id;
    int waypoint_index{-1};
    std::uint64_t segment_generation{0};
    std::uint64_t path_generation{0};
    RouteTarget target;
    bool planned{false};
  };
  struct PathSafetyResult {
    std::uint64_t generation{0};
    bool valid{false};
    std::string reason;
  };

  // Synchronous service call helper (response delivered on cb_group_clients_).
  template <typename ClientT, typename RequestT>
  typename ClientT::Response::SharedPtr callSync(typename ClientT::SharedPtr client,
                                                 RequestT request,
                                                 double timeout_sec,
                                                 std::string *failure_reason = nullptr,
                                                 double discovery_sec = -1.0) {
    if (failure_reason) failure_reason->clear();
    if (!client || !client->wait_for_service(tf2::durationFromSec(
            discovery_sec < 0.0 ? timeout_sec : discovery_sec))) {
      if (failure_reason) *failure_reason = "PLAN_SERVICE_UNAVAILABLE";
      return nullptr;
    }
    auto future = client->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
        client->remove_pending_request(future);
        if (failure_reason) *failure_reason =
            rclcpp::ok() ? "PLAN_RESPONSE_TIMEOUT" : "PLAN_REQUEST_CANCELLED";
        return nullptr;
      }
    }
    return future.get();
  }
  void sendPathGoal(const nav_msgs::msg::Path &path,
                    const RouteTarget *target = nullptr);
  bool waitPathDone(double timeout_sec);

  void startMissionCb(const std::shared_ptr<nav_interfaces::srv::StartMission::Request> req,
                      std::shared_ptr<nav_interfaces::srv::StartMission::Response> res);
  void pauseCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void resumeCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void returnHomeCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void cancelCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void resetCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void goalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  // Dynamic target following mode services and callbacks
  void setDynamicModeCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void dynamicGoalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void processDynamicMode();
  bool startDynamicGoal();
  void exitDynamicMode(const std::string &message);
  // Stepped rotation search in DYNAMIC_SEARCHING mode
  void followSearchCb(const std::shared_ptr<nav_interfaces::srv::FollowSearch::Request> req,
                      std::shared_ptr<nav_interfaces::srv::FollowSearch::Response> res);
  void scan360Cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg);
  // Minimum all-round clearance (m) from robot center; < 0 if no /scan_360 received yet.
  double allRoundClearance() const;
  void startFollowSearch(double direction, double step_deg, double pause_sec,
                         double total_limit_deg, double timeout_sec,
                         double clearance_m);
  void followSearchLoop(double direction, double step_deg, double pause_sec,
                        double total_limit_deg, double timeout_sec,
                        double clearance_m, std::uint64_t generation);
  void joinSearchThread();
  // Set cancel flag and publish /neupan/stop to interrupt an active atomic_move turn immediately.
  void cancelFollowSearch(const std::string &reason);
  static std::string searchStopText(SearchStop code);
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void statusTimerCb();
  void pathDoneCb(std::uint64_t generation,
                  const rclcpp_action::ClientGoalHandle<NavPathAction>::WrappedResult &result);
  void pathActiveCb(std::uint64_t generation);
  void pathRejectedCb(std::uint64_t generation);
  void pathFeedbackCb(std::uint64_t generation,
                      const std::shared_ptr<const NavPathAction::Feedback> &feedback);
  void processPeriodicReplan();
  bool startInitialRoutePlan();
  void processInitialRoutePlan();
  void installInitialRoute(nav_msgs::msg::Path path);
  void maybeAdvanceRouteSegment();
  std::vector<RouteTarget> routeTargets(
      const geometry_msgs::msg::PoseStamped &pose, bool advance) const;
  void commitRouteTarget();
  bool validateForInstall(nav_msgs::msg::Path &path, std::string &reason);
  void processPathSafety();
  void holdForPathSafety(const std::string &reason);
  bool pathScanFresh() const;
  void clearRouteSegment();
  void replanCb(const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void recoveryRequestCb(const nav_interfaces::msg::RecoveryRequest::ConstSharedPtr &msg);
  void processRecoveryShadow();
  void clearRecoveryCandidate(bool notify_abort = false);
  void commitRecoveryCandidate(const nav_interfaces::msg::RecoveryRequest &request);
  nav_msgs::msg::Path prepareRecoveryPath(
      const nav_msgs::msg::Path &candidate,
      const geometry_msgs::msg::PoseStamped &pose) const;
  nav_msgs::msg::Path preparePeriodicPath(
      const nav_msgs::msg::Path &candidate,
      const geometry_msgs::msg::PoseStamped &pose) const;
  void publishRecoveryCommand(const nav_interfaces::msg::RecoveryRequest &request,
                              uint8_t command, int8_t side,
                              double signed_heading,
                              const std::string &flags,
                              const nav_msgs::msg::Path &path,
                              const std::string &reason);
  double candidateHeadingDelta(const nav_msgs::msg::Path &path,
                               const geometry_msgs::msg::PoseStamped &pose) const;

  // Periodic replan install gate
  void resetInstallGateState(const std::string &why);
  void updateInstallGateBlocked();
  std::vector<mission_manager_gate::Pt> pathToPoints(
      const nav_msgs::msg::Path &path) const;

  bool startCurrentWaypoint();
  // Skips the current waypoint and advances to the next one (or transitions to FAILED
  // if none remain; restores the paused mission when running a temporary mission).
  void skipCurrentWaypointOrFail(const std::string &fail_reason);
  bool planCurrentWaypoint(nav_msgs::msg::Path &path, std::string &fail_reason);
  // SRU local planner backend (local_planner="sru")
  bool startSruWaypoint();
  void processSruNavigation();
  void publishSruEnable(bool enable);
  void sruCmdVelCb(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
  bool sruGoalFromWaypoint(const nav_interfaces::msg::Waypoint &waypoint,
                           geometry_msgs::msg::PoseStamped &out) const;
  void publishGlobalPathVisualization();
  bool performAlignment(const nav_interfaces::msg::Waypoint &target);
  void startAlignment();
  void joinAlignmentThread();
  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose) const;
  bool getRobotPoseInFrame(const std::string &frame,
                           geometry_msgs::msg::PoseStamped &pose) const;
  bool captureHomePoseIfNeeded();
  geometry_msgs::msg::PoseStamped waypointPose(
      const nav_interfaces::msg::Waypoint &waypoint) const;
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

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_main_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Service<nav_interfaces::srv::StartMission>::SharedPtr start_mission_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_dynamic_mode_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dynamic_goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hot_path_pub_;
  rclcpp::Client<nav_interfaces::srv::PlanPath>::SharedPtr plan_client_;
  rclcpp::Client<nav_interfaces::srv::ValidatePath>::SharedPtr validate_path_client_;
  rclcpp::Client<nav_interfaces::srv::PlanSequence>::SharedPtr sequence_plan_client_;
  rclcpp::Client<atomic_move::srv::AlignTo>::SharedPtr align_client_;
  rclcpp::Publisher<nav_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_interfaces::msg::MissionEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<nav_interfaces::msg::OperatorCommand>::SharedPtr operator_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_interfaces::msg::RecoveryCommand>::SharedPtr recovery_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  rclcpp::Subscription<nav_interfaces::msg::RecoveryRequest>::SharedPtr recovery_request_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  NavPathClient::SharedPtr path_client_;
  nav_msgs::msg::Path active_path_;

  State state_{State::IDLE};
  State paused_from_{State::IDLE};
  std::vector<nav_interfaces::msg::Waypoint> waypoints_;
  int current_index_{-1};
  bool loop_{false};
  bool temporary_mission_{false};
  std::vector<nav_interfaces::msg::Waypoint> paused_waypoints_;
  int paused_current_index_{-1};
  bool paused_loop_{false};
  State paused_resume_state_{State::NAVIGATING};
  std::string paused_mission_id_;
  bool returning_home_{false};
  std::string mission_id_;
  geometry_msgs::msg::Pose home_pose_;
  bool home_pose_valid_{false};
  rclcpp::Time dwell_start_{0, 0, RCL_ROS_TIME};
  double dwell_duration_{0.0};
  double dwell_remaining_{0.0};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  bool enable_alignment_{false};
  double plan_timeout_sec_{2.0};
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
  nav_interfaces::msg::RecoveryRequest recovery_pending_commit_request_;
  std::string recovery_candidate_id_;
  std::string recovery_candidate_mission_id_;
  int recovery_candidate_waypoint_index_{-1};
  int8_t recovery_candidate_side_{0};
  std::string recovery_candidate_flags_;
  nav_msgs::msg::Path recovery_candidate_path_;
  rclcpp::Time recovery_candidate_created_at_{0, 0, RCL_ROS_TIME};
  double nav_watchdog_timeout_sec_{15.0};
  double nav_stall_timeout_sec_{15.0};
  double alignment_watchdog_timeout_sec_{25.0};
  // Settle delay between publishing /neupan/stop and calling /atomic_move/align
  // to avoid the stop callback racing the new alignment request.
  double handover_settle_sec_{0.2};
  int completed_waypoints_{0};
  rclcpp::Time path_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_feedback_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_progress_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_rx_at_{0, 0, RCL_ROS_TIME};
  bool ec_perception_offline_{false};
  double last_feedback_distance_{-1.0};
  rclcpp::Time alignment_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_at_{0, 0, RCL_ROS_TIME};
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
  std::atomic<bool> path_goal_active_{false};
  std::atomic<double> distance_remaining_{-1.0};
  std::atomic<bool> replan_requested_{false};
  std::atomic<std::uint64_t> path_goal_generation_{0};
  std::string navigation_mode_{"escape"};
  double periodic_replan_sec_{2.0};
  rclcpp::Time last_periodic_replan_at_{0, 0, RCL_ROS_TIME};
  std::future<PeriodicReplanResult> periodic_replan_future_;
  std::future<PeriodicReplanResult> initial_plan_future_;
  bool initial_plan_requested_{false};
  int initial_plan_attempts_{0};
  double initial_plan_timeout_sec_{15.0};
  rclcpp::Time next_initial_plan_at_{0, 0, RCL_ROS_TIME};
  bool periodic_replan_running_{false};
  // Path-validity watchdog: when the active path corridor is blocked (install
  // gate blocked hysteresis), trigger a replan immediately instead of waiting
  // periodic_replan_sec_. Cooldown avoids a replan storm while stuck. Requires
  // install_gate enabled (blocked detection only runs when the gate is active).
  bool path_block_replan_enable_{false};
  double path_block_replan_cooldown_sec_{0.5};
  rclcpp::Time last_path_block_replan_at_{0, 0, RCL_ROS_TIME};
  bool route_segment_enable_{true};
  double route_segment_length_m_{10.0};
  double route_segment_overlap_m_{3.0};
  nav_msgs::msg::Path global_route_;
  std::size_t route_progress_index_{0};
  std::size_t route_segment_anchor_index_{0};
  std::uint64_t route_segment_generation_{0};
  bool route_segment_final_{true};
  bool route_advance_requested_{false};
  RouteTarget pending_route_target_;
  std::uint64_t pending_route_goal_generation_{0};
  rclcpp::Time pending_route_sent_at_{0, 0, RCL_ROS_TIME};
  std::atomic<std::uint64_t> accepted_path_generation_{0};
  bool path_safety_waiting_{false};
  double path_safety_scan_max_age_sec_{1.0};
  std::atomic<std::int64_t> path_safety_scan_stamp_ns_{0};
  std::atomic<std::int64_t> path_safety_scan_received_ns_{0};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr path_safety_scan_sub_;
  std::future<PathSafetyResult> path_safety_future_;
  rclcpp::Time last_path_safety_check_{0, 0, RCL_ROS_TIME};

  // Dynamic target following mode (disabled by default)
  bool enable_dynamic_mode_{false};
  bool dynamic_mode_{false};
  geometry_msgs::msg::PoseStamped dynamic_goal_;
  bool dynamic_goal_pending_{false};
  bool dynamic_goal_valid_{false};
  bool dynamic_nav_active_{false};
  rclcpp::Time last_dynamic_plan_at_{0, 0, RCL_ROS_TIME};
  double dynamic_replan_cooldown_sec_{0.5};
  std::string dynamic_goal_topic_{"/waypoint_manager/dynamic_goal"};
  int dynamic_plan_failures_{0};

  // Stepped rotation search on target loss (disabled by default)
  rclcpp::Service<nav_interfaces::srv::FollowSearch>::SharedPtr follow_search_srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan360_sub_;
  rclcpp::Client<atomic_move::srv::Turn>::SharedPtr turn_client_;
  std::thread search_thread_;
  std::atomic<bool> search_cancel_requested_{false};
  std::atomic<bool> search_running_{false};
  std::atomic<bool> search_result_pending_{false};
  std::atomic<std::uint64_t> search_generation_{0};
  std::atomic<int> search_stop_code_{0};
  std::atomic<double> all_round_clearance_{-1.0};
  std::atomic<bool> scan360_received_{false};
  bool enable_follow_search_{false};
  // Minimum all-round clearance (m) from robot center before each rotation step
  // (0.60m = robot circumradius 0.553m + 5cm margin).
  double follow_search_clearance_m_{0.60};
  double follow_search_step_deg_{90.0};
  double follow_search_step_pause_sec_{0.3};
  double follow_search_timeout_sec_{0.0};
  double follow_search_total_limit_deg_{0.0};
  std::string follow_search_turn_service_{"/atomic_move/turn"};
  std::string follow_search_scan_topic_{"/scan_360"};

  // Periodic replan install gate
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr install_gate_shadow_pub_;
  bool install_gate_enable_{false};
  bool install_gate_shadow_{false};
  bool install_gate_scan_subscribed_{false};
  mission_manager_gate::GateConfig install_gate_cfg_;
  std::unique_ptr<mission_manager_gate::BlockedHysteresis> install_gate_blocked_;
  std::vector<mission_manager_gate::Pt> install_gate_scan_map_pts_;
  rclcpp::Time install_gate_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool install_gate_scan_ready_{false};
  std::atomic<bool> install_gate_blocked_raw_{false};
  std::atomic<bool> install_gate_blocked_gated_{false};
  std::atomic<double> install_gate_min_lateral_m_{-1.0};
  double install_gate_blocked_dwell_sec_{0.0};

  // SRU local planner backend runtime state
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sru_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sru_enable_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sru_cmd_vel_sub_;
  double sru_arrive_threshold_m_{0.7};
  double sru_goal_refresh_sec_{1.0};
  double sru_goal_refresh_min_delta_m_{0.30};
  double sru_cmd_vel_silence_sec_{2.0};
  double sru_stall_progress_epsilon_m_{0.05};
  std::string sru_robot_frame_{"odin1_base_link"};
  geometry_msgs::msg::PoseStamped sru_goal_odom_;
  bool sru_goal_valid_{false};
  rclcpp::Time sru_last_refresh_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sru_last_cmd_vel_at_{0, 0, RCL_ROS_TIME};
  bool sru_cmd_vel_seen_{false};
  bool sru_silence_warned_{false};
  double sru_stall_ref_distance_{-1.0};
  rclcpp::Time sru_stall_ref_at_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_odom_pose_;
  bool last_odom_pose_valid_{false};
};

#endif
