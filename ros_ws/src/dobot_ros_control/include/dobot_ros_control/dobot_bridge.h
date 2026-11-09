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

/**
 * @file dobot_bridge.h
 * @brief /cmd_vel -> Dobot Lumo RPC bridge node (ROS 1).
 *
 * State machine (DAMPED is a terminal state requiring manual reset via /dobot/reset):
 *   INIT -> PREPARE(100) -> WALK(304) -> RUNNING(SetVel stream)
 *   RUNNING --cmd timeout--> ZERO --additional 1s--> DAMPED(1)
 *   RUNNING --estop / FSM!=WALK / action--> DAMPED or ACTION
 */
#ifndef DOBOT_ROS_CONTROL_DOBOT_BRIDGE_H
#define DOBOT_ROS_CONTROL_DOBOT_BRIDGE_H

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <set>

#include "dobot_ros_control/dobot_rpc.h"

namespace dobot_ros_control
{

/** @brief Node parameters (configurable via config/dobot_control.yaml or launch). */
struct BridgeParams
{
  // Connection
  std::string backend = "sdk";         ///< sdk: official libclient_rpc.a; raw: built-in JSON-RPC
  std::string robot_ip = "192.168.5.2";
  int robot_port = 51234;
  int connect_timeout_ms = 3000;
  int control_mode = 2;                ///< 0=gamepad, 1=APP, 2=third-party control

  // Top-level switches
  bool dry_run = true;                 ///< true: read-only GetFsmId, no velocity/mode commands
  bool auto_start = false;             ///< Automatically transition INIT->PREPARE->WALK on startup

  // Velocity and acceleration limits
  double max_linear_vel = 0.3;         ///< Forward speed limit (m/s)
  double max_linear_backward = 0.2;    ///< Backward speed limit (m/s)
  double max_angular_vel = 0.5;        ///< Yaw rate limit (rad/s)
  double max_accel = 0.4;              ///< Linear acceleration limit (m/s^2)
  double max_yaw_accel = 1.0;          ///< Angular acceleration limit (rad/s^2)
  bool enable_strafe = false;          ///< Lateral motion disabled (vy = 0)

  // Command freshness
  double cmd_vel_timeout = 0.4;        ///< Timeout before zeroing velocity (s)
  double cmd_vel_decay_window = 0.15;  ///< Linear decay window before timeout (s), 0=disabled
  double cmd_vel_damp_timeout = 1.0;   ///< Timeout after zeroing velocity before entering damping (s)
  double setvel_duration_s = 0.005;    ///< Per-command SetVel duration (s)

  // Deadband
  double deadband_lin = 0.10;          ///< Linear velocity deadband threshold (m/s)
  double min_effective_lin = 0.10;     ///< Minimum effective linear velocity for boost policy (m/s)
  /// zero: send 0 below deadband (default) | boost: clamp up to min_effective_lin
  std::string below_deadband_policy = "zero";
  double deadband_ang = 0.05;

  // Control loop rates
  double control_rate = 20.0;          ///< SetVel dispatch frequency (Hz)
  double fsm_watch_hz = 2.0;           ///< GetFsmId polling frequency (Hz)
  int fsm_watch_fail_limit = 3;        ///< Consecutive failures before entering DAMPED

  // FSM state IDs
  int fsm_damping = 1;                 ///< Damping state (emergency stop target)
  int fsm_standing = 300;              ///< Standing state (action playback allowed, no walking)
  int fsm_prepare = 100;               ///< Prepare standing state
  int fsm_walk = 304;                  ///< Humanoid walking state (required for velocity commands)
  bool require_walk_state = true;      ///< Reject velocity commands when not in WALK state

  // ---- Mode ownership ----
  // Only transitions between standing (300) and walking (304) are permitted when
  // allow_walk_switch=true; states in forbidden_fsm_csv are never entered by code.
  bool allow_walk_switch = true;    ///< Allow automatic 300 <-> 304 transitions
  int fsm_switch_guard_ms = 800;    ///< Minimum guard interval (ms) after switching to 304

  /// Comma-separated FSM IDs that code must never command
  std::string forbidden_fsm_csv = "1,2,100,310,311,312,313";

  // ---- Upper-body actions ----
  std::string action_topic = "/dobot/action";
  std::vector<std::string> action_whitelist;  ///< Empty = unrestricted
  bool action_requires_standing = true;       ///< Switch to standing before playing action
  double action_max_sec = 30.0;               ///< Maximum wait time per action (s)
  double action_poll_hz = 5.0;

  // Topics
  std::string cmd_vel_topic = "/cmd_vel";
  std::string estop_topic = "/estop";
  std::string state_topic = "/base/state";
  bool publish_state = true;
};

/** @brief Runtime bridge states. */
enum class BridgeState
{
  kInit = 0,
  kPrepare,
  kWalk,
  kRunning,
  kPaused,   ///< Non-walk FSM state: velocity output paused without altering robot mode
  kZero,
  kAction,   ///< Executing upper-body action
  kDamped,   ///< Terminal damped state requiring manual reset
  kFault,    ///< Unrecoverable link or configuration fault
};

const char* stateString(BridgeState s);

/** @brief Diagnostic error codes. */
enum class ErrCode
{
  kOk = 0,
  kRpcUnreachable = 1,     ///< DOBOT_E001 RPC service unreachable or connection failed
  kNotWalkState = 2,       ///< DOBOT_E002 FSM not in walking state
  kEstop = 3,              ///< DOBOT_E003 Emergency stop triggered (damped terminal state)
  kTakeover = 4,           ///< DOBOT_E004 Suspected manual takeover (FSM switched externally)
  kCmdTimeout = 5,         ///< DOBOT_E005 Command timeout
  kStalled = 6,            ///< DOBOT_E006 Non-zero command with near-zero displacement
  kFsmUnconfigured = 7,    ///< DOBOT_E007 FSM state ID not configured
  kActionFailed = 8,       ///< DOBOT_E008 Action execution failed
  kStaleData = 9,          ///< DOBOT_E009 Stale state data
  kBelowDeadband = 10,     ///< DOBOT_E010 Command below walking deadband
};

const char* errCodeString(ErrCode c);

class DobotBridge
{
public:
  DobotBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~DobotBridge();

  DobotBridge(const DobotBridge&) = delete;
  DobotBridge& operator=(const DobotBridge&) = delete;

  /** @brief Load parameters, establish RPC connection, and register ROS interfaces. */
  bool init();

  /** @brief Main loop: dispatches velocity at control_rate and polls FSM at fsm_watch_hz. */
  void spin();

private:
  // Callbacks
  void onCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
  void onEstop(const std_msgs::Bool::ConstPtr& msg);
  void onReset(const std_msgs::Bool::ConstPtr& msg);
  void onAction(const std_msgs::String::ConstPtr& msg);

  // State transitions
  bool enterPrepare();
  bool enterWalk();
  void enterDamped(ErrCode why);
  void resetFromDamped();
  RpcCode sendZero();
  RpcCode sendVel(float vx, float vyaw);
  void startAction(const std::string& name);
  void tickAction();

  // Helpers
  double nowSec() const;
  bool fsmConfigured() const { return p_.fsm_walk >= 0; }
  void publishState();
  void publishDiagnostic(uint8_t level, const std::string& name, const std::string& msg);
  void setErr(ErrCode c) { err_ = c; }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  BridgeParams p_;

  std::unique_ptr<RpcClient> rpc_;
  BridgeState state_ = BridgeState::kInit;
  std::set<int> forbidden_fsm_;          ///< Parsed from forbidden_fsm_csv
  ros::Time walk_switch_stamp_;          ///< Timestamp of last 300->304 transition
  ErrCode err_ = ErrCode::kOk;

  // Command freshness
  std::mutex cmd_mu_;
  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_stamp_;
  bool has_cmd_ = false;

  // Acceleration-limited output velocities
  double cur_vx_ = 0.0;
  double cur_vyaw_ = 0.0;
  ros::Time last_tick_;
  bool below_deadband_ = false;
  double below_deadband_vx_ = 0.0;

  // FSM watchdog
  int32_t last_fsm_id_ = -1;
  int fsm_fail_count_ = 0;
  ros::Time zero_since_;
  ros::Time damp_since_;

  // Action playback
  std::string action_name_;
  ros::Time action_start_;

  // ROS pubs/subs
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_estop_;
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_action_;
  ros::Publisher pub_state_;
  ros::Publisher pub_estop_state_;
  ros::Publisher pub_action_state_;
};

}  // namespace dobot_ros_control

#endif  // DOBOT_ROS_CONTROL_DOBOT_BRIDGE_H
