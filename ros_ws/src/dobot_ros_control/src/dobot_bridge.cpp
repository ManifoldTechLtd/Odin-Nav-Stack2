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
 * @file dobot_bridge.cpp
 * @brief /cmd_vel -> Dobot Lumo RPC bridge implementation.
 *
 * Key constraints:
 *   - Lateral velocity vy is fixed to 0 (no strafing support in WALK mode 304).
 *   - Velocity commands only take effect in the WALK FSM state (304).
 *   - Health monitoring relies on polling GetFsmId; leaving WALK triggers damping.
 *   - DAMPED is a terminal state requiring manual reset via /dobot/reset.
 */
#include "dobot_ros_control/dobot_bridge.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace dobot_ros_control
{

const char* stateString(BridgeState s)
{
  switch (s)
  {
    case BridgeState::kInit: return "INIT";
    case BridgeState::kPrepare: return "PREPARE";
    case BridgeState::kWalk: return "WALK";
    case BridgeState::kRunning: return "RUNNING";
    case BridgeState::kPaused: return "PAUSED";
    case BridgeState::kZero: return "ZERO";
    case BridgeState::kAction: return "ACTION";
    case BridgeState::kDamped: return "DAMPED";
    case BridgeState::kFault: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char* errCodeString(ErrCode c)
{
  switch (c)
  {
    case ErrCode::kOk: return "DOBOT_E000 OK";
    case ErrCode::kRpcUnreachable: return "DOBOT_E001 RPC_UNREACHABLE";
    case ErrCode::kNotWalkState: return "DOBOT_E002 NOT_WALK_STATE";
    case ErrCode::kEstop: return "DOBOT_E003 ESTOP";
    case ErrCode::kTakeover: return "DOBOT_E004 TAKEOVER_SUSPECTED";
    case ErrCode::kCmdTimeout: return "DOBOT_E005 CMD_TIMEOUT";
    case ErrCode::kStalled: return "DOBOT_E006 PROGRESS_STALLED";
    case ErrCode::kFsmUnconfigured: return "DOBOT_E007 FSM_UNCONFIGURED";
    case ErrCode::kActionFailed: return "DOBOT_E008 ACTION_FAILED";
    case ErrCode::kStaleData: return "DOBOT_E009 STATE_STALE";
    default: return "DOBOT_E999 UNKNOWN";
  }
}

namespace
{
double clampAbs(double v, double lim)
{
  if (!std::isfinite(v)) return 0.0;
  return std::max(-lim, std::min(lim, v));
}
double rateLimit(double target, double current, double max_delta)
{
  const double d = target - current;
  if (std::fabs(d) <= max_delta) return target;
  return current + (d > 0 ? max_delta : -max_delta);
}
}  // namespace

DobotBridge::DobotBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {}

DobotBridge::~DobotBridge() = default;

bool DobotBridge::init()
{
  // Load parameters
  pnh_.param<std::string>("backend", p_.backend, p_.backend);
  pnh_.param<std::string>("robot_ip", p_.robot_ip, p_.robot_ip);
  pnh_.param("robot_port", p_.robot_port, p_.robot_port);
  pnh_.param("connect_timeout_ms", p_.connect_timeout_ms, p_.connect_timeout_ms);
  pnh_.param("control_mode", p_.control_mode, p_.control_mode);

  pnh_.param("dry_run", p_.dry_run, p_.dry_run);
  pnh_.param("auto_start", p_.auto_start, p_.auto_start);

  pnh_.param("max_linear_vel", p_.max_linear_vel, p_.max_linear_vel);
  pnh_.param("max_linear_backward", p_.max_linear_backward, p_.max_linear_backward);
  pnh_.param("max_angular_vel", p_.max_angular_vel, p_.max_angular_vel);
  pnh_.param("max_accel", p_.max_accel, p_.max_accel);
  pnh_.param("max_yaw_accel", p_.max_yaw_accel, p_.max_yaw_accel);
  pnh_.param("enable_strafe", p_.enable_strafe, p_.enable_strafe);

  pnh_.param("cmd_vel_timeout", p_.cmd_vel_timeout, p_.cmd_vel_timeout);
  pnh_.param("cmd_vel_decay_window", p_.cmd_vel_decay_window, p_.cmd_vel_decay_window);
  pnh_.param("cmd_vel_damp_timeout", p_.cmd_vel_damp_timeout, p_.cmd_vel_damp_timeout);
  pnh_.param("setvel_duration_s", p_.setvel_duration_s, p_.setvel_duration_s);
  pnh_.param("deadband_lin", p_.deadband_lin, p_.deadband_lin);
  pnh_.param("min_effective_lin", p_.min_effective_lin, p_.min_effective_lin);
  pnh_.param<std::string>("below_deadband_policy", p_.below_deadband_policy, p_.below_deadband_policy);
  pnh_.param("deadband_ang", p_.deadband_ang, p_.deadband_ang);

  pnh_.param("control_rate", p_.control_rate, p_.control_rate);
  pnh_.param("fsm_watch_hz", p_.fsm_watch_hz, p_.fsm_watch_hz);
  pnh_.param("fsm_watch_fail_limit", p_.fsm_watch_fail_limit, p_.fsm_watch_fail_limit);

  pnh_.param("fsm_damping", p_.fsm_damping, p_.fsm_damping);
  pnh_.param("fsm_standing", p_.fsm_standing, p_.fsm_standing);
  pnh_.param("fsm_prepare", p_.fsm_prepare, p_.fsm_prepare);
  pnh_.param("fsm_walk", p_.fsm_walk, p_.fsm_walk);
  pnh_.param("require_walk_state", p_.require_walk_state, p_.require_walk_state);

  // ---- Mode ownership: only allow 300 (standing) <-> 304 (walking) ----
  pnh_.param("allow_walk_switch", p_.allow_walk_switch, p_.allow_walk_switch);
  pnh_.param("fsm_switch_guard_ms", p_.fsm_switch_guard_ms, p_.fsm_switch_guard_ms);
  pnh_.param("forbidden_fsm_csv", p_.forbidden_fsm_csv, p_.forbidden_fsm_csv);
  {
    std::stringstream ss(p_.forbidden_fsm_csv);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      try { forbidden_fsm_.insert(std::stoi(item)); } catch (...) {}
    }
    std::string join;
    for (int v : forbidden_fsm_) join += (join.empty() ? "" : ",") + std::to_string(v);
    ROS_INFO("[dobot] forbidden FSM (never switch into): [%s]", join.c_str());
    ROS_INFO("[dobot] allow_walk_switch=%s (300<->304 only)", p_.allow_walk_switch ? "true" : "false");
  }

  pnh_.param<std::string>("action_topic", p_.action_topic, p_.action_topic);
  pnh_.param("action_requires_standing", p_.action_requires_standing, p_.action_requires_standing);
  pnh_.param("action_max_sec", p_.action_max_sec, p_.action_max_sec);
  pnh_.param("action_poll_hz", p_.action_poll_hz, p_.action_poll_hz);
  if (pnh_.hasParam("action_whitelist"))
  {
    XmlRpc::XmlRpcValue v;
    pnh_.getParam("action_whitelist", v);
    p_.action_whitelist.clear();
    if (v.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < v.size(); ++i)
        if (v[i].getType() == XmlRpc::XmlRpcValue::TypeString)
          p_.action_whitelist.push_back(static_cast<std::string>(v[i]));
    }
  }

  pnh_.param<std::string>("cmd_vel_topic", p_.cmd_vel_topic, p_.cmd_vel_topic);
  pnh_.param<std::string>("estop_topic", p_.estop_topic, p_.estop_topic);
  pnh_.param<std::string>("state_topic", p_.state_topic, p_.state_topic);
  pnh_.param("publish_state", p_.publish_state, p_.publish_state);

  if (p_.control_rate <= 0.0 || p_.fsm_watch_hz <= 0.0)
  {
    ROS_ERROR("[dobot] control_rate/fsm_watch_hz must be > 0");
    return false;
  }

  // Validate configuration
  if (!rpcBackendAvailable(p_.backend))
  {
    ROS_WARN("[dobot] backend=%s not available in this binary (SDK not built) -> falling back to raw", p_.backend.c_str());
    p_.backend = "raw";
  }
  if (!fsmConfigured() && !p_.dry_run)
  {
    ROS_ERROR("[dobot] fsm_walk is not configured (current %d) and dry_run=false -> refuse to start.\n"
              "  Run `dobot_cmd_vel_bridge --getfsm <ip> 51234` first to find the walkable FSM id\n"
              "  (300/304 or 301/302), then set fsm_prepare / fsm_walk in config/dobot_control.yaml.",
              p_.fsm_walk);
    state_ = BridgeState::kFault;
    setErr(ErrCode::kFsmUnconfigured);
    return false;
  }
  if (!p_.enable_strafe)
  {
    ROS_INFO("[dobot] enable_strafe=false -> linear.y is ignored (Lumo cannot strafe)");
  }

  // Initialize RPC connection
  rpc_ = makeRpcClient(p_.backend);
  std::string err;
  if (!rpc_->connect(p_.robot_ip, p_.robot_port, p_.connect_timeout_ms, &err))
  {
    if (p_.dry_run)
    {
      ROS_WARN("[dobot] RPC connect failed (backend=%s, %s:%d): %s", rpc_->backendName(),
               p_.robot_ip.c_str(), p_.robot_port, err.c_str());
      ROS_WARN("[dobot] dry_run=true -> keep running, publish /base/state only, retry GetFsmId");
      setErr(ErrCode::kRpcUnreachable);
    }
    else
    {
      ROS_ERROR("[dobot] RPC connect failed (backend=%s, %s:%d): %s", rpc_->backendName(),
                p_.robot_ip.c_str(), p_.robot_port, err.c_str());
      setErr(ErrCode::kRpcUnreachable);
      state_ = BridgeState::kFault;
      return false;
    }
  }
  else
  {
    ROS_INFO("[dobot] RPC connected: backend=%s %s:%d", rpc_->backendName(), p_.robot_ip.c_str(),
             p_.robot_port);
    setErr(ErrCode::kOk);
  }

  // Query initial FSM state
  int32_t fsm_now = -1;
  RpcCode rcf = rpc_->getFsmId(&fsm_now);
  if (rcf == RpcCode::kOk)
  {
    last_fsm_id_ = fsm_now;
    ROS_INFO("[dobot] current FSM id = %d", fsm_now);
  }
  else
  {
    ROS_WARN("[dobot] GetFsmId failed: %s", rpcCodeString(rcf));
  }

  if (!p_.dry_run)
  {
    // Only claim third-party control mode when auto_start=true; calling SetControlMode(2)
    // when the operator already placed the robot in WALK via gamepad resets it to DAMPING(1).
    if (p_.auto_start)
    {
      RpcCode rcm = rpc_->setControlMode(p_.control_mode);
      if (rcm != RpcCode::kOk)
        ROS_WARN("[dobot] SetControlMode(%d) returned %s (may be unnecessary; see 06_* Q6)", p_.control_mode,
                 rpcCodeString(rcm));
      else
        ROS_INFO("[dobot] control mode set to %d (0=joystick 1=APP 2=third-party)", p_.control_mode);
    }
    else
    {
      ROS_INFO("[dobot] auto_start=false: keep joystick authority (skip SetControlMode)");
    }
  }

  // Register subscribers and publishers
  sub_cmd_ = nh_.subscribe(p_.cmd_vel_topic, 1, &DobotBridge::onCmdVel, this);
  sub_estop_ = nh_.subscribe(p_.estop_topic, 1, &DobotBridge::onEstop, this);
  sub_reset_ = nh_.subscribe("/dobot/reset", 1, &DobotBridge::onReset, this);
  sub_action_ = nh_.subscribe(p_.action_topic, 1, &DobotBridge::onAction, this);

  pub_state_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(p_.state_topic, 1);
  pub_estop_state_ = nh_.advertise<std_msgs::Bool>("/estop_state", 1, /*latch=*/true);
  pub_action_state_ = nh_.advertise<std_msgs::String>("/dobot/action_state", 1, /*latch=*/true);

  last_tick_ = ros::Time::now();
  state_ = BridgeState::kInit;

  if (p_.dry_run)
  {
    ROS_WARN("[dobot] dry_run=true: read-only (GetFsmId only), no SetVel / no FSM switch");
  }
  else if (p_.auto_start)
  {
    if (!enterPrepare() || !enterWalk())
    {
      ROS_ERROR("[dobot] auto start failed, stuck at %s", stateString(state_));
    }
  }
  else
  {
    int32_t cur = -1;
    if (rpc_->getFsmId(&cur) == RpcCode::kOk && cur == p_.fsm_walk)
    {
      ROS_INFO("[dobot] auto_start=false but robot already in WALK(%d): entering RUNNING directly", cur);
      state_ = BridgeState::kRunning;
    }
    else
    {
      state_ = BridgeState::kPaused;
      ROS_INFO("[dobot] auto_start=false: robot FSM=%d (not walk) -> PAUSED; will enter RUNNING once FSM=304",
               cur);
      ROS_INFO("[dobot]   switch to WALK with the gamepad, or just send cmd_vel (auto 300->304 if allow_walk_switch)");
    }
  }
  return true;
}

void DobotBridge::onCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(cmd_mu_);
  last_cmd_ = *msg;
  last_cmd_stamp_ = ros::Time::now();
  has_cmd_ = true;
}

void DobotBridge::onEstop(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) return;
  // Emergency stop zeroes velocity and enters PAUSED without switching to DAMPING(1).
  ROS_WARN("[dobot] /estop=true -> zero velocity + PAUSED (mode left untouched)");
  sendZero();
  setErr(ErrCode::kEstop);
  state_ = BridgeState::kPaused;
  {
    std::lock_guard<std::mutex> lk(cmd_mu_);
    has_cmd_ = false;
  }
  ROS_WARN("[dobot] cleared pending cmd; publish /dobot/reset true or send fresh cmd_vel to resume (FSM=300 only)");
}

void DobotBridge::onReset(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) return;
  if (state_ == BridgeState::kDamped || state_ == BridgeState::kFault)
  {
    ROS_WARN("[dobot] manual reset requested (current %s)", stateString(state_));
    resetFromDamped();
  }
  else if (p_.auto_start)
  {
    enterPrepare();
    enterWalk();
  }
}

void DobotBridge::onAction(const std_msgs::String::ConstPtr& msg)
{
  startAction(msg->data);
}

double DobotBridge::nowSec() const { return ros::Time::now().toSec(); }

bool DobotBridge::enterPrepare()
{
  if (p_.dry_run) return false;
  if (p_.fsm_prepare < 0)
  {
    ROS_WARN("[dobot] fsm_prepare not configured -> skip PREPARE, try WALK directly");
  }
  else
  {
    RpcCode rc = rpc_->setFsmId(p_.fsm_prepare);
    if (rc != RpcCode::kOk)
    {
      ROS_ERROR("[dobot] SetFsmId(%d=prepare) failed: %s", p_.fsm_prepare, rpcCodeString(rc));
      enterDamped(ErrCode::kNotWalkState);
      return false;
    }
    ROS_INFO("[dobot] switched to PREPARE(%d)", p_.fsm_prepare);
    state_ = BridgeState::kPrepare;
    ros::Duration(0.5).sleep();
  }
  return true;
}

bool DobotBridge::enterWalk()
{
  if (p_.dry_run) return false;
  if (p_.fsm_walk < 0)
  {
    ROS_ERROR("[dobot] fsm_walk not configured -> cannot enter walking state");
    setErr(ErrCode::kFsmUnconfigured);
    return false;
  }
  RpcCode rc = rpc_->setFsmId(p_.fsm_walk);
  if (rc != RpcCode::kOk)
  {
    ROS_ERROR("[dobot] SetFsmId(%d=walk) failed: %s", p_.fsm_walk, rpcCodeString(rc));
    enterDamped(ErrCode::kNotWalkState);
    return false;
  }
  int32_t fsm_now = -1;
  if (rpc_->getFsmId(&fsm_now) == RpcCode::kOk)
  {
    last_fsm_id_ = fsm_now;
    if (fsm_now != p_.fsm_walk)
    {
      ROS_WARN("[dobot] FSM readback=%d after WALK switch (expected %d) -- may be rejected or taken over", fsm_now, p_.fsm_walk);
    }
  }
  ROS_INFO("[dobot] switched to WALK(%d), entering RUNNING", p_.fsm_walk);
  state_ = BridgeState::kRunning;
  return true;
}

void DobotBridge::enterDamped(ErrCode why)
{
  if (p_.dry_run)
  {
    state_ = BridgeState::kZero;
    return;
  }
  rpc_->setVel(0.0f, 0.0f, 0.0f, static_cast<float>(p_.setvel_duration_s));
  if (p_.fsm_damping >= 0)
  {
    RpcCode rc = rpc_->setFsmId(p_.fsm_damping);
    if (rc != RpcCode::kOk)
      ROS_ERROR("[dobot] SetFsmId(%d=damping) failed: %s", p_.fsm_damping, rpcCodeString(rc));
  }
  state_ = BridgeState::kDamped;
  if (err_ == ErrCode::kOk) setErr(why);
  damp_since_ = ros::Time::now();
  ROS_ERROR("[dobot] entered DAMPED (%s); manual reset required via /dobot/reset", errCodeString(err_));
  std_msgs::Bool b;
  b.data = true;
  pub_estop_state_.publish(b);
}

void DobotBridge::resetFromDamped()
{
  if (p_.dry_run) return;
  ROS_WARN("[dobot] reset: DAMPED -> PREPARE -> WALK");
  setErr(ErrCode::kOk);
  std_msgs::Bool b;
  b.data = false;
  pub_estop_state_.publish(b);
  has_cmd_ = false;
  cur_vx_ = cur_vyaw_ = 0.0;
  state_ = BridgeState::kInit;
  if (!enterPrepare() || !enterWalk()) return;
}

RpcCode DobotBridge::sendZero()
{
  if (p_.dry_run) return RpcCode::kOk;
  return rpc_->setVel(0.0f, 0.0f, 0.0f, static_cast<float>(p_.setvel_duration_s));
}

RpcCode DobotBridge::sendVel(float vx, float vyaw)
{
  if (p_.dry_run) return RpcCode::kOk;
  return rpc_->setVel(vx, 0.0f, vyaw, static_cast<float>(p_.setvel_duration_s));
}

void DobotBridge::startAction(const std::string& name)
{
  if (name.empty()) return;
  if (!p_.action_whitelist.empty())
  {
    const bool ok = std::find(p_.action_whitelist.begin(), p_.action_whitelist.end(), name) !=
                    p_.action_whitelist.end();
    if (!ok)
    {
      ROS_ERROR("[dobot] action '%s' not in action_whitelist, rejected", name.c_str());
      setErr(ErrCode::kActionFailed);
      return;
    }
  }
  if (state_ == BridgeState::kDamped || state_ == BridgeState::kFault)
  {
    ROS_ERROR("[dobot] state=%s, refuse to play action", stateString(state_));
    return;
  }
  if (p_.dry_run)
  {
    ROS_WARN("[dobot] dry_run=true -> not playing action '%s'", name.c_str());
    return;
  }

  sendZero();
  if (p_.action_requires_standing && p_.fsm_standing >= 0)
  {
    RpcCode rc = rpc_->setFsmId(p_.fsm_standing);
    if (rc != RpcCode::kOk)
      ROS_WARN("[dobot] set standing(%d) failed: %s; trying action anyway", p_.fsm_standing, rpcCodeString(rc));
  }
  RpcCode rc = rpc_->executeAction(name);
  if (rc != RpcCode::kOk)
  {
    ROS_ERROR("[dobot] executeAction('%s') failed: %s", name.c_str(), rpcCodeString(rc));
    setErr(ErrCode::kActionFailed);
    return;
  }
  action_name_ = name;
  action_start_ = ros::Time::now();
  state_ = BridgeState::kAction;
  ROS_INFO("[dobot] start action '%s' (max wait %.0fs)", name.c_str(), p_.action_max_sec);

  std_msgs::String s;
  s.data = name;
  pub_action_state_.publish(s);
}

void DobotBridge::tickAction()
{
  if (state_ != BridgeState::kAction) return;
  const double elapsed = (ros::Time::now() - action_start_).toSec();
  int32_t fsm_now = -1;
  const bool fsm_ok = (rpc_->getFsmId(&fsm_now) == RpcCode::kOk);
  if (fsm_ok) last_fsm_id_ = fsm_now;

  // Complete action on timeout or when FSM returns to WALK state
  const bool timeout = elapsed > p_.action_max_sec;
  const bool fsm_moved_on = fsm_ok && p_.fsm_walk >= 0 && fsm_now == p_.fsm_walk;
  if (timeout || fsm_moved_on)
  {
    ROS_INFO("[dobot] action '%s' finished (%s, %.1fs)", action_name_.c_str(),
             timeout ? "timeout" : "FSM back to walking", elapsed);
    action_name_.clear();
    std_msgs::String s;
    s.data = "";
    pub_action_state_.publish(s);
    if (fsm_moved_on || p_.fsm_walk < 0)
    {
      state_ = (p_.fsm_walk >= 0) ? BridgeState::kRunning : BridgeState::kZero;
    }
    else
    {
      enterWalk();
    }
  }
}

void DobotBridge::publishDiagnostic(uint8_t level, const std::string& name, const std::string& msg)
{
  if (!p_.publish_state) return;
  diagnostic_msgs::DiagnosticArray arr;
  arr.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus st;
  st.level = level;
  st.name = name;
  st.message = msg;
  st.hardware_id = "dobot_lumo";
  arr.status.push_back(st);
  pub_state_.publish(arr);
}

void DobotBridge::publishState()
{
  if (!p_.publish_state) return;
  diagnostic_msgs::DiagnosticArray arr;
  arr.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus st;
  st.name = "dobot_ros_control";
  st.hardware_id = "dobot_lumo";

  auto kv = [&st](const std::string& k, const std::string& v) {
    diagnostic_msgs::KeyValue p;
    p.key = k;
    p.value = v;
    st.values.push_back(p);
  };

  st.level = (state_ == BridgeState::kDamped || state_ == BridgeState::kFault)
                 ? diagnostic_msgs::DiagnosticStatus::ERROR
                 : (state_ == BridgeState::kZero ? diagnostic_msgs::DiagnosticStatus::WARN
                                                 : diagnostic_msgs::DiagnosticStatus::OK);
  st.message = std::string(stateString(state_)) + " | " + errCodeString(err_);

  kv("state", stateString(state_));
  kv("err", errCodeString(err_));
  kv("backend", rpc_ ? rpc_->backendName() : "none");
  kv("fsm_id", std::to_string(last_fsm_id_));
  kv("fsm_walk_cfg", std::to_string(p_.fsm_walk));
  kv("dry_run", p_.dry_run ? "true" : "false");
  kv("cmd_vx", std::to_string(cur_vx_));
  kv("cmd_vyaw", std::to_string(cur_vyaw_));
  kv("action", action_name_);
  kv("below_deadband", below_deadband_ ? "true" : "false");
  kv("below_deadband_vx", std::to_string(below_deadband_vx_));
  kv("setvel_duration_s", std::to_string(p_.setvel_duration_s));
  arr.status.push_back(st);
  pub_state_.publish(arr);
}

void DobotBridge::spin()
{
  const ros::Duration dt(1.0 / p_.control_rate);
  const ros::Duration watch_dt(1.0 / p_.fsm_watch_hz);
  ros::Time next_watch = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    const double tick = std::max(1e-3, (now - last_tick_).toSec());
    last_tick_ = now;

    // 1) FSM watchdog and link heartbeat
    if (now >= next_watch)
    {
      next_watch = now + watch_dt;
      int32_t fsm_now = -1;
      RpcCode rc = rpc_->getFsmId(&fsm_now);
      if (rc == RpcCode::kOk)
      {
        fsm_fail_count_ = 0;
        if (fsm_now != last_fsm_id_)
        {
          ROS_INFO("[dobot] FSM %d -> %d", last_fsm_id_, fsm_now);
          last_fsm_id_ = fsm_now;
        }
        // Pause velocity output when FSM leaves WALK without altering the robot's mode
        if (p_.require_walk_state && p_.fsm_walk >= 0)
        {
          if (state_ == BridgeState::kRunning && fsm_now != p_.fsm_walk)
          {
            const bool forbidden = forbidden_fsm_.count(fsm_now) > 0;
            if (forbidden)
            {
              ROS_WARN("[dobot] human switched to FSM=%d (forbidden set) -> PAUSE only, no mode change",
                       fsm_now);
              setErr(ErrCode::kTakeover);
            }
            else
            {
              ROS_INFO("[dobot] FSM=%d (not walk) -> PAUSE; will resume on WALK or fresh cmd_vel", fsm_now);
              setErr(ErrCode::kNotWalkState);
            }
            state_ = BridgeState::kPaused;
          }
          else if (state_ == BridgeState::kPaused && fsm_now == p_.fsm_walk)
          {
            ROS_INFO("[dobot] FSM back to WALK(%d) -> RUNNING", fsm_now);
            setErr(ErrCode::kOk);
            state_ = BridgeState::kRunning;
          }
        }
      }
      else
      {
        ++fsm_fail_count_;
        if (fsm_fail_count_ == p_.fsm_watch_fail_limit)
        {
          ROS_ERROR("[dobot] GetFsmId failed %d times in a row (%s)", fsm_fail_count_,
                    rpcCodeString(rc));
          setErr(ErrCode::kRpcUnreachable);
          if (!p_.dry_run)
          {
            ROS_ERROR("[dobot] -> PAUSED (RPC link lost); mode left untouched");
            state_ = BridgeState::kPaused;
          }
        }
        else if (fsm_fail_count_ > p_.fsm_watch_fail_limit)
        {
          ROS_WARN_THROTTLE(10.0, "[dobot] GetFsmId still failing (count=%d, %s)", fsm_fail_count_,
                            rpcCodeString(rc));
        }
      }
    }

    // 2) Action in progress: poll without dispatching velocity
    if (state_ == BridgeState::kAction)
    {
      tickAction();
      publishState();
      dt.sleep();
      continue;
    }

    // 3) Velocity dispatch
    if (p_.dry_run)
    {
      publishState();
      dt.sleep();
      continue;
    }

    double want_vx = 0.0;
    double want_vyaw = 0.0;
    bool have = false;
    bool cmd_nonzero = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mu_);
      if (has_cmd_)
      {
        const double age = (now - last_cmd_stamp_).toSec();
        have = true;
        want_vx = last_cmd_.linear.x;
        want_vyaw = last_cmd_.angular.z;
        cmd_nonzero = (std::fabs(last_cmd_.linear.x) > 1e-6 || std::fabs(last_cmd_.angular.z) > 1e-6);
        // 1) Velocity limits
        want_vx = clampAbs(want_vx, want_vx >= 0 ? p_.max_linear_vel : p_.max_linear_backward);
        want_vyaw = clampAbs(want_vyaw, p_.max_angular_vel);
        // 2) Deadband handling
        if (std::fabs(want_vx) < p_.deadband_lin && want_vx != 0.0)
        {
          below_deadband_ = true;
          below_deadband_vx_ = want_vx;
          if (p_.below_deadband_policy == "boost")
          {
            want_vx = (want_vx > 0.0 ? 1.0 : -1.0) * p_.min_effective_lin;
          }
          else
          {
            want_vx = 0.0;
          }
        }
        else
        {
          below_deadband_ = false;
          below_deadband_vx_ = 0.0;
        }
        if (std::fabs(want_vyaw) < p_.deadband_ang) want_vyaw = 0.0;
        // 3) Timeout and decay
        if (age > p_.cmd_vel_timeout) { want_vx = 0.0; want_vyaw = 0.0; }
        else if (p_.cmd_vel_decay_window > 0.0 && age > p_.cmd_vel_timeout - p_.cmd_vel_decay_window)
        {
          const double k = std::max(0.0, (p_.cmd_vel_timeout - age) / p_.cmd_vel_decay_window);
          want_vx *= k;
          want_vyaw *= k;
        }
      }
    }

    if (state_ == BridgeState::kPaused)
    {
      // Automatically transition 300 (standing) -> 304 (walking) when a fresh non-zero command arrives
      const bool fresh = have && (now - last_cmd_stamp_).toSec() < p_.cmd_vel_timeout;
      if (p_.allow_walk_switch && fresh && cmd_nonzero && last_fsm_id_ == p_.fsm_standing &&
          (now - walk_switch_stamp_).toSec() * 1000.0 > p_.fsm_switch_guard_ms)
      {
        walk_switch_stamp_ = now;
        if (rpc_->setFsmId(p_.fsm_walk) == RpcCode::kOk)
        {
          state_ = BridgeState::kRunning;
          setErr(ErrCode::kOk);
          ROS_INFO("[dobot] fresh cmd_vel + FSM=%d -> switch to WALK(%d) [300<->304 only]", last_fsm_id_,
                   p_.fsm_walk);
        }
        else
        {
          ROS_WARN_THROTTLE(3.0, "[dobot] auto 300->304 rejected; staying PAUSED");
        }
      }
    }
    else if (state_ == BridgeState::kRunning)
    {
      // 4) Acceleration limits
      const double dvx = p_.max_accel * tick;
      const double dw = p_.max_yaw_accel * tick;
      cur_vx_ = rateLimit(want_vx, cur_vx_, dvx);
      cur_vyaw_ = rateLimit(want_vyaw, cur_vyaw_, dw);

      RpcCode rc = sendVel(static_cast<float>(cur_vx_), static_cast<float>(cur_vyaw_));
      if (rc != RpcCode::kOk && rc != RpcCode::kSetVelFailed)
      {
        ROS_WARN_THROTTLE(2.0, "[dobot] SetVel failed: %s", rpcCodeString(rc));
      }
      else if (rc == RpcCode::kSetVelFailed)
      {
        ROS_WARN("[dobot] SetVel rejected (not walkable) -> PAUSE, mode left untouched");
        setErr(ErrCode::kNotWalkState);
        state_ = BridgeState::kPaused;
      }

      // 5) Command timeout -> ZERO -> DAMPED
      if (below_deadband_)
      {
        ROS_WARN_THROTTLE(5.0,
                          "[dobot] cmd below deadband (%.3f < %.3f m/s, policy=%s): Lumo will NOT move. "
                          "Tune min_effective_lin / below_deadband_policy if this persists.",
                          below_deadband_vx_, p_.deadband_lin, p_.below_deadband_policy.c_str());
      }
      if (!have || (!std::isfinite(want_vx) && !std::isfinite(want_vyaw)))
      {
        if (state_ == BridgeState::kRunning && (now - last_cmd_stamp_).toSec() > p_.cmd_vel_timeout)
        {
          sendZero();
          state_ = BridgeState::kZero;
          zero_since_ = now;
          setErr(ErrCode::kCmdTimeout);
          ROS_WARN("[dobot] cmd_vel timeout %.2fs -> ZERO", p_.cmd_vel_timeout);
        }
      }
    }
    else if (state_ == BridgeState::kZero)
    {
      sendZero();
      if ((now - zero_since_).toSec() > p_.cmd_vel_damp_timeout)
      {
        ROS_INFO("[dobot] zero held %.2fs -> PAUSED (mode left untouched)", p_.cmd_vel_damp_timeout);
        state_ = BridgeState::kPaused;
        setErr(ErrCode::kOk);
      }
      else
      {
        std::lock_guard<std::mutex> lk(cmd_mu_);
        if (has_cmd_ && (now - last_cmd_stamp_).toSec() < p_.cmd_vel_timeout)
        {
          state_ = BridgeState::kRunning;
          setErr(ErrCode::kOk);
          ROS_INFO("[dobot] cmd_vel resumed -> RUNNING");
        }
      }
    }

    publishState();
    dt.sleep();
  }

  // Shutdown: zero velocity and switch to damping
  if (!p_.dry_run && rpc_)
  {
    ROS_INFO("[dobot] shutdown: zero velocity + damping");
    rpc_->setVel(0.0f, 0.0f, 0.0f, static_cast<float>(p_.setvel_duration_s));
    if (p_.fsm_damping >= 0) rpc_->setFsmId(p_.fsm_damping);
  }
}

}  // namespace dobot_ros_control
