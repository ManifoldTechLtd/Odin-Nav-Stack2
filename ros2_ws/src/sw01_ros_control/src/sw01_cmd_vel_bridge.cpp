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
 * @file sw01_cmd_vel_bridge.cpp
 * @brief ROS 2 Twist -> Hypertron SW01 (ASTRALL SDK) velocity bridge node.
 *
 * Data flow:
 *   /cmd_vel (geometry_msgs/msg/Twist)
 *       linear.x  -> vx   (forward velocity, m/s)
 *       linear.y  -> vy   (lateral velocity, m/s)
 *       angular.z -> vyaw (yaw rate, rad/s)
 *   -> AstrallMove(vx, vy, vyaw, timeout) via UDP 3600
 *
 * Safety mechanisms:
 *   1. Velocity clamping (max_linear_vel / max_angular_vel)
 *   2. Command timeout protection (cmd_vel_timeout) -> automatic zero velocity
 *   3. Dedicated heartbeat thread (heartbeat_rate)
 *   4. Graceful SIGINT/SIGTERM shutdown: send zero velocity before SDK deinit
 *   5. Control authority verification before sending motion commands
 */
#include "sw01_ros_control/sw01_bridge.h"

#include <csignal>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "ec_trace.h"

namespace sw01_ros_control
{

Sw01Bridge::Sw01Bridge(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_auth_retry_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

Sw01Bridge::~Sw01Bridge()
{
  shutdown();
}

double Sw01Bridge::clampValue(double value, double limit)
{
  const double abs_limit = std::fabs(limit);
  if (!std::isfinite(value))
  {
    return 0.0;  // Treat NaN / Inf as zero velocity
  }
  return std::max(-abs_limit, std::min(abs_limit, value));
}

const char* Sw01Bridge::sportStatusToString(uint16_t status)
{
  switch (status)
  {
    case ASTRALL_SPORT_STATUS_INIT:             return "INIT(初始化中)";
    case ASTRALL_SPORT_STATUS_DAMPING:          return "DAMPING(阻尼)";
    case ASTRALL_SPORT_STATUS_FIXEDSTAND:       return "FIXEDSTAND(锁定站立)";
    case ASTRALL_SPORT_STATUS_FIXEDDOWN:        return "FIXEDDOWN(锁定趴下)";
    case ASTRALL_SPORT_STATUS_MOVE:             return "MOVE(行走)";
    case ASTRALL_SPORT_STATUS_UPSTAIRS:         return "UPSTAIRS(爬楼梯)";
    case ASTRALL_SPORT_STATUS_CAR:              return "CAR(负重小车)";
    case ASTRALL_SPORT_STATUS_AUTOCHAEGE:       return "AUTOCHARGE(准备自动充电)";
    case ASTRALL_SPORT_STATUS_SEARCHCHARGING:   return "SEARCHCHARGING(寻找充电桩)";
    case ASTRALL_SPORT_STATUS_PLUGGINGCHARGING: return "PLUGGINGCHARGING(上桩中)";
    case ASTRALL_SPORT_STATUS_ATCHARGING:       return "ATCHARGING(在充电桩上)";
    case ASTRALL_SPORT_STATUS_EXITCHARGING:     return "EXITCHARGING(退出充电桩)";
    case ASTRALL_SPORT_STATUS_CHARGING:         return "CHARGING(充电中)";
    case ASTRALL_SPORT_STATUS_FIXEDSTANDING:    return "FIXEDSTANDING(站立执行中)";
    case ASTRALL_SPORT_STATUS_FIXEDDOWNING:     return "FIXEDDOWNING(趴下执行中)";
    case ASTRALL_SPORT_STATUS_GET_RIGHT:        return "GET_RIGHT(摔倒恢复)";
    case ASTRALL_SPORT_STATUS_UNKNOWN:          return "UNKNOWN(未知)";
    default:                                    return "UNRECOGNIZED(未识别)";
  }
}

const char* Sw01Bridge::resultToString(uint16_t res)
{
  switch (res)
  {
    case ASTRALL_RES_SUCCESSED:     return "SUCCESSED(成功)";
    case ASTRALL_RES_FAILED:        return "FAILED(执行/申请失败)";
    case ASTRALL_RES_TIMEOUT:       return "TIMEOUT(超时)";
    case ASTRALL_RES_RUNNING:       return "RUNNING(执行中)";
    case ASTRALL_RES_INVALID_PARAM: return "INVALID_PARAM(无效参数)";
    case ASTRALL_RES_NOT_INIT:      return "NOT_INIT(SDK未初始化)";
    case ASTRALL_RES_RC_NORELEASE:  return "RC_NORELEASE(遥控器抢夺未释放控制权)";
    case ASTRALL_RES_BEEN_OBTAINED: return "BEEN_OBTAINED(控制权已被其它设备占用)";
    case ASTRALL_RES_WITHOUT_AUTH:  return "WITHOUT_AUTH(没有控制权)";
    default:                        return "UNKNOWN(未知返回码)";
  }
}

void Sw01Bridge::loadParams()
{
  params_.cmd_vel_topic = node_->declare_parameter("cmd_vel_topic", params_.cmd_vel_topic);
  params_.max_linear_vel = node_->declare_parameter("max_linear_vel", params_.max_linear_vel);
  params_.max_angular_vel = node_->declare_parameter("max_angular_vel", params_.max_angular_vel);
  params_.cmd_vel_timeout = node_->declare_parameter("cmd_vel_timeout", params_.cmd_vel_timeout);
  params_.cmd_vel_decay_window =
      node_->declare_parameter("cmd_vel_decay_window", params_.cmd_vel_decay_window);
  params_.heartbeat_rate = node_->declare_parameter("heartbeat_rate", params_.heartbeat_rate);
  params_.control_rate = node_->declare_parameter("control_rate", params_.control_rate);
  params_.sdk_timeout = node_->declare_parameter("sdk_timeout", params_.sdk_timeout);
  params_.auth_timeout = node_->declare_parameter("auth_timeout", params_.auth_timeout);
  params_.move_timeout = node_->declare_parameter("move_timeout", params_.move_timeout);
  params_.heartbeat_timeout =
      node_->declare_parameter("heartbeat_timeout", params_.heartbeat_timeout);
  params_.request_control_authority = node_->declare_parameter(
      "request_control_authority", params_.request_control_authority);
  params_.require_move_mode = node_->declare_parameter(
      "require_move_mode", params_.require_move_mode);
  params_.auto_enter_move_mode = node_->declare_parameter(
      "auto_enter_move_mode", params_.auto_enter_move_mode);
  params_.publish_status = node_->declare_parameter("publish_status", params_.publish_status);

  // Clamp parameters to safe bounds
  const double kHardLinearLimit = 1.0;   // SDK range [-1.0, 1.0] m/s
  const double kHardAngularLimit = 1.0;  // SDK range [-1.0, 1.0] rad/s

  if (params_.max_linear_vel <= 0.0 || params_.max_linear_vel > kHardLinearLimit)
  {
    RCLCPP_WARN(node_->get_logger(), "max_linear_vel=%.3f 超出允许范围 (0, %.1f]，已强制为 %.1f",
                params_.max_linear_vel, kHardLinearLimit, kHardLinearLimit);
    params_.max_linear_vel = kHardLinearLimit;
  }
  if (params_.max_angular_vel <= 0.0 || params_.max_angular_vel > kHardAngularLimit)
  {
    RCLCPP_WARN(node_->get_logger(), "max_angular_vel=%.3f 超出允许范围 (0, %.1f]，已强制为 %.1f",
                params_.max_angular_vel, kHardAngularLimit, kHardAngularLimit);
    params_.max_angular_vel = kHardAngularLimit;
  }
  if (params_.cmd_vel_timeout <= 0.0)
  {
    RCLCPP_WARN(node_->get_logger(), "cmd_vel_timeout=%.3f 非法，已强制为 0.5s",
                params_.cmd_vel_timeout);
    params_.cmd_vel_timeout = 0.5;
  }
  if (params_.cmd_vel_decay_window < 0.0)
  {
    RCLCPP_WARN(node_->get_logger(), "cmd_vel_decay_window=%.3f 非法，已强制为 0",
                params_.cmd_vel_decay_window);
    params_.cmd_vel_decay_window = 0.0;
  }
  if (params_.heartbeat_rate < 2.0 || params_.heartbeat_rate > 100.0)
  {
    RCLCPP_WARN(node_->get_logger(), "heartbeat_rate=%.1f 超出建议范围 [2, 100]Hz，已强制为 10Hz",
                params_.heartbeat_rate);
    params_.heartbeat_rate = 10.0;
  }
  if (params_.control_rate < 10.0 || params_.control_rate > 250.0)
  {
    // SDK requires a new motion command within 50ms (>= 20Hz) or it stops automatically
    RCLCPP_WARN(node_->get_logger(), "control_rate=%.1f 超出建议范围 [10, 250]Hz，已强制为 50Hz",
                params_.control_rate);
    params_.control_rate = 50.0;
  }
  if (params_.sdk_timeout < 1000)
  {
    RCLCPP_WARN(node_->get_logger(), "sdk_timeout=%d 过小，已强制为 10000ms", params_.sdk_timeout);
    params_.sdk_timeout = 10000;
  }
  if (params_.auth_timeout < 100)
  {
    RCLCPP_WARN(node_->get_logger(), "auth_timeout=%d 过小，已强制为 2000ms", params_.auth_timeout);
    params_.auth_timeout = 2000;
  }
  if (params_.move_timeout < 5)
  {
    RCLCPP_WARN(node_->get_logger(), "move_timeout=%d 过小，已强制为 20ms", params_.move_timeout);
    params_.move_timeout = 20;
  }
  if (params_.heartbeat_timeout < 5)
  {
    RCLCPP_WARN(node_->get_logger(), "heartbeat_timeout=%d 过小，已强制为 20ms",
                params_.heartbeat_timeout);
    params_.heartbeat_timeout = 20;
  }

  RCLCPP_INFO(node_->get_logger(), "---------------- sw01_cmd_vel_bridge 参数 ----------------");
  RCLCPP_INFO(node_->get_logger(), "  cmd_vel_topic            : %s", params_.cmd_vel_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "  max_linear_vel           : %.2f m/s", params_.max_linear_vel);
  RCLCPP_INFO(node_->get_logger(), "  max_angular_vel          : %.2f rad/s", params_.max_angular_vel);
  RCLCPP_INFO(node_->get_logger(), "  cmd_vel_timeout          : %.2f s", params_.cmd_vel_timeout);
  RCLCPP_INFO(node_->get_logger(), "  cmd_vel_decay_window     : %.2f s", params_.cmd_vel_decay_window);
  RCLCPP_INFO(node_->get_logger(), "  heartbeat_rate           : %.1f Hz", params_.heartbeat_rate);
  RCLCPP_INFO(node_->get_logger(), "  control_rate             : %.1f Hz", params_.control_rate);
  RCLCPP_INFO(node_->get_logger(), "  sdk_timeout              : %d ms", params_.sdk_timeout);
  RCLCPP_INFO(node_->get_logger(), "  auth_timeout             : %d ms", params_.auth_timeout);
  RCLCPP_INFO(node_->get_logger(), "  request_control_authority: %s",
              params_.request_control_authority ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "  require_move_mode        : %s",
              params_.require_move_mode ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "  auto_enter_move_mode     : %s",
              params_.auto_enter_move_mode ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "---------------------------------------------------------");
}

bool Sw01Bridge::init()
{
  loadParams();

  if (params_.publish_status)
  {
    link_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "~/sdk_linked", rclcpp::QoS(1).transient_local());
    authority_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "~/control_authority", rclcpp::QoS(1).transient_local());
  }
  cmd_sent_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_sent", 100);
  control_trace_pub_ = node_->create_publisher<std_msgs::msg::String>("~/control_trace", 100);

  // Initialize SDK
  AstrallConfig cfg;
  cfg.heartbeatCb = [this](void* /*data*/, uint16_t /*len*/) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "收到 SW01 心跳应答 / heartbeat ack from SW01");
  };
  cfg.sdkStatusCb = [this](void* data, uint16_t len) {
    if (data == nullptr || len < sizeof(AstrallSdkStatus))
    {
      return;
    }
    const AstrallSdkStatus* status = static_cast<const AstrallSdkStatus*>(data);
    const bool linked = (status->link != 0);
    const bool authority = (status->ctrlAuthority != 0);
    if (linked != sdk_linked_.load() || authority != has_authority_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "SDK 状态变化 / status changed: link=%d ctrlAuthority=%d",
                  static_cast<int>(linked), static_cast<int>(authority));
    }
    sdk_linked_.store(linked);
    has_authority_.store(authority);
  };

  RCLCPP_INFO(node_->get_logger(), "正在初始化 ASTRALL SDK（超时 %d ms）...", params_.sdk_timeout);
  RCLCPP_INFO(node_->get_logger(),
              "请确认：1) NX 与 SW01 SDK 网口连通 (SW01: 10.18.0.100, UDP 3600)；"
              "2) SW01 处于阻尼模式或锁定趴下状态，否则 SDK 无法建立连接");

  uint16_t res = AstrallSdkInit(cfg, static_cast<uint32_t>(params_.sdk_timeout));
  if (res != ASTRALL_RES_SUCCESSED)
  {
    RCLCPP_ERROR(node_->get_logger(), "ASTRALL SDK 初始化失败 / SDK init failed: 0x%04X %s",
                 res, resultToString(res));
    RCLCPP_ERROR(node_->get_logger(),
                 "排查建议：ping 10.18.0.100 是否通；eth1 是否配置为 10.18.0.200/24；"
                 "SW01 是否处于阻尼/锁定趴下状态；SDK 动态库版本是否匹配");
    EC_TRACE(ec::SDK_CONNECT_FAIL, "sdk_init", "res=0x%04X", res);
    return false;
  }
  sdk_initialized_.store(true);
  ::setenv("EC_NODE", "sw01_cmd_vel_bridge", 1);
  EC_TRACE(ec::NODE_READY, "startup", "ASTRALL SDK initialized ok");
  RCLCPP_INFO(node_->get_logger(), "ASTRALL SDK 初始化成功 / SDK initialized");

  // Log device, battery, and system status
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    AstrallDeviceInfo info;
    uint16_t info_res = AstrallGetDeviceInfo(info, 200);
    if (info_res == ASTRALL_RES_SUCCESSED)
    {
      RCLCPP_INFO(node_->get_logger(), "设备信息 / device: version=%s sn=%s model=%u",
                  info.version.c_str(), info.serialNumber.c_str(), info.model);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "读取设备信息失败: 0x%04X %s", info_res,
                  resultToString(info_res));
    }

    AstrallPowerStatus power;
    if (AstrallGetPowerStatus(power) == ASTRALL_RES_SUCCESSED)
    {
      RCLCPP_INFO(node_->get_logger(), "电池状态 / battery: soc=%.1f%% voltage=%.1fV temp=%.1f℃",
                  power.soc, power.voltage, power.temp);
      if (power.soc > 0.0f && power.soc < 20.0f)
      {
        RCLCPP_WARN(node_->get_logger(), "电池电量偏低 (%.1f%%)，建议充电后再进行运动测试",
                    power.soc);
      }
    }

    AstrallSystemStatus sys;
    if (AstrallGetSystemStatus(sys) == ASTRALL_RES_SUCCESSED)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "系统状态 / system: sysStatus=%u errorCode=0x%08X warnCode=0x%08X",
                  static_cast<unsigned>(sys.sysStatus),
                  static_cast<unsigned>(sys.errorCode),
                  static_cast<unsigned>(sys.warnCode));
      if (sys.errorCode != ASTRALL_ERR_NONE)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "机器人存在错误码 0x%08X，请先排除故障再下发运动指令",
                     static_cast<unsigned>(sys.errorCode));
      }
    }
  }

  // Request and verify control authority
  if (params_.request_control_authority)
  {
    if (!requestControlAuthority())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "启动时未取得 SDK 控制权，节点将继续运行并周期性重试；"
                  "在取得控制权之前不会下发任何运动指令");
    }
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),
                "request_control_authority=false：不主动申请控制权，"
                "请确保已由其它途径将控制权交给 SDK");
  }

  // Check current sport mode
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    uint16_t sport = AstrallGetSportStatus();
    RCLCPP_INFO(node_->get_logger(), "当前运动状态 / sport status: 0x%04X %s", sport,
                sportStatusToString(sport));
    if (sport != ASTRALL_SPORT_STATUS_MOVE)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "机器人尚未进入 MOVE(行走) 状态，速度指令暂不会生效。"
                  "请通过遥控器或 SDK 依次执行「锁定站立」->「行走」后再下发速度指令");
    }
  }

  // Start heartbeat thread and velocity subscription
  running_.store(true);
  heartbeat_thread_ = std::thread(&Sw01Bridge::heartbeatLoop, this);

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(1),
      std::bind(&Sw01Bridge::cmdVelCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "已订阅速度话题 / subscribed: %s", params_.cmd_vel_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "桥接节点就绪 / bridge ready");
  publishStatus();
  return true;
}

bool Sw01Bridge::requestControlAuthority()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  uint16_t res = AstrallAuthControl(ASTRALL_AUTH_SDK, static_cast<uint32_t>(params_.auth_timeout));
  if (res == ASTRALL_RES_SUCCESSED)
  {
    has_authority_.store(true);
    RCLCPP_INFO(node_->get_logger(), "已取得 SDK 控制权 / control authority granted");
    return true;
  }

  has_authority_.store(false);
  if (res == ASTRALL_RES_RC_NORELEASE)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "控制权申请失败：遥控器已抢夺控制权且未释放 (0x%04X)。"
                 "请将遥控器开关拨至释放位置后重试", res);
  }
  else if (res == ASTRALL_RES_BEEN_OBTAINED)
  {
    RCLCPP_ERROR(node_->get_logger(), "控制权申请失败：控制权已被其它设备占用 (0x%04X)", res);
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "控制权申请失败: 0x%04X %s", res, resultToString(res));
  }
  return false;
}

void Sw01Bridge::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  const double vx = clampValue(msg->linear.x, params_.max_linear_vel);
  const double vy = clampValue(msg->linear.y, params_.max_linear_vel);
  const double vyaw = clampValue(msg->angular.z, params_.max_angular_vel);

  if (std::fabs(msg->linear.x) > params_.max_linear_vel ||
      std::fabs(msg->linear.y) > params_.max_linear_vel ||
      std::fabs(msg->angular.z) > params_.max_angular_vel)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "速度指令超限已被限幅 / clamped: 输入(%.2f, %.2f, %.2f) -> "
                         "输出(%.2f, %.2f, %.2f)",
                         msg->linear.x, msg->linear.y, msg->angular.z, vx, vy, vyaw);
  }

  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cmd_vx_ = vx;
  cmd_vy_ = vy;
  cmd_vyaw_ = vyaw;
  last_cmd_time_ = node_->now();
  cmd_received_ = true;
}

void Sw01Bridge::heartbeatLoop()
{
  // SDK requires a heartbeat within 500ms or it clears commands and stands; another 500ms without heartbeat disconnects
  const double period = 1.0 / params_.heartbeat_rate;
  const auto period_us = std::chrono::microseconds(static_cast<long long>(period * 1e6));
  int consecutive_failures = 0;

  while (running_.load())
  {
    uint16_t res;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      res = AstrallHeartbeat(static_cast<uint32_t>(params_.heartbeat_timeout));
    }

    if (res == ASTRALL_RES_SUCCESSED)
    {
      if (consecutive_failures > 0)
      {
        RCLCPP_INFO(node_->get_logger(), "心跳恢复正常 / heartbeat recovered (之前连续失败 %d 次)",
                    consecutive_failures);
      }
      consecutive_failures = 0;
    }
    else
    {
      ++consecutive_failures;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "心跳失败 / heartbeat failed: 0x%04X %s (连续 %d 次)",
                           res, resultToString(res), consecutive_failures);
      if (consecutive_failures == 5)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "心跳连续失败 5 次，机器人可能即将断开连接并趴下，请检查网络与 SW01 状态");
      }
    }

    std::this_thread::sleep_for(period_us);
  }
  RCLCPP_INFO(node_->get_logger(), "心跳线程已退出 / heartbeat thread stopped");
}

bool Sw01Bridge::motionAllowed()
{
  if (!has_authority_.load())
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "当前没有 SDK 控制权，运动指令被丢弃 / no control authority");

    if (params_.request_control_authority)
    {
      const rclcpp::Time now = node_->now();
      if (last_auth_retry_.nanoseconds() == 0 ||
          (now - last_auth_retry_).seconds() >= 2.0)
      {
        last_auth_retry_ = now;
        requestControlAuthority();
      }
    }
    return has_authority_.load();
  }

  if (!params_.require_move_mode)
  {
    return true;
  }

  uint16_t sport;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    sport = AstrallGetSportStatus();
  }
  if (sport == ASTRALL_SPORT_STATUS_MOVE)
  {
    return true;
  }

  if (params_.auto_enter_move_mode &&
      (sport == ASTRALL_SPORT_STATUS_FIXEDSTAND || sport == ASTRALL_SPORT_STATUS_DAMPING))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "auto_enter_move_mode=true，正在从 %s 切换到 MOVE(行走) 状态",
                         sportStatusToString(sport));
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (sport == ASTRALL_SPORT_STATUS_DAMPING)
    {
      // Transition from DAMPING requires FIXEDSTAND before entering MOVE
      uint16_t stand_res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND,
                                                  static_cast<uint32_t>(params_.move_timeout));
      RCLCPP_WARN(node_->get_logger(), "下发「锁定站立」指令: 0x%04X %s", stand_res,
                  resultToString(stand_res));
      return false;
    }
    uint16_t move_res = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE,
                                               static_cast<uint32_t>(params_.move_timeout));
    RCLCPP_WARN(node_->get_logger(), "下发「行走」指令: 0x%04X %s", move_res,
                resultToString(move_res));
    return false;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "机器人当前状态为 0x%04X %s，非 MOVE(行走) 状态，速度指令被忽略。"
                       "请先让机器人站立并进入行走模式",
                       sport, sportStatusToString(sport));
  return false;
}

uint16_t Sw01Bridge::sendMove(float vx, float vy, float vyaw)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  uint16_t res = AstrallMove(vx, vy, vyaw, static_cast<uint32_t>(params_.move_timeout));
  if (res != ASTRALL_RES_SUCCESSED)
  {
    if (res == ASTRALL_RES_WITHOUT_AUTH)
    {
      has_authority_.store(false);
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "运动指令被拒绝：没有控制权 (0x%04X)，可能已被遥控器抢夺", res);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "运动指令下发失败 / move failed: 0x%04X %s (vx=%.2f vy=%.2f vyaw=%.2f)",
                           res, resultToString(res), vx, vy, vyaw);
    }
  }
  else
  {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "move: vx=%.2f vy=%.2f vyaw=%.2f", vx, vy, vyaw);
  }
  return res;
}

void Sw01Bridge::sendZeroVelocity(int repeat)
{
  if (!sdk_initialized_.load())
  {
    return;
  }
  for (int i = 0; i < repeat; ++i)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    AstrallMove(0.0f, 0.0f, 0.0f, static_cast<uint32_t>(params_.move_timeout));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void Sw01Bridge::publishStatus()
{
  if (!params_.publish_status)
  {
    return;
  }
  std_msgs::msg::Bool linked_msg;
  linked_msg.data = sdk_linked_.load();
  link_pub_->publish(linked_msg);

  std_msgs::msg::Bool auth_msg;
  auth_msg.data = has_authority_.load();
  authority_pub_->publish(auth_msg);
}

void Sw01Bridge::publishControlTrace(float vx, float vy, float vyaw, double cmd_age,
                                     bool timed_out, bool motion_allowed,
                                     bool move_attempted, uint16_t move_result)
{
  if (move_attempted)
  {
    geometry_msgs::msg::TwistStamped sent;
    sent.header.stamp = node_->now();
    sent.header.frame_id = "sw01_cmd_vel_bridge";
    sent.twist.linear.x = vx;
    sent.twist.linear.y = vy;
    sent.twist.angular.z = vyaw;
    cmd_sent_pub_->publish(sent);
  }

  std::ostringstream json;
  json << std::boolalpha << std::fixed << std::setprecision(6)
       << "{\"stamp\":" << node_->now().seconds()
       << ",\"cmd_age_sec\":" << cmd_age
       << ",\"selected_vx\":" << vx
       << ",\"selected_vy\":" << vy
       << ",\"selected_wz\":" << vyaw
       << ",\"timed_out\":" << timed_out
       << ",\"motion_allowed\":" << motion_allowed
       << ",\"move_attempted\":" << move_attempted
       << ",\"move_result\":" << move_result
       << ",\"sdk_linked\":" << sdk_linked_.load()
       << ",\"control_authority\":" << has_authority_.load()
       << "}";
  std_msgs::msg::String trace;
  trace.data = json.str();
  control_trace_pub_->publish(trace);
}

void Sw01Bridge::spin()
{
  rclcpp::WallRate rate(params_.control_rate);
  const int status_publish_interval = static_cast<int>(params_.control_rate);
  int tick = 0;

  while (rclcpp::ok() && !stop_requested_.load())
  {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    bool timed_out = false;
    bool ever_received = false;
    double cmd_age = -1.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      const rclcpp::Time now = node_->now();
      ever_received = cmd_received_;
      if (!cmd_received_)
      {
        timed_out = true;
      }
      else
      {
        const double dt = (now - last_cmd_time_).seconds();
        cmd_age = dt;
        if (dt > params_.cmd_vel_timeout)
        {
          timed_out = true;
        }
        else if (params_.cmd_vel_decay_window > 0.0 &&
                 dt > (params_.cmd_vel_timeout - params_.cmd_vel_decay_window))
        {
          const double frac = (params_.cmd_vel_timeout - dt) /
                              params_.cmd_vel_decay_window;
          vx = cmd_vx_ * frac;
          vyaw = cmd_vyaw_ * frac;
        }
        else
        {
          vx = cmd_vx_;
          vy = cmd_vy_;
          vyaw = cmd_vyaw_;
        }
      }
    }

    if (timed_out && !cmd_timed_out_.exchange(true))
    {
      if (ever_received)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "超过 %.2fs 未收到 %s 消息，已切换为零速输出 / cmd_vel timeout -> zero velocity",
                    params_.cmd_vel_timeout, params_.cmd_vel_topic.c_str());
      }
    }
    else if (!timed_out && cmd_timed_out_.exchange(false))
    {
      RCLCPP_INFO(node_->get_logger(), "重新收到速度指令 / cmd_vel resumed");
    }

    const bool motion_allowed = motionAllowed();
    bool move_attempted = false;
    uint16_t move_result = 0;
    if (motion_allowed)
    {
      move_attempted = true;
      move_result = sendMove(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
    }
    publishControlTrace(static_cast<float>(vx), static_cast<float>(vy),
                        static_cast<float>(vyaw), cmd_age, timed_out,
                        motion_allowed, move_attempted, move_result);

    if (++tick >= status_publish_interval)
    {
      tick = 0;
      publishStatus();
    }

    rate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "主控制循环退出 / control loop exited");
}

void Sw01Bridge::requestStop()
{
  stop_requested_.store(true);
}

void Sw01Bridge::shutdown()
{
  if (shutdown_done_.exchange(true))
  {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "开始优雅退出 / graceful shutdown ...");

  if (sdk_initialized_.load() && has_authority_.load())
  {
    RCLCPP_INFO(node_->get_logger(), "下发零速指令 / sending zero velocity");
    sendZeroVelocity(5);
  }

  running_.store(false);
  if (heartbeat_thread_.joinable())
  {
    heartbeat_thread_.join();
  }

  if (sdk_initialized_.exchange(false))
  {
    AstrallSdkDeinit();
    RCLCPP_INFO(node_->get_logger(), "ASTRALL SDK 已断开 / SDK deinitialized");
  }

  RCLCPP_WARN(node_->get_logger(),
              "节点已退出。机器人仍处于当前运动模式，若需要收车请通过遥控器切换到"
              "「锁定趴下」或「阻尼」状态");
}

}  // namespace sw01_ros_control

namespace
{
sw01_ros_control::Sw01Bridge* g_bridge = nullptr;
volatile sig_atomic_t g_sigint_received = 0;

/** @brief Signal handler setting the stop flag so shutdown runs on the main thread. */
void sigintHandler(int /*sig*/)
{
  g_sigint_received = 1;
  if (g_bridge != nullptr)
  {
    g_bridge->requestStop();
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Install custom SIGINT/SIGTERM handler so zero velocity is sent before SDK deinitialization.
  rclcpp::uninstall_signal_handlers();

  auto node = std::make_shared<rclcpp::Node>("sw01_cmd_vel_bridge");
  sw01_ros_control::Sw01Bridge bridge(node);
  g_bridge = &bridge;
  signal(SIGINT, sigintHandler);
  signal(SIGTERM, sigintHandler);

  if (!bridge.init())
  {
    RCLCPP_ERROR(node->get_logger(), "sw01_cmd_vel_bridge 初始化失败，节点退出");
    g_bridge = nullptr;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bridge.spin();
  bridge.shutdown();

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }

  g_bridge = nullptr;
  rclcpp::shutdown();
  return 0;
}
