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
 * @file sw01_bridge.cpp
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
 *   6. Explicit motion enable (~/enable); start-up, authority loss, link loss and faults disable motion
 */
#include "sw01_ros_control/sw01_bridge.h"

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "ec_trace.h"

namespace sw01_ros_control
{

namespace
{
constexpr auto kAuthRetryInterval = std::chrono::seconds(2);
constexpr auto kStatusPollInterval = std::chrono::milliseconds(500);
// SDK requires a new motion command within 50ms or it stops automatically
constexpr double kSendIntervalWarnMs = 50.0;
constexpr double kMotionEpsilon = 1e-3;

double secondsBetween(std::chrono::steady_clock::time_point later,
                      std::chrono::steady_clock::time_point earlier)
{
  return std::chrono::duration<double>(later - earlier).count();
}
}  // namespace

Sw01Bridge::Sw01Bridge(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ChassisSdk> sdk,
                       SteadyClockFn now_fn)
  : node_(node), sdk_(std::move(sdk)), now_fn_(std::move(now_fn))
{
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
  params_.require_enable = node_->declare_parameter("require_enable", params_.require_enable);
  params_.blocking_error_mask =
      node_->declare_parameter("blocking_error_mask", params_.blocking_error_mask);

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
  if (params_.cmd_vel_decay_window > params_.cmd_vel_timeout)
  {
    RCLCPP_WARN(node_->get_logger(),
                "cmd_vel_decay_window=%.3f 大于 cmd_vel_timeout=%.3f，已截断为 %.3f",
                params_.cmd_vel_decay_window, params_.cmd_vel_timeout, params_.cmd_vel_timeout);
    params_.cmd_vel_decay_window = params_.cmd_vel_timeout;
  }
  if (params_.heartbeat_rate < 2.0 || params_.heartbeat_rate > 100.0)
  {
    RCLCPP_WARN(node_->get_logger(), "heartbeat_rate=%.1f 超出建议范围 [2, 100]Hz，已强制为 10Hz",
                params_.heartbeat_rate);
    params_.heartbeat_rate = 10.0;
  }
  if (params_.control_rate < 25.0 || params_.control_rate > 250.0)
  {
    // SDK requires a new motion command within 50ms (>= 20Hz) or it stops automatically
    RCLCPP_WARN(node_->get_logger(), "control_rate=%.1f 超出建议范围 [25, 250]Hz，已强制为 50Hz",
                params_.control_rate);
    params_.control_rate = 50.0;
  }
  if (params_.sdk_timeout < 1000)
  {
    RCLCPP_WARN(node_->get_logger(), "sdk_timeout=%d 过小，已强制为 10000ms", params_.sdk_timeout);
    params_.sdk_timeout = 10000;
  }
  if (params_.auth_timeout < 50)
  {
    RCLCPP_WARN(node_->get_logger(), "auth_timeout=%d 过小，已强制为 200ms", params_.auth_timeout);
    params_.auth_timeout = 200;
  }
  else if (params_.auth_timeout > 400)
  {
    // The SDK holds its send lock while waiting, so heartbeats stall for the whole request
    RCLCPP_WARN(node_->get_logger(),
                "auth_timeout=%d 过大（申请期间心跳被阻塞，SDK 心跳容限 500ms），已限制为 400ms",
                params_.auth_timeout);
    params_.auth_timeout = 400;
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
  if (params_.blocking_error_mask < 0 || params_.blocking_error_mask > 0xFFFFFFFFLL)
  {
    RCLCPP_WARN(node_->get_logger(), "blocking_error_mask=%lld 非法，已强制为 0",
                static_cast<long long>(params_.blocking_error_mask));
    params_.blocking_error_mask = 0;
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
  RCLCPP_INFO(node_->get_logger(), "  require_enable           : %s",
              params_.require_enable ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "  blocking_error_mask      : 0x%08llX",
              static_cast<unsigned long long>(params_.blocking_error_mask));
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
  enabled_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "~/motion_enabled", rclcpp::QoS(1).transient_local());
  cmd_sent_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_sent", 100);
  control_trace_pub_ = node_->create_publisher<std_msgs::msg::String>("~/control_trace", 100);

  // Initialize SDK
  sdk_cfg_.heartbeatCb = [this](void* /*data*/, uint16_t /*len*/) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "收到 SW01 心跳应答 / heartbeat ack from SW01");
  };
  sdk_cfg_.sdkStatusCb = [this](void* data, uint16_t len) {
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

  uint16_t res = sdk_->init(sdk_cfg_, static_cast<uint32_t>(params_.sdk_timeout));
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
  sdk_linked_.store(true);
  ::setenv("EC_NODE", "sw01_cmd_vel_bridge", 1);
  EC_TRACE(ec::NODE_READY, "startup", "ASTRALL SDK initialized ok");
  RCLCPP_INFO(node_->get_logger(), "ASTRALL SDK 初始化成功 / SDK initialized");

  // Keep the link alive before any query or authority request that may block
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    sdk_->heartbeat(static_cast<uint32_t>(params_.heartbeat_timeout));
  }
  running_.store(true);
  heartbeat_thread_ = std::thread(&Sw01Bridge::heartbeatLoop, this);

  // Log device, battery, and system status
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    AstrallDeviceInfo info;
    uint16_t info_res = sdk_->deviceInfo(info, 200);
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
    if (sdk_->powerStatus(power) == ASTRALL_RES_SUCCESSED)
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
    if (sdk_->systemStatus(sys) == ASTRALL_RES_SUCCESSED)
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
  pollSystemStatus(now_fn_(), true);

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
    uint16_t sport = sdk_->sportStatus();
    RCLCPP_INFO(node_->get_logger(), "当前运动状态 / sport status: 0x%04X %s", sport,
                sportStatusToString(sport));
    if (sport != ASTRALL_SPORT_STATUS_MOVE)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "机器人尚未进入 MOVE(行走) 状态，速度指令暂不会生效。"
                  "请通过遥控器或 SDK 依次执行「锁定站立」->「行走」后再下发速度指令");
    }
  }

  motion_enabled_.store(!params_.require_enable);

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      params_.cmd_vel_topic, rclcpp::QoS(1),
      std::bind(&Sw01Bridge::cmdVelCallback, this, std::placeholders::_1));
  enable_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "~/enable", std::bind(&Sw01Bridge::enableCallback, this, std::placeholders::_1,
                            std::placeholders::_2));
  RCLCPP_INFO(node_->get_logger(), "已订阅速度话题 / subscribed: %s", params_.cmd_vel_topic.c_str());
  if (params_.require_enable)
  {
    RCLCPP_WARN(node_->get_logger(),
                "底盘运动未使能 / motion disabled：确认现场安全后执行 "
                "ros2 service call %s/enable std_srvs/srv/SetBool \"{data: true}\"",
                node_->get_fully_qualified_name());
  }
  RCLCPP_INFO(node_->get_logger(), "桥接节点就绪 / bridge ready");
  publishStatus();
  return true;
}

bool Sw01Bridge::requestControlAuthority()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  uint16_t res = sdk_->requestAuthority(static_cast<uint32_t>(params_.auth_timeout));
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
  last_cmd_time_ = now_fn_();
  cmd_received_ = true;
}

void Sw01Bridge::enableCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> gate(gate_mutex_);
  if (!request->data)
  {
    if (motion_enabled_.exchange(false))
    {
      RCLCPP_WARN(node_->get_logger(), "底盘运动已由操作员解除使能 / motion disabled by operator");
    }
    response->success = true;
    response->message = "motion disabled";
    publishStatus();
    return;
  }

  std::string reason;
  if (!sdk_initialized_.load())
  {
    reason = "SDK not initialized";
  }
  else if (!sdk_linked_.load())
  {
    reason = "SDK link is down";
  }
  else if (!has_authority_.load())
  {
    reason = "no SDK control authority (release the remote controller first)";
  }
  else if (fault_active_.load())
  {
    reason = "robot fault active (check sysStatus / errorCode)";
  }
  if (!reason.empty())
  {
    response->success = false;
    response->message = "enable rejected: " + reason;
    RCLCPP_WARN(node_->get_logger(), "使能被拒绝 / enable rejected: %s", reason.c_str());
    return;
  }

  {
    // Only commands received after enabling may move the robot
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_received_ = false;
    cmd_vx_ = 0.0;
    cmd_vy_ = 0.0;
    cmd_vyaw_ = 0.0;
  }
  motion_enabled_.store(true);
  response->success = true;
  response->message = "motion enabled; only commands received from now on are executed";
  RCLCPP_WARN(node_->get_logger(), "底盘运动已使能，只执行此后收到的速度指令 / motion enabled");
  publishStatus();
}

void Sw01Bridge::disableMotion(const std::string& reason)
{
  if (!params_.require_enable)
  {
    return;
  }
  std::lock_guard<std::mutex> gate(gate_mutex_);
  if (!motion_enabled_.exchange(false))
  {
    return;
  }
  RCLCPP_WARN(node_->get_logger(),
              "底盘运动已解除使能 / motion disabled: %s。确认安全后重新使能: "
              "ros2 service call %s/enable std_srvs/srv/SetBool \"{data: true}\"",
              reason.c_str(), node_->get_fully_qualified_name());
  publishStatus();
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
      res = sdk_->heartbeat(static_cast<uint32_t>(params_.heartbeat_timeout));
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

bool Sw01Bridge::ensureAuthority(std::chrono::steady_clock::time_point now)
{
  if (has_authority_.load())
  {
    return true;
  }
  disableMotion("失去 SDK 控制权 / control authority lost");
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "当前没有 SDK 控制权，运动指令被丢弃 / no control authority");

  if (params_.request_control_authority &&
      (!auth_retry_started_ || now - last_auth_retry_ >= kAuthRetryInterval))
  {
    auth_retry_started_ = true;
    last_auth_retry_ = now;
    requestControlAuthority();
  }
  return has_authority_.load();
}

void Sw01Bridge::pollSystemStatus(std::chrono::steady_clock::time_point now, bool force)
{
  if (!force && status_polled_ && now - last_status_poll_ < kStatusPollInterval)
  {
    return;
  }
  status_polled_ = true;
  last_status_poll_ = now;

  AstrallSystemStatus sys;
  uint16_t res;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    res = sdk_->systemStatus(sys);
  }
  if (res != ASTRALL_RES_SUCCESSED)
  {
    return;
  }
  const uint32_t error_bits = static_cast<uint32_t>(sys.errorCode);
  const uint32_t mask = static_cast<uint32_t>(params_.blocking_error_mask);
  const bool fault = (sys.sysStatus == ASTRALL_SYSTEM_CODE_ERROR) || (error_bits & mask) != 0;
  if (fault != fault_active_.exchange(fault))
  {
    if (fault)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "机器人故障，禁止运动 / robot fault: sysStatus=%u errorCode=0x%08X "
                   "(blocking_error_mask=0x%08X)",
                   static_cast<unsigned>(sys.sysStatus), error_bits, mask);
    }
    else
    {
      if (params_.require_enable)
      {
        RCLCPP_INFO(node_->get_logger(), "机器人故障已清除，需重新使能 / robot fault cleared");
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "机器人故障已清除，已恢复允许运动 / robot fault cleared");
      }
    }
  }
}

Sw01Bridge::CommandSample Sw01Bridge::sampleCommand(std::chrono::steady_clock::time_point now)
{
  CommandSample sample;
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  sample.ever_received = cmd_received_;
  if (!cmd_received_)
  {
    return sample;
  }
  const double dt = secondsBetween(now, last_cmd_time_);
  sample.age_sec = dt;
  if (dt > params_.cmd_vel_timeout)
  {
    return sample;
  }
  sample.timed_out = false;
  double frac = 1.0;
  if (params_.cmd_vel_decay_window > 0.0 &&
      dt > (params_.cmd_vel_timeout - params_.cmd_vel_decay_window))
  {
    frac = (params_.cmd_vel_timeout - dt) / params_.cmd_vel_decay_window;
  }
  sample.vx = cmd_vx_ * frac;
  sample.vy = cmd_vy_ * frac;
  sample.vyaw = cmd_vyaw_ * frac;
  return sample;
}

void Sw01Bridge::handleNonMoveState(uint16_t sport, bool motion_ok, const CommandSample& cmd)
{
  const bool wants_motion =
      !cmd.timed_out &&
      (std::fabs(cmd.vx) + std::fabs(cmd.vy) + std::fabs(cmd.vyaw)) > kMotionEpsilon;
  if (params_.auto_enter_move_mode && motion_ok && wants_motion &&
      (sport == ASTRALL_SPORT_STATUS_FIXEDSTAND || sport == ASTRALL_SPORT_STATUS_DAMPING))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "auto_enter_move_mode=true，正在从 %s 切换到 MOVE(行走) 状态",
                         sportStatusToString(sport));
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (sport == ASTRALL_SPORT_STATUS_DAMPING)
    {
      // Transition from DAMPING requires FIXEDSTAND before entering MOVE
      uint16_t stand_res = sdk_->sportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND,
                                                  static_cast<uint32_t>(params_.move_timeout));
      RCLCPP_WARN(node_->get_logger(), "下发「锁定站立」指令: 0x%04X %s", stand_res,
                  resultToString(stand_res));
      return;
    }
    uint16_t move_res = sdk_->sportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE,
                                               static_cast<uint32_t>(params_.move_timeout));
    RCLCPP_WARN(node_->get_logger(), "下发「行走」指令: 0x%04X %s", move_res,
                resultToString(move_res));
    return;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "机器人当前状态为 0x%04X %s，非 MOVE(行走) 状态，速度指令被忽略。"
                       "请先让机器人站立并进入行走模式",
                       sport, sportStatusToString(sport));
}

uint16_t Sw01Bridge::sendMove(float vx, float vy, float vyaw)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  uint16_t res = sdk_->move(vx, vy, vyaw, static_cast<uint32_t>(params_.move_timeout));
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
  int failures = 0;
  for (int i = 0; i < repeat; ++i)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (sdk_->move(0.0f, 0.0f, 0.0f, static_cast<uint32_t>(params_.move_timeout)) !=
        ASTRALL_RES_SUCCESSED)
    {
      ++failures;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (failures > 0)
  {
    RCLCPP_WARN(node_->get_logger(), "零速指令下发失败 %d/%d 次 / zero velocity not acknowledged",
                failures, repeat);
  }
}

void Sw01Bridge::publishStatus()
{
  if (enabled_pub_)
  {
    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = motion_enabled_.load();
    enabled_pub_->publish(enabled_msg);
  }
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

void Sw01Bridge::publishControlTrace(float vx, float vy, float vyaw, const CommandSample& cmd,
                                     bool motion_ok, bool move_attempted, uint16_t move_result,
                                     double send_interval_ms)
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
       << ",\"cmd_age_sec\":" << cmd.age_sec
       << ",\"selected_vx\":" << cmd.vx
       << ",\"selected_vy\":" << cmd.vy
       << ",\"selected_wz\":" << cmd.vyaw
       << ",\"sent_vx\":" << vx
       << ",\"sent_vy\":" << vy
       << ",\"sent_wz\":" << vyaw
       << ",\"timed_out\":" << cmd.timed_out
       << ",\"motion_allowed\":" << motion_ok
       << ",\"move_attempted\":" << move_attempted
       << ",\"move_result\":" << move_result
       << ",\"send_interval_ms\":" << send_interval_ms
       << ",\"motion_enabled\":" << motion_enabled_.load()
       << ",\"fault_active\":" << fault_active_.load()
       << ",\"sdk_linked\":" << sdk_linked_.load()
       << ",\"control_authority\":" << has_authority_.load()
       << "}";
  std_msgs::msg::String trace;
  trace.data = json.str();
  control_trace_pub_->publish(trace);
}

void Sw01Bridge::step()
{
  pollSystemStatus(now_fn_(), false);
  if (fault_active_.load())
  {
    disableMotion("机器人故障 / robot fault");
  }
  if (!sdk_linked_.load())
  {
    disableMotion("SDK 链路断开 / SDK link lost");
  }
  const bool authority = ensureAuthority(now_fn_());

  // Select the command only after the (possibly blocking) authority request
  const auto now = now_fn_();
  const CommandSample cmd = sampleCommand(now);
  if (cmd.timed_out && !cmd_timed_out_.exchange(true))
  {
    if (cmd.ever_received)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "超过 %.2fs 未收到 %s 消息，已切换为零速输出 / cmd_vel timeout -> zero velocity",
                  params_.cmd_vel_timeout, params_.cmd_vel_topic.c_str());
    }
  }
  else if (!cmd.timed_out && cmd_timed_out_.exchange(false))
  {
    RCLCPP_INFO(node_->get_logger(), "重新收到速度指令 / cmd_vel resumed");
  }

  const bool enabled = motion_enabled_.load();
  const bool motion_ok = enabled && !fault_active_.load() && sdk_linked_.load();
  const bool wants_motion =
      !cmd.timed_out &&
      (std::fabs(cmd.vx) + std::fabs(cmd.vy) + std::fabs(cmd.vyaw)) > kMotionEpsilon;
  if (wants_motion && !motion_ok)
  {
    if (!enabled)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "收到速度指令但底盘运动未使能，指令被忽略。确认安全后执行: "
                           "ros2 service call %s/enable std_srvs/srv/SetBool \"{data: true}\"",
                           node_->get_fully_qualified_name());
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "收到速度指令但存在故障或 SDK 链路断开，指令被忽略 / "
                           "command ignored: fault or link lost");
    }
  }

  bool move_attempted = false;
  uint16_t move_result = 0;
  double send_interval_ms = -1.0;
  float out_vx = 0.0f;
  float out_vy = 0.0f;
  float out_vyaw = 0.0f;
  if (authority)
  {
    uint16_t sport = ASTRALL_SPORT_STATUS_MOVE;
    if (params_.require_move_mode)
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      sport = sdk_->sportStatus();
    }
    if (params_.require_move_mode && sport != ASTRALL_SPORT_STATUS_MOVE)
    {
      handleNonMoveState(sport, motion_ok, cmd);
    }
    else
    {
      if (motion_ok)
      {
        out_vx = static_cast<float>(cmd.vx);
        out_vy = static_cast<float>(cmd.vy);
        out_vyaw = static_cast<float>(cmd.vyaw);
      }
      if (have_last_move_)
      {
        send_interval_ms = 1000.0 * secondsBetween(now, last_move_time_);
        if (send_interval_ms > kSendIntervalWarnMs)
        {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                               "运动指令实际发送间隔 %.1f ms 超过 %.0f ms / move interval too long",
                               send_interval_ms, kSendIntervalWarnMs);
        }
      }
      have_last_move_ = true;
      last_move_time_ = now;
      move_attempted = true;
      move_result = sendMove(out_vx, out_vy, out_vyaw);
    }
  }
  if (!move_attempted)
  {
    have_last_move_ = false;
  }
  publishControlTrace(out_vx, out_vy, out_vyaw, cmd, motion_ok, move_attempted, move_result,
                      send_interval_ms);

  if (++status_tick_ >= static_cast<int>(params_.control_rate))
  {
    status_tick_ = 0;
    publishStatus();
  }
}

void Sw01Bridge::spin()
{
  rclcpp::WallRate rate(params_.control_rate);
  while (rclcpp::ok() && !stop_requested_.load())
  {
    step();
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
  motion_enabled_.store(false);

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
    sdk_->deinit();
    RCLCPP_INFO(node_->get_logger(), "ASTRALL SDK 已断开 / SDK deinitialized");
  }

  RCLCPP_WARN(node_->get_logger(),
              "节点已退出。机器人仍处于当前运动模式，若需要收车请通过遥控器切换到"
              "「锁定趴下」或「阻尼」状态");
}

}  // namespace sw01_ros_control
