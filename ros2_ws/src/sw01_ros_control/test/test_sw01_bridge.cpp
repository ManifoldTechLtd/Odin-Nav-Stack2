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
 * @file test_sw01_bridge.cpp
 * @brief Sw01Bridge behaviour against a scripted SDK and an injected steady clock.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "fake_chassis_sdk.h"
#include "sw01_ros_control/sw01_bridge.h"

using namespace std::chrono_literals;
using sw01_ros_control::Sw01Bridge;
using sw01_ros_control::test::FakeChassisSdk;
using sw01_ros_control::test::MoveCall;

namespace
{

class ManualClock
{
public:
  std::chrono::steady_clock::time_point now() const
  {
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns_.load()));
  }

  void advance(double seconds)
  {
    ns_ += static_cast<int64_t>(seconds * 1e9);
  }

private:
  std::atomic<int64_t> ns_{1000LL * 1000000000LL};
};

class BridgeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void TearDown() override
  {
    if (bridge_)
    {
      bridge_->shutdown();
    }
    bridge_.reset();
    enable_client_.reset();
    cmd_pub_.reset();
    executor_.reset();
    node_.reset();
  }

  void start(std::vector<rclcpp::Parameter> params = {})
  {
    static int counter = 0;
    params.emplace_back("cmd_vel_topic", "cmd_vel");
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    options.use_intra_process_comms(true);
    node_ = std::make_shared<rclcpp::Node>(
        "sw01_cmd_vel_bridge", "/bridge_test_" + std::to_string(counter++), options);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>("~/enable");
    bridge_ = std::make_unique<Sw01Bridge>(node_, sdk_, [this] { return clock_.now(); });
    ASSERT_TRUE(bridge_->init());
  }

  void publishCmd(double vx, double vy, double wz)
  {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = vx;
    msg->linear.y = vy;
    msg->angular.z = wz;
    cmd_pub_->publish(std::move(msg));
    for (int i = 0; i < 5; ++i)
    {
      executor_->spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  bool callEnable(bool on, std::string* message = nullptr)
  {
    EXPECT_TRUE(enable_client_->wait_for_service(2s));
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = on;
    auto future = enable_client_->async_send_request(request);
    EXPECT_EQ(executor_->spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    auto response = future.get();
    if (message != nullptr)
    {
      *message = response->message;
    }
    return response->success;
  }

  MoveCall lastMove()
  {
    const auto moves = sdk_->moves();
    EXPECT_FALSE(moves.empty());
    if (moves.empty())
    {
      return MoveCall{NAN, NAN, NAN};
    }
    return moves.back();
  }

  std::shared_ptr<FakeChassisSdk> sdk_ = std::make_shared<FakeChassisSdk>();
  ManualClock clock_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  std::unique_ptr<Sw01Bridge> bridge_;
};

TEST_F(BridgeTest, DefaultsAreSafe)
{
  start();
  EXPECT_TRUE(bridge_->params().require_enable);
  EXPECT_FALSE(bridge_->params().auto_enter_move_mode);
  EXPECT_EQ(bridge_->params().auth_timeout, 200);
  EXPECT_FALSE(bridge_->motionEnabled());
}

TEST_F(BridgeTest, StartsDisabledAndSendsZero)
{
  start();
  publishCmd(0.3, 0.0, 0.2);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  const MoveCall m = lastMove();
  EXPECT_FLOAT_EQ(m.vx, 0.0f);
  EXPECT_FLOAT_EQ(m.vy, 0.0f);
  EXPECT_FLOAT_EQ(m.vyaw, 0.0f);
}

TEST_F(BridgeTest, EnabledBridgeSendsFreshCommand)
{
  start();
  ASSERT_TRUE(callEnable(true));
  EXPECT_TRUE(bridge_->motionEnabled());
  publishCmd(0.3, 0.1, 0.2);
  bridge_->step();
  const MoveCall m = lastMove();
  EXPECT_FLOAT_EQ(m.vx, 0.3f);
  EXPECT_FLOAT_EQ(m.vy, 0.1f);
  EXPECT_FLOAT_EQ(m.vyaw, 0.2f);
}

TEST_F(BridgeTest, CommandsBeforeEnableAreDiscarded)
{
  start();
  publishCmd(0.3, 0.0, 0.0);
  ASSERT_TRUE(callEnable(true));
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
  publishCmd(0.2, 0.0, 0.0);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.2f);
}

TEST_F(BridgeTest, EnableRejectedWithoutAuthority)
{
  sdk_->setAuthResult(ASTRALL_RES_RC_NORELEASE);
  start();
  std::string message;
  EXPECT_FALSE(callEnable(true, &message));
  EXPECT_NE(message.find("authority"), std::string::npos);
  EXPECT_FALSE(bridge_->motionEnabled());
}

TEST_F(BridgeTest, EnableRejectedWhenFaulted)
{
  sdk_->setSystem(ASTRALL_SYSTEM_CODE_ERROR, 0);
  start();
  bridge_->step();
  EXPECT_FALSE(callEnable(true));
  EXPECT_FALSE(bridge_->motionEnabled());
}

TEST_F(BridgeTest, EnableRejectedWhenLinkLost)
{
  start();
  sdk_->emitStatus(false, true);
  EXPECT_FALSE(callEnable(true));
  EXPECT_FALSE(bridge_->motionEnabled());
}

TEST_F(BridgeTest, OperatorDisableStopsMotion)
{
  start();
  ASSERT_TRUE(callEnable(true));
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);
  ASSERT_TRUE(callEnable(false));
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, AuthorityLossDisablesAndRegainDoesNotReEnable)
{
  start();
  ASSERT_TRUE(callEnable(true));
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);

  // Remote controller seizes authority and keeps it.
  sdk_->setAuthResult(ASTRALL_RES_RC_NORELEASE);
  sdk_->emitStatus(true, false);
  publishCmd(0.3, 0.0, 0.0);
  const size_t moves_before = sdk_->moves().size();
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_EQ(sdk_->moves().size(), moves_before);

  // Remote controller releases; the periodic retry regains authority.
  sdk_->setAuthResult(ASTRALL_RES_SUCCESSED);
  clock_.advance(2.1);
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, BlockingAuthorityRequestDoesNotSendStaleCommand)
{
  start({rclcpp::Parameter("require_enable", false)});
  publishCmd(0.3, 0.0, 0.0);
  sdk_->setOnAuth([this] { clock_.advance(1.0); });
  sdk_->emitStatus(true, false);
  clock_.advance(0.1);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, CommandTimeoutUsesInjectedSteadyClock)
{
  start({rclcpp::Parameter("require_enable", false)});
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);
  clock_.advance(0.4);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);
  clock_.advance(0.2);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, DecayWindowScalesLateralVelocity)
{
  start({rclcpp::Parameter("require_enable", false),
         rclcpp::Parameter("cmd_vel_decay_window", 0.2)});
  publishCmd(0.4, 0.2, 0.4);
  clock_.advance(0.4);
  bridge_->step();
  const MoveCall m = lastMove();
  EXPECT_NEAR(m.vx, 0.2, 1e-5);
  EXPECT_NEAR(m.vy, 0.1, 1e-5);
  EXPECT_NEAR(m.vyaw, 0.2, 1e-5);
}

TEST_F(BridgeTest, NoPostureChangeWhenDisabled)
{
  sdk_->setSport(ASTRALL_SPORT_STATUS_DAMPING);
  start({rclcpp::Parameter("auto_enter_move_mode", true)});
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_TRUE(sdk_->modeCommands().empty());
}

TEST_F(BridgeTest, NoPostureChangeWithoutFreshCommand)
{
  sdk_->setSport(ASTRALL_SPORT_STATUS_DAMPING);
  start({rclcpp::Parameter("auto_enter_move_mode", true)});
  ASSERT_TRUE(callEnable(true));
  bridge_->step();
  EXPECT_TRUE(sdk_->modeCommands().empty());
}

TEST_F(BridgeTest, PostureSequenceWhenEnabledAndCommanded)
{
  sdk_->setSport(ASTRALL_SPORT_STATUS_DAMPING);
  start({rclcpp::Parameter("auto_enter_move_mode", true)});
  ASSERT_TRUE(callEnable(true));
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  sdk_->setSport(ASTRALL_SPORT_STATUS_FIXEDSTAND);
  bridge_->step();
  const auto cmds = sdk_->modeCommands();
  ASSERT_EQ(cmds.size(), 2u);
  EXPECT_EQ(cmds[0], ASTRALL_SPORT_MODE_CMD_FIXEDSTAND);
  EXPECT_EQ(cmds[1], ASTRALL_SPORT_MODE_CMD_MOVE);
  sdk_->setSport(ASTRALL_SPORT_STATUS_MOVE);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);
}

TEST_F(BridgeTest, FaultDisablesMotion)
{
  start();
  ASSERT_TRUE(callEnable(true));
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FLOAT_EQ(lastMove().vx, 0.3f);
  sdk_->setSystem(ASTRALL_SYSTEM_CODE_ERROR, 0);
  clock_.advance(0.6);
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, BlockingErrorMaskTriggersFault)
{
  sdk_->setSystem(ASTRALL_SYSTEM_CODE_RUNNING, ASTRALL_ERR_LOSE_CAMERA);
  start({rclcpp::Parameter("blocking_error_mask", static_cast<int64_t>(ASTRALL_ERR_MOTOR_STATUS))});
  bridge_->step();
  EXPECT_TRUE(callEnable(true));
  sdk_->setSystem(ASTRALL_SYSTEM_CODE_RUNNING, ASTRALL_ERR_MOTOR_STATUS);
  clock_.advance(0.6);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_FALSE(callEnable(true));
}

TEST_F(BridgeTest, LinkLossDisablesMotion)
{
  start();
  ASSERT_TRUE(callEnable(true));
  sdk_->emitStatus(false, true);
  publishCmd(0.3, 0.0, 0.0);
  bridge_->step();
  EXPECT_FALSE(bridge_->motionEnabled());
  EXPECT_FLOAT_EQ(lastMove().vx, 0.0f);
}

TEST_F(BridgeTest, ParameterValidation)
{
  start({rclcpp::Parameter("auth_timeout", 2000), rclcpp::Parameter("control_rate", 10.0),
         rclcpp::Parameter("cmd_vel_decay_window", 0.8)});
  EXPECT_EQ(bridge_->params().auth_timeout, 400);
  EXPECT_DOUBLE_EQ(bridge_->params().control_rate, 50.0);
  EXPECT_DOUBLE_EQ(bridge_->params().cmd_vel_decay_window, bridge_->params().cmd_vel_timeout);
}

TEST_F(BridgeTest, TooSmallAuthTimeoutFallsBackToDefault)
{
  start({rclcpp::Parameter("auth_timeout", 20)});
  EXPECT_EQ(bridge_->params().auth_timeout, 200);
}

TEST_F(BridgeTest, HeartbeatIssuedBeforeAuthorityRequest)
{
  start();
  const auto calls = sdk_->calls();
  const auto heartbeat = std::find(calls.begin(), calls.end(), "heartbeat");
  const auto auth = std::find(calls.begin(), calls.end(), "auth");
  ASSERT_NE(heartbeat, calls.end());
  ASSERT_NE(auth, calls.end());
  EXPECT_LT(heartbeat - calls.begin(), auth - calls.begin());
}

}  // namespace
