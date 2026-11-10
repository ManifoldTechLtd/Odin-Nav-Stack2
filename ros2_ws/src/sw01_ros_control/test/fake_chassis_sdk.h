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
 * @file fake_chassis_sdk.h
 * @brief Scripted ChassisSdk test double recording every SDK call in order.
 */
#ifndef SW01_ROS_CONTROL_TEST_FAKE_CHASSIS_SDK_H
#define SW01_ROS_CONTROL_TEST_FAKE_CHASSIS_SDK_H

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "sw01_ros_control/chassis_sdk.h"

namespace sw01_ros_control
{
namespace test
{

struct MoveCall
{
  float vx;
  float vy;
  float vyaw;
};

class FakeChassisSdk : public ChassisSdk
{
public:
  uint16_t init(AstrallConfig& cfg, uint32_t /*timeout_ms*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg_ = &cfg;
    calls_.push_back("init");
    return ASTRALL_RES_SUCCESSED;
  }

  void deinit() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back("deinit");
    cfg_ = nullptr;
  }

  uint16_t heartbeat(uint32_t /*timeout_ms*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back("heartbeat");
    return ASTRALL_RES_SUCCESSED;
  }

  uint16_t requestAuthority(uint32_t /*timeout_ms*/) override
  {
    std::function<void()> hook;
    uint16_t result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calls_.push_back("auth");
      hook = on_auth_;
      result = auth_result_;
    }
    if (hook)
    {
      hook();
    }
    return result;
  }

  uint16_t sportStatus() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sport_;
  }

  uint16_t sportModeControl(AstrallSportModeCmd cmd, uint32_t /*timeout_ms*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back("mode");
    mode_cmds_.push_back(cmd);
    return ASTRALL_RES_SUCCESSED;
  }

  uint16_t move(float vx, float vy, float vyaw, uint32_t /*timeout_ms*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back("move");
    moves_.push_back(MoveCall{vx, vy, vyaw});
    return ASTRALL_RES_SUCCESSED;
  }

  uint16_t systemStatus(AstrallSystemStatus& status) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status = system_;
    return ASTRALL_RES_SUCCESSED;
  }

  uint16_t powerStatus(AstrallPowerStatus& power) override
  {
    power = AstrallPowerStatus();
    power.soc = 80.0f;
    return ASTRALL_RES_SUCCESSED;
  }

  uint16_t deviceInfo(AstrallDeviceInfo& info, uint32_t /*timeout_ms*/) override
  {
    info = AstrallDeviceInfo();
    info.version = "fake";
    return ASTRALL_RES_SUCCESSED;
  }

  void setSport(uint16_t sport)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sport_ = sport;
  }

  void setAuthResult(uint16_t result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auth_result_ = result;
  }

  /** @brief Hook run inside requestAuthority(), e.g. to simulate a blocking request. */
  void setOnAuth(std::function<void()> hook)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_auth_ = std::move(hook);
  }

  void setSystem(AstrallSystemCode code, uint32_t error_bits)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    system_.sysStatus = code;
    system_.errorCode = static_cast<AstrallErrorCode>(error_bits);
  }

  /** @brief Invoke the registered SDK status callback like the SDK callback thread does. */
  void emitStatus(bool link, bool authority)
  {
    std::function<void(void*, uint16_t)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cfg_ != nullptr)
      {
        callback = cfg_->sdkStatusCb;
      }
    }
    AstrallSdkStatus status;
    status.link = link ? 1 : 0;
    status.ctrlAuthority = authority ? 1 : 0;
    if (callback)
    {
      callback(&status, sizeof(status));
    }
  }

  std::vector<MoveCall> moves()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return moves_;
  }

  std::vector<AstrallSportModeCmd> modeCommands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_cmds_;
  }

  std::vector<std::string> calls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

private:
  std::mutex mutex_;
  AstrallConfig* cfg_ = nullptr;
  uint16_t sport_ = ASTRALL_SPORT_STATUS_MOVE;
  uint16_t auth_result_ = ASTRALL_RES_SUCCESSED;
  std::function<void()> on_auth_;
  AstrallSystemStatus system_;
  std::vector<MoveCall> moves_;
  std::vector<AstrallSportModeCmd> mode_cmds_;
  std::vector<std::string> calls_;
};

}  // namespace test
}  // namespace sw01_ros_control

#endif  // SW01_ROS_CONTROL_TEST_FAKE_CHASSIS_SDK_H
