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
 * @file astrall_chassis_sdk.cpp
 * @brief ChassisSdk implementation forwarding to the ASTRALL SDK free functions.
 */
#include "sw01_ros_control/chassis_sdk.h"

namespace sw01_ros_control
{
namespace
{

class AstrallChassisSdk : public ChassisSdk
{
public:
  uint16_t init(AstrallConfig& cfg, uint32_t timeout_ms) override
  {
    return AstrallSdkInit(cfg, timeout_ms);
  }

  void deinit() override
  {
    AstrallSdkDeinit();
  }

  uint16_t heartbeat(uint32_t timeout_ms) override
  {
    return AstrallHeartbeat(timeout_ms);
  }

  uint16_t requestAuthority(uint32_t timeout_ms) override
  {
    return AstrallAuthControl(ASTRALL_AUTH_SDK, timeout_ms);
  }

  uint16_t sportStatus() override
  {
    return AstrallGetSportStatus();
  }

  uint16_t sportModeControl(AstrallSportModeCmd cmd, uint32_t timeout_ms) override
  {
    return AstrallSportModeControl(cmd, timeout_ms);
  }

  uint16_t move(float vx, float vy, float vyaw, uint32_t timeout_ms) override
  {
    return AstrallMove(vx, vy, vyaw, timeout_ms);
  }

  uint16_t systemStatus(AstrallSystemStatus& status) override
  {
    return AstrallGetSystemStatus(status);
  }

  uint16_t powerStatus(AstrallPowerStatus& power) override
  {
    return AstrallGetPowerStatus(power);
  }

  uint16_t deviceInfo(AstrallDeviceInfo& info, uint32_t timeout_ms) override
  {
    return AstrallGetDeviceInfo(info, timeout_ms);
  }
};

}  // namespace

std::shared_ptr<ChassisSdk> makeAstrallChassisSdk()
{
  return std::make_shared<AstrallChassisSdk>();
}

}  // namespace sw01_ros_control
