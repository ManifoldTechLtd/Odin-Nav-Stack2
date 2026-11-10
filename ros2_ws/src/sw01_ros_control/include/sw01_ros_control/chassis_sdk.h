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
 * @file chassis_sdk.h
 * @brief Seam over the ASTRALL SDK calls used by the SW01 bridge.
 *
 * The production implementation forwards to the SDK free functions; tests
 * substitute a scripted double. ASTRALL SDK 1.0.7 serializes every command API
 * (heartbeat / authority / sport mode / move) on one internal mutex and blocks
 * until the robot acknowledges or the timeout (ms) expires, so a long
 * requestAuthority() also delays heartbeats.
 */
#ifndef SW01_ROS_CONTROL_CHASSIS_SDK_H
#define SW01_ROS_CONTROL_CHASSIS_SDK_H

#include <cstdint>
#include <memory>

// ASTRALL SDK header (see sdk/README_SDK_SETUP.md)
#include "interface.h"

namespace sw01_ros_control
{

class ChassisSdk
{
public:
  virtual ~ChassisSdk() = default;

  /** @brief Initialize the SDK; @p cfg must outlive the SDK session. */
  virtual uint16_t init(AstrallConfig& cfg, uint32_t timeout_ms) = 0;
  virtual void deinit() = 0;
  virtual uint16_t heartbeat(uint32_t timeout_ms) = 0;
  /** @brief Request SDK control authority (AstrallAuthControl(ASTRALL_AUTH_SDK)). */
  virtual uint16_t requestAuthority(uint32_t timeout_ms) = 0;
  /** @brief Cached sport status (non-blocking). */
  virtual uint16_t sportStatus() = 0;
  virtual uint16_t sportModeControl(AstrallSportModeCmd cmd, uint32_t timeout_ms) = 0;
  virtual uint16_t move(float vx, float vy, float vyaw, uint32_t timeout_ms) = 0;
  /** @brief Cached system status (non-blocking). */
  virtual uint16_t systemStatus(AstrallSystemStatus& status) = 0;
  /** @brief Cached power status (non-blocking). */
  virtual uint16_t powerStatus(AstrallPowerStatus& power) = 0;
  virtual uint16_t deviceInfo(AstrallDeviceInfo& info, uint32_t timeout_ms) = 0;
};

/** @brief Create the production implementation backed by libASTRALL_SDK. */
std::shared_ptr<ChassisSdk> makeAstrallChassisSdk();

}  // namespace sw01_ros_control

#endif  // SW01_ROS_CONTROL_CHASSIS_SDK_H
