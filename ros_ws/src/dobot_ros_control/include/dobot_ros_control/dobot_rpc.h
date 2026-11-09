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
 * @file dobot_rpc.h
 * @brief Thin wrapper around Dobot Lumo high-level JSON-over-TCP RPC (switchable backend).
 *
 * Supported backends:
 *   sdk -- Official Atom::AlgsRpcClient (libclient_rpc.a), recommended.
 *   raw -- Built-in minimal JSON-RPC client (src/dobot_rpc_raw.cpp).
 */
#ifndef DOBOT_ROS_CONTROL_DOBOT_RPC_H
#define DOBOT_ROS_CONTROL_DOBOT_RPC_H

#include <cstdint>
#include <memory>
#include <string>

namespace dobot_ros_control
{

/** @brief Return codes aligned with Atom::RpcErrorCode. */
enum class RpcCode : int32_t
{
  kOk = 0,
  kSocketCreationFailed = 1,
  kConnectFailed = 2,
  kNoResponse = 3,
  kJsonParseError = 4,
  kUnexpectedResponse = 5,
  kServerInternalError = 6,
  kFsmTransitionFailed = 100,
  kUpperLimbFailed = 101,
  kSetVelFailed = 102,
  kActionFailed = 106,
  kNotImplemented = -1,  ///< Not supported by backend
};

const char* rpcCodeString(RpcCode code);

/** @brief Abstract interface for Lumo high-level RPC operations. */
class RpcClient
{
public:
  virtual ~RpcClient() = default;

  /** @brief Backend name ("sdk" or "raw"). */
  virtual const char* backendName() const = 0;

  /** @brief Establish TCP connection. Populates err on failure. */
  virtual bool connect(const std::string& ip, int port, int timeout_ms, std::string* err) = 0;

  /** @brief Query current FSM state ID. */
  virtual RpcCode getFsmId(int32_t* fsm_id) = 0;

  /** @brief Request FSM state transition. */
  virtual RpcCode setFsmId(int32_t fsm_id) = 0;

  /** @brief Send velocity command (vx forward+, vy left+, vyaw CCW+, duration in seconds). */
  virtual RpcCode setVel(float vx, float vy, float vyaw, float duration) = 0;

  /** @brief Set control authority mode: 0=gamepad, 1=APP, 2=third-party. */
  virtual RpcCode setControlMode(int32_t mode) = 0;

  /** @brief Execute built-in upper-body action by name. */
  virtual RpcCode executeAction(const std::string& name) = 0;

  /** @brief Query available FSM states (raw JSON array). */
  virtual RpcCode getFsmList(std::string* json_out) { return RpcCode::kNotImplemented; }
};

/** @brief Create an RPC client ("sdk" or "raw"). Falls back to "raw" if built without SDK. */
std::unique_ptr<RpcClient> makeRpcClient(const std::string& backend);

/** @brief Check whether the specified backend is compiled into this binary. */
bool rpcBackendAvailable(const std::string& backend);

}  // namespace dobot_ros_control

#endif  // DOBOT_ROS_CONTROL_DOBOT_RPC_H
