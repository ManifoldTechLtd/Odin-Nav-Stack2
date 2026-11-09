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
 * @file dobot_rpc_sdk.cpp
 * @brief Official Lumo SDK RPC backend (Atom::AlgsRpcClient).
 *
 * Compiled only when lumo_sdk is found at build time (CMake: DOBOT_USE_SDK=ON).
 */
#include "dobot_ros_control/dobot_rpc.h"

#if DOBOT_USE_SDK

#include <chrono>
#include <memory>

#include "rpc/algs_rpc_client.h"

namespace dobot_ros_control
{

namespace
{

RpcCode fromAtom(Atom::RpcErrorCode c)
{
  switch (c)
  {
    case Atom::RpcErrorCode::SUCCESS: return RpcCode::kOk;
    case Atom::RpcErrorCode::SOCKET_CREATION_FAILED: return RpcCode::kSocketCreationFailed;
    case Atom::RpcErrorCode::CONNECT_FAILED: return RpcCode::kConnectFailed;
    case Atom::RpcErrorCode::NO_RESPONSE: return RpcCode::kNoResponse;
    case Atom::RpcErrorCode::JSON_PARSE_ERROR: return RpcCode::kJsonParseError;
    case Atom::RpcErrorCode::UNEXPECTED_RESPONSE: return RpcCode::kUnexpectedResponse;
    case Atom::RpcErrorCode::SERVER_INTERNAL_ERROR: return RpcCode::kServerInternalError;
    case Atom::RpcErrorCode::FSM_TRANSITION_FAILED: return RpcCode::kFsmTransitionFailed;
    case Atom::RpcErrorCode::SWITCH_UPPER_LIMB_CONTROL_FAILED: return RpcCode::kUpperLimbFailed;
    case Atom::RpcErrorCode::SET_VEL_FAILED: return RpcCode::kSetVelFailed;
    case Atom::RpcErrorCode::EXECUTE_ACTION_FAILED: return RpcCode::kActionFailed;
    default: return RpcCode::kUnexpectedResponse;
  }
}

class SdkRpcClient : public RpcClient
{
public:
  const char* backendName() const override { return "sdk"; }

  bool connect(const std::string& ip, int port, int /*timeout_ms*/, std::string* err) override
  {
    ip_ = ip;
    port_ = port;
    // AlgsRpcClient connects lazily on first request; probe with GetFsmId.
    rpc_.reset(new Atom::AlgsRpcClient(ip, port));
    int32_t fsm_id = 0;
    const auto t0 = std::chrono::steady_clock::now();
    Atom::RpcErrorCode rc = rpc_->GetFsmId(fsm_id);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
    if (rc != Atom::RpcErrorCode::SUCCESS)
    {
      if (err)
      {
        *err = std::string("GetFsmId probe failed: ") + rpcCodeString(fromAtom(rc)) +
               " (ip=" + ip_ + ", port=" + std::to_string(port_) + ")";
      }
      rpc_.reset();
      return false;
    }
    probe_fsm_id_ = fsm_id;
    probe_ms_ = ms;
    return true;
  }

  /** @brief Initial FSM ID and latency measured during connection probe. */
  int32_t probeFsmId() const { return probe_fsm_id_; }
  long probeMs() const { return probe_ms_; }

  RpcCode getFsmId(int32_t* fsm_id) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    int32_t v = 0;
    RpcCode rc = fromAtom(rpc_->GetFsmId(v));
    if (rc == RpcCode::kOk && fsm_id) *fsm_id = v;
    return rc;
  }

  RpcCode setFsmId(int32_t fsm_id) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    return fromAtom(rpc_->SetFsmId(fsm_id));
  }

  RpcCode setVel(float vx, float vy, float vyaw, float duration) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    const auto t0 = std::chrono::steady_clock::now();
    RpcCode rc = fromAtom(rpc_->SetVel(vx, vy, vyaw, duration));
    last_setvel_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now() - t0).count() / 1000.0;
    return rc;
  }

  RpcCode setControlMode(int32_t mode) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    return fromAtom(rpc_->SetControlMode(mode));
  }

  RpcCode executeAction(const std::string& name) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    return fromAtom(rpc_->executeAction(name));
  }

  RpcCode getFsmList(std::string* json_out) override
  {
    if (!rpc_) return RpcCode::kConnectFailed;
    nlohmann::json list;
    RpcCode rc = fromAtom(rpc_->GetFsmList(&list));
    if (rc == RpcCode::kOk && json_out) *json_out = list.dump();
    return rc;
  }

  /** @brief Duration (ms) of the most recent SetVel call. */
  double lastSetVelMs() const { return last_setvel_ms_; }

private:
  std::unique_ptr<Atom::AlgsRpcClient> rpc_;
  std::string ip_;
  int port_ = 51234;
  int32_t probe_fsm_id_ = -1;
  long probe_ms_ = -1;
  double last_setvel_ms_ = -1.0;
};

}  // namespace

std::unique_ptr<RpcClient> makeSdkRpcClient()
{
  return std::unique_ptr<RpcClient>(new SdkRpcClient());
}

}  // namespace dobot_ros_control

#endif  // DOBOT_USE_SDK
