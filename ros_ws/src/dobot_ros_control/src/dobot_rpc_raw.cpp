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
 * @file dobot_rpc_raw.cpp
 * @brief Built-in minimal JSON-RPC over TCP client (backend=raw) and RPC client factory.
 */
#include "dobot_ros_control/dobot_rpc.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace dobot_ros_control
{

const char* rpcCodeString(RpcCode c)
{
  switch (c)
  {
    case RpcCode::kOk: return "SUCCESS";
    case RpcCode::kSocketCreationFailed: return "SOCKET_CREATION_FAILED";
    case RpcCode::kConnectFailed: return "CONNECT_FAILED";
    case RpcCode::kNoResponse: return "NO_RESPONSE";
    case RpcCode::kJsonParseError: return "JSON_PARSE_ERROR";
    case RpcCode::kUnexpectedResponse: return "UNEXPECTED_RESPONSE";
    case RpcCode::kServerInternalError: return "SERVER_INTERNAL_ERROR";
    case RpcCode::kFsmTransitionFailed: return "FSM_TRANSITION_FAILED";
    case RpcCode::kUpperLimbFailed: return "SWITCH_UPPER_LIMB_CONTROL_FAILED";
    case RpcCode::kSetVelFailed: return "SET_VEL_FAILED";
    case RpcCode::kActionFailed: return "EXECUTE_ACTION_FAILED";
    case RpcCode::kNotImplemented: return "NOT_IMPLEMENTED";
    default: return "UNKNOWN";
  }
}

#if DOBOT_USE_SDK
std::unique_ptr<RpcClient> makeSdkRpcClient();
#endif

namespace
{

/** @brief Minimal blocking JSON-RPC over TCP implementation. */
class RawRpcClient : public RpcClient
{
public:
  const char* backendName() const override { return "raw"; }

  bool connect(const std::string& ip, int port, int timeout_ms, std::string* err) override
  {
    ip_ = ip;
    port_ = port;
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
    {
      if (err) *err = std::string("socket(): ") + std::strerror(errno);
      return false;
    }
    int one = 1;
    ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
    {
      if (err) *err = "inet_pton failed: " + ip;
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    // Non-blocking connect + select for connection timeout
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc < 0 && errno == EINPROGRESS)
    {
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(fd_, &wfds);
      timeval tv{};
      tv.tv_sec = timeout_ms / 1000;
      tv.tv_usec = (timeout_ms % 1000) * 1000;
      rc = ::select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
      if (rc <= 0)
      {
        if (err) *err = (rc == 0) ? "connect timeout" : std::string("select: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
      }
      int soerr = 0;
      socklen_t len = sizeof(soerr);
      ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &soerr, &len);
      if (soerr != 0)
      {
        if (err) *err = std::string("connect: ") + std::strerror(soerr);
        ::close(fd_);
        fd_ = -1;
        return false;
      }
    }
    else if (rc < 0)
    {
      if (err) *err = std::string("connect: ") + std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    // Restore blocking mode with socket read/write timeouts
    ::fcntl(fd_, F_SETFL, flags);
    timeval rtv{};
    rtv.tv_sec = timeout_ms / 1000;
    rtv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));
    ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &rtv, sizeof(rtv));
    return true;
  }

  RpcCode getFsmId(int32_t* fsm_id) override
  {
    std::string resp;
    RpcCode rc = call("GetFsmId", "{}", &resp);
    if (rc != RpcCode::kOk) return rc;
    int32_t v = 0;
    if (!extractResultInt(resp, &v)) return RpcCode::kJsonParseError;
    if (fsm_id) *fsm_id = v;
    return RpcCode::kOk;
  }

  RpcCode setFsmId(int32_t fsm_id) override
  {
    std::ostringstream p;
    p << "{\"fsm_id\":" << fsm_id << "}";
    return call("SetFsmId", p.str(), nullptr);
  }

  RpcCode setVel(float vx, float vy, float vyaw, float duration) override
  {
    std::ostringstream p;
    p.setf(std::ios::fixed);
    p.precision(4);
    p << "{\"vx\":" << vx << ",\"vy\":" << vy << ",\"vyaw\":" << vyaw << ",\"duration\":" << duration << "}";
    return call("SetVel", p.str(), nullptr);
  }

  RpcCode setControlMode(int32_t mode) override
  {
    std::ostringstream p;
    p << "{\"control_mode\":" << mode << "}";
    return call("SetControlMode", p.str(), nullptr);
  }

  RpcCode executeAction(const std::string& name) override
  {
    std::ostringstream p;
    p << "{\"file_path\":\"" << escape(name) << "\"}";
    return call("executeAction", p.str(), nullptr);
  }

  RpcCode getFsmList(std::string* json_out) override
  {
    std::string resp;
    RpcCode rc = call("GetFsmList", "{}", &resp);
    if (rc == RpcCode::kOk && json_out) *json_out = resp;
    return rc;
  }

private:
  static std::string escape(const std::string& s)
  {
    std::string o;
    for (char c : s)
    {
      if (c == '"' || c == '\\') o.push_back('\\');
      o.push_back(c);
    }
    return o;
  }

  /** @brief Send one JSON-RPC request and read the response. */
  RpcCode call(const std::string& method, const std::string& params, std::string* resp_out)
  {
    if (fd_ < 0) return RpcCode::kConnectFailed;
    std::ostringstream req;
    req << "{\"jsonrpc\":\"2.0\",\"id\":" << (++seq_) << ",\"method\":\"" << method
        << "\",\"params\":" << params << "}";
    const std::string body = req.str();
    ssize_t sent = ::send(fd_, body.data(), body.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(body.size())) return RpcCode::kNoResponse;

    // Read a complete JSON object by matching braces
    std::string buf;
    int depth = 0;
    bool started = false;
    char ch = 0;
    while (true)
    {
      ssize_t n = ::recv(fd_, &ch, 1, 0);
      if (n <= 0) return RpcCode::kNoResponse;
      buf.push_back(ch);
      if (ch == '{') { ++depth; started = true; }
      else if (ch == '}') { --depth; }
      if (started && depth == 0) break;
      if (buf.size() > 65536) return RpcCode::kUnexpectedResponse;
    }
    if (resp_out) *resp_out = buf;

    if (buf.find("\"error\"") != std::string::npos && buf.find("\"result\"") == std::string::npos)
      return RpcCode::kUnexpectedResponse;

    int32_t v = 0;
    if (!extractResultInt(buf, &v)) return RpcCode::kJsonParseError;
    if (v == 0) return RpcCode::kOk;
    if (v == -1) return RpcCode::kServerInternalError;
    return static_cast<RpcCode>(v);
  }

  /** @brief Extract integer "result" field from response JSON. */
  static bool extractResultInt(const std::string& s, int32_t* out)
  {
    size_t k = s.find("\"result\"");
    if (k == std::string::npos) return false;
    size_t c = s.find(':', k);
    if (c == std::string::npos) return false;
    size_t i = c + 1;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    bool neg = false;
    if (i < s.size() && s[i] == '-') { neg = true; ++i; }
    long v = 0;
    bool any = false;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9') { v = v * 10 + (s[i] - '0'); ++i; any = true; }
    if (!any) return false;
    *out = static_cast<int32_t>(neg ? -v : v);
    return true;
  }

  int fd_ = -1;
  std::string ip_;
  int port_ = 51234;
  int seq_ = 0;
};

}  // namespace

std::unique_ptr<RpcClient> makeRpcClient(const std::string& backend)
{
#if DOBOT_USE_SDK
  if (backend == "sdk") return makeSdkRpcClient();
  (void)backend;
#else
  (void)backend;
#endif
  return std::unique_ptr<RpcClient>(new RawRpcClient());
}

bool rpcBackendAvailable(const std::string& backend)
{
  if (backend == "raw") return true;
#if DOBOT_USE_SDK
  if (backend == "sdk") return true;
#endif
  return false;
}

}  // namespace dobot_ros_control
