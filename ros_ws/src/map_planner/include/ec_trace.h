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

// Header-only diagnostic event code (EC) logger (ROS-independent, thread-safe, synchronous fsync).
//
// Usage:
//   #include "ec_trace.h"
//   EC_TRACE(ec::ASTAR_FAIL, "plan", "start=(%.2f,%.2f) goal=(%.2f,%.2f)", sx, sy, gx, gy);
#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace ec {

// Event code table (synchronized with NeuPAN/neupan/ros/ec_trace.py)
enum Code : int {
  // 0xx Lifecycle
  NODE_START = 1, NODE_READY = 2, NODE_EXIT = 3, NODE_START_FAIL = 10,
  // 1xxx Mode transitions
  MODE_ENTER = 1000, MODE_MPC = 1001, MODE_ESCAPE = 1002, MODE_PREALIGN = 1003,
  WAIT_INITIAL_PATH = 1004, PATH_INSTALL = 1005, PATH_REJECT = 1006,
  ADAPTIVE_DMAX = 1007, GRACE_TRIGGER = 1008, ESCAPE_WATCH = 1009,
  MPC_SOLVE = 1010, REPLAN_TRIGGER = 1011, PLAN_DONE = 1012,
  // 2xxx Performance and resources
  SOLVE_SLOW = 2000, LOOP_GAP = 2001, CPU_HIGH = 2002, DISK_LOW = 2003, HEAVY_ALLOC_PRE = 2004,
  // 3xxx NeuPAN
  NEUPAN_CRASH_PRE = 3000, NEUPAN_SOLVE_FAIL = 3001, NEUPAN_OBSTACLE_BURST = 3002,
  NEUPAN_LOOP_ALIVE = 3003, NEUPAN_FEEDBACK_FROZEN = 3004,
  // 4xxx Map planner
  ASTAR_FAIL = 4000, MAP_LOAD_FAIL = 4001, ASTAR_SLOW = 4002, PATH_INVALID = 4003,
  // 5xxx Chassis bridge
  SDK_CONNECT_FAIL = 5000, CMDVEL_STALL = 5001, CMDVEL_ABNORMAL = 5002,
  // 6xxx Perception driver
  DEVICE_OFFLINE = 6000, CLOUD_STALL = 6001, TF_FAIL = 6002,
  // 7xxx Environment dependencies
  PY_VER_MISMATCH = 7000, ABI_RISK = 7001, SDK_LIB_MISSING = 7002,
};

namespace detail {

inline std::string log_dir() {
  const char* env = std::getenv("EC_LOG_DIR");
  if (env && *env) return std::string(env);
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/base_inspection/logs";
}

inline void ensure_dir(const std::string& d) {
  ::mkdir(d.c_str(), 0755);
  std::string sub = d;
  for (size_t i = 1; i < sub.size(); ++i) {
    if (sub[i] == '/') { sub[i] = '\0'; ::mkdir(sub.c_str(), 0755); sub[i] = '/'; }
  }
}

inline const char* node_name() {
  const char* n = std::getenv("EC_NODE");
  return (n && *n) ? n : "unknown";
}

inline void write_line(int code, const char* phase, const char* detail) {
  static std::mutex mtx;
  std::lock_guard<std::mutex> lk(mtx);
  std::string dir = log_dir();
  ensure_dir(dir);
  std::FILE* fp = std::fopen((dir + "/ec_events.log").c_str(), "a");
  if (!fp) return;
  std::fprintf(fp, "%.3f [EC:%05d] 节点=%s 线程=%lu 阶段=%s 详情=%s\n",
               (double)::time(nullptr), code, node_name(),
               (unsigned long)::syscall(SYS_gettid), phase, detail ? detail : "");
  std::fflush(fp);
  ::fsync(::fileno(fp));
  std::fclose(fp);
}

}  // namespace detail

/// Write a formatted diagnostic event code entry.
inline void trace(int code, const char* phase, const char* fmt, ...) {
  char buf[512];
  if (fmt && *fmt) {
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
  } else {
    buf[0] = '\0';
  }
  detail::write_line(code, phase, buf);
}

}  // namespace ec

#define EC_TRACE(code, phase, ...) ::ec::trace((code), (phase), __VA_ARGS__)
