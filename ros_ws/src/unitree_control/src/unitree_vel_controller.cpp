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

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <functional>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include "ec_trace.h"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <mutex>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <net/if.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// Resolve DDS network interface dynamically:
//   1) UNITREE_NETWORK_INTERFACE environment variable
//   2) Interface holding a 192.168.123.0/24 IPv4 address (Unitree SDK subnet)
//   3) eth0 -> eth10 -> fallback eth0
static inline const char* resolve_network_interface()
{
  static char cached[IFNAMSIZ] = {0};
  if (cached[0]) return cached;

  const char* env = std::getenv("UNITREE_NETWORK_INTERFACE");
  if (env && *env) {
    std::snprintf(cached, sizeof(cached), "%s", env);
    return cached;
  }
  struct ifaddrs* ifs = nullptr;
  if (getifaddrs(&ifs) == 0) {
    for (struct ifaddrs* p = ifs; p; p = p->ifa_next) {
      if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
      auto* sin = reinterpret_cast<struct sockaddr_in*>(p->ifa_addr);
      const uint32_t ip = ntohl(sin->sin_addr.s_addr);
      if ((ip & 0xFFFFFF00u) == 0xC0A87B00u) {  // 192.168.123.0/24
        std::snprintf(cached, sizeof(cached), "%s", p->ifa_name);
        break;
      }
    }
    freeifaddrs(ifs);
  }
  if (!cached[0] && if_nametoindex("eth0") > 0)  std::snprintf(cached, sizeof(cached), "eth0");
  if (!cached[0] && if_nametoindex("eth10") > 0) std::snprintf(cached, sizeof(cached), "eth10");
  if (!cached[0]) std::snprintf(cached, sizeof(cached), "eth0");
  return cached;
}
#define NETWORK_INTERFACE resolve_network_interface()
using namespace unitree::common;

class Custom
{
public:
  Custom(ros::NodeHandle &nh) : should_exit(false)
  {
    // Initialize Unitree SportClient
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    sport_client.StandUp();
    sleep(1);
    sport_client.BalanceStand();
    sleep(1);

    if (sport_client.FreeWalk() == 0) {
      std::cout << "Switched to AI Walk mode." << std::endl;
    } else {
      std::cerr << "Failed to switch to AI Walk mode." << std::endl;
    }
    if (sport_client.FreeAvoid(false) == 0) {
      std::cout << "Disabled obstacle avoidance." << std::endl;
    } else {
      std::cerr << "Failed to disable obstacle avoidance." << std::endl;
    }

    change_walk_mode_srv = nh.advertiseService("unitree/classic_walk_mode", &Custom::ChangeWalkMode, this);

    cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &Custom::CmdVelCallback, this);
    // Publish SportModeState (chassis internal state) for diagnostics.
    // Layout: [0]error_code [1]mode [2]gait_type [3..5]velocity [6]yaw_speed
    //         [7..10]foot_force [11..14]range_obstacle [15..18]foot_z
    state_pub = nh.advertise<std_msgs::Float32MultiArray>("/unitree/sport_mode_state", 10);
    state_sub.InitChannel(std::bind(&Custom::StateCallback, this, std::placeholders::_1));
    ::setenv("EC_NODE", "unitree_vel_controller", 1);
    EC_TRACE(ec::NODE_READY, "startup", "unitree SDK ready (stand+freewalk done)");
  }

  ~Custom()
  {
    SafeShutdown();
  }
  void SafeShutdown()
  {
    if (!should_exit.exchange(true)) 
    {
      std::cout << "Initiating safe shutdown..." << std::endl;
      sport_client.StopMove();
      sport_client.StandDown();
      std::cout << "Safety procedures completed." << std::endl;
    }
  }

private:
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    int32_t ret = sport_client.Move(msg->linear.x, msg->linear.y, msg->angular.z);
    if (ret != 0) {
      ROS_WARN_THROTTLE(1.0, "[vel_to_sdk] Move failed: ret=%d cmd=(%.3f, %.3f, %.3f)",
                        ret, msg->linear.x, msg->linear.y, msg->angular.z);
    }
  }
  void StateCallback(const void* msg)
  {
    const auto* s = static_cast<const unitree_go::msg::dds_::SportModeState_*>(msg);
    std_msgs::Float32MultiArray out;
    out.data.reserve(19);
    out.data.push_back(static_cast<float>(s->error_code()));
    out.data.push_back(static_cast<float>(s->mode()));
    out.data.push_back(static_cast<float>(s->gait_type()));
    out.data.push_back(s->velocity()[0]);
    out.data.push_back(s->velocity()[1]);
    out.data.push_back(s->velocity()[2]);
    out.data.push_back(s->yaw_speed());
    for (int i = 0; i < 4; ++i) out.data.push_back(static_cast<float>(s->foot_force()[i]));
    for (int i = 0; i < 4; ++i) out.data.push_back(s->range_obstacle()[i]);
    for (int i = 0; i < 4; ++i) out.data.push_back(s->foot_position_body()[i * 3 + 2]);
    state_pub.publish(out);
  }
  bool ChangeWalkMode(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res) {
    if (req.data) {
      std::cout << "Switching to ClassicWalk mode" << std::endl;
    } else {
      std::cout << "Switching to AI walk mode" << std::endl;
    }
    int result = sport_client.ClassicWalk(req.data);
    res.success = result == 0;
    if (!res.success) {
      std::cerr << "Failed to switch walk mode with error code " << result << std::endl;
    }
    res.message = std::string("Switch walk mode ") + 
      (res.success ? "succeeded." : "failed with error code " + std::to_string(result));
    return true;
  }
  unitree::robot::go2::SportClient sport_client;

  ros::ServiceServer change_walk_mode_srv;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher state_pub;
  unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> state_sub{"rt/sportmodestate"};

  std::atomic<bool> should_exit;
};

Custom* global_custom = nullptr;

void signalHandler(int signum)
{
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  if (global_custom != nullptr) {
    global_custom->SafeShutdown();
  }
  ros::shutdown();
}

int main(int argc, char **argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
  ros::init(argc, argv, "unitree_cmd_vel_controller");
  ros::NodeHandle nh;

  Custom custom(nh);
  global_custom = &custom;
 
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  ros::spin();
  global_custom = nullptr;
  return 0;
}
