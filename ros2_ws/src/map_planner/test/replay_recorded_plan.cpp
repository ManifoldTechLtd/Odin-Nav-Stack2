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

#include "map_planner.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

#include <tf2_msgs/msg/tf_message.hpp>

template <class Message>
Message readCdr(const std::string& filename) {
  std::ifstream input(filename, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read fixture: " + filename);
  const std::vector<unsigned char> bytes{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& buffer = serialized.get_rcl_serialized_message();
  std::memcpy(buffer.buffer, bytes.data(), bytes.size());
  buffer.buffer_length = bytes.size();
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

class MapPlannerRecordedReplay {
public:
  static bool run(MapPlanner& planner, const std::string& directory, bool serve) {
    const auto transforms = readCdr<tf2_msgs::msg::TFMessage>(directory + "/tf.cdr");
    for (const auto& transform : transforms.transforms) {
      if (!planner.tf_buffer_->setTransform(transform, "recorded_fixture", true)) {
        throw std::runtime_error("Invalid recorded TF");
      }
    }
    auto fake = readCdr<nav_msgs::msg::OccupancyGrid>(directory + "/fake.cdr");
    fake.header.stamp = planner.node_->now();
    planner.slidingMapCallback(std::make_shared<nav_msgs::msg::OccupancyGrid>(fake));
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(
        readCdr<nav_msgs::msg::OccupancyGrid>(directory + "/map.cdr"));
    planner.mapCallback(map);
    if (serve) {
      std::cout << "RECORDED_MAP_READY" << std::endl;
      rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
      executor.add_node(planner.node_);
      executor.spin();
      return true;
    }
    geometry_msgs::msg::PoseStamped start;
    if (!planner.getRobotPose(start)) throw std::runtime_error("Recorded robot TF unavailable");
    auto goal = readCdr<geometry_msgs::msg::PoseStamped>(directory + "/goal.cdr");
    goal.header.stamp = planner.node_->now();
    // Geometry is frozen; rebase time only after static-map initialization.
    fake.header.stamp = planner.node_->now();
    planner.slidingMapCallback(std::make_shared<nav_msgs::msg::OccupancyGrid>(fake));
    nav_msgs::msg::Path path;
    std::string reason;
    const auto begin = std::chrono::steady_clock::now();
    const bool success = planner.plan(start, goal, path, reason);
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    const bool collision_free = success && planner.collisionFreePath(path, planner.path_min_clear_m_);
    std::cout << "RECORDED_PLAN start=(" << start.pose.position.x << ","
              << start.pose.position.y << ") goal=(" << goal.pose.position.x << ","
              << goal.pose.position.y << ") success=" << success
              << " collision_free=" << collision_free << " poses=" << path.poses.size()
              << " elapsed_ms=" << elapsed << " reason=" << reason << "\n";
    if (success) {
      rclcpp::SerializedMessage serialized;
      rclcpp::Serialization<nav_msgs::msg::Path>().serialize_message(&path, &serialized);
      const auto& bytes = serialized.get_rcl_serialized_message();
      std::ofstream output(directory + "/planned_path.cdr", std::ios::binary);
      output.write(reinterpret_cast<const char*>(bytes.buffer), bytes.buffer_length);
    }
    return success && collision_free;
  }
};

int main(int argc, char** argv) {
  if (argc < 2 || !std::getenv("ROS_DOMAIN_ID") ||
      std::string(std::getenv("ROS_DOMAIN_ID")) != "184" ||
      !std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") ||
      std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")) != "LOCALHOST") {
    std::cerr << "Requires a CDR fixture directory and localhost test domain 184\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("map_planner");
    MapPlanner planner(node);
    bool serve = false;
    for (int i = 2; i < argc; ++i) serve = serve || std::string(argv[i]) == "--serve";
    const bool success = MapPlannerRecordedReplay::run(planner, argv[1], serve);
    rclcpp::shutdown();
    return success ? 0 : 1;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << "\n";
    rclcpp::shutdown();
    return 2;
  }
}
