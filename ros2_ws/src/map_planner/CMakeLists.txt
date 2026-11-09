# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(map_planner)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Planning services (PlanPath / PlanSequence / ValidatePath) are provided by nav_interfaces.
find_package(nav_interfaces REQUIRED)

add_executable(map_planner_node src/map_planner.cpp)
target_include_directories(map_planner_node PRIVATE include)
ament_target_dependencies(map_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_interfaces
)

install(TARGETS map_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# Site maps are deployment data and are not tracked in a fresh checkout.
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
# Scan and point-cloud temporal filtering nodes.
install(PROGRAMS
  scripts/scan_temporal_filter.py
  scripts/odin_cloud_temporal_filter.py
  scripts/odin_slam_temporal_filter.py
  scripts/persist_filter_node_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/low_flicker_filter.py
  scripts/slam_cloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(tf2_msgs REQUIRED)
  add_executable(map_planner_recorded_replay
    test/replay_recorded_plan.cpp src/map_planner.cpp)
  target_compile_definitions(map_planner_recorded_replay PRIVATE MAP_PLANNER_RECORDED_REPLAY)
  target_include_directories(map_planner_recorded_replay PRIVATE include)
  ament_target_dependencies(map_planner_recorded_replay
    rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
    tf2_msgs nav_interfaces)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(map_planner_path_quality_test test/test_path_quality.cpp)
  if(TARGET map_planner_path_quality_test)
    target_include_directories(map_planner_path_quality_test PRIVATE include)
  endif()
  ament_add_gtest(map_planner_path_safety_test test/test_path_safety.cpp)
  if(TARGET map_planner_path_safety_test)
    target_include_directories(map_planner_path_safety_test PRIVATE include)
  endif()
  ament_add_pytest_test(
    odin_cloud_temporal_filter_test
    test/test_odin_cloud_temporal_filter.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    low_flicker_filter_test
    test/test_low_flicker_filter.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    odin_cloud_filter_integration_test
    test/test_odin_cloud_filter_integration.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    slam_cloud_filter_test
    test/test_slam_cloud_filter.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    odin_slam_temporal_filter_test
    test/test_odin_slam_temporal_filter.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
