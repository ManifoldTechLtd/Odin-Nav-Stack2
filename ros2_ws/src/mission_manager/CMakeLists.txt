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
project(mission_manager)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(atomic_move REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
  src/navigation_path_safety.cpp
  src/initial_route_planning.cpp
)
target_include_directories(mission_manager_node PRIVATE include)
ament_target_dependencies(mission_manager_node
  rclcpp
  rclcpp_action
  atomic_move
  nav_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(replan_install_gate_test test/replan_install_gate_test.cpp)
  target_include_directories(replan_install_gate_test PRIVATE include)
  enable_testing()
  add_test(NAME replan_install_gate_test COMMAND replan_install_gate_test)
  ament_add_pytest_test(navigation_path_safety
    test/test_navigation_path_safety.py
    TIMEOUT 180
    WERROR
    ENV
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      ROS_DOMAIN_ID=184
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_LOCALHOST_ONLY=1
  )
endif()

ament_package()
