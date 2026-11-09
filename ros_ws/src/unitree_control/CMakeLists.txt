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

cmake_minimum_required(VERSION 3.0.2)
project(unitree_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  std_srvs
)
find_package(PkgConfig REQUIRED)
catkin_package()
include_directories(
  include
  /usr/local/include/unitree
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  /usr/local/lib
)
add_executable(unitree_vel_controller 
  src/unitree_vel_controller.cpp
)
add_dependencies(unitree_vel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(unitree_vel_controller
  ${catkin_LIBRARIES}
  ddsc ddscxx
  Threads::Threads
  unitree_sdk2
)


