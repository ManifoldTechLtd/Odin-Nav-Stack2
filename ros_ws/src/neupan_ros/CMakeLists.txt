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
project(neupan_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_interfaces
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs nav_interfaces nav_msgs rospy sensor_msgs std_msgs tf visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/neupan_upstream_action_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
