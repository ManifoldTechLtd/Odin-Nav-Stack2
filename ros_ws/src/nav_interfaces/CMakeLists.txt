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
project(nav_interfaces)

# Unified navigation stack interface contract package:
#   msg/srv -- mission and health layer contracts (mission_manager, health monitor, frontend bridge)
#   action  -- planning layer contracts (global PlanPath/PlanSequence, local nav/navPath)
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

add_message_files(
  FILES
  Waypoint.msg
  MissionStatus.msg
  MissionEvent.msg
  OperatorCommand.msg
  NodeStatus.msg
  SystemHealth.msg
  FollowTarget.msg
  RecoveryRequest.msg
  RecoveryCommand.msg
)

add_service_files(
  FILES
  StartMission.srv
  RestartNode.srv
  FollowSearch.srv
  PlanPath.srv
  PlanSequence.srv
)

add_action_files(
  FILES
  nav.action
  navPath.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime nav_msgs std_msgs
)
