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
project(nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/MissionStatus.msg"
  "msg/MissionEvent.msg"
  "msg/OperatorCommand.msg"
  "msg/NodeStatus.msg"
  "msg/SystemHealth.msg"
  "msg/FollowTarget.msg"
  "msg/RecoveryRequest.msg"
  "msg/RecoveryCommand.msg"
  "srv/StartMission.srv"
  "srv/RestartNode.srv"
  "srv/FollowSearch.srv"
  "srv/PlanPath.srv"
  "srv/ValidatePath.srv"
  "srv/PlanSequence.srv"
  "action/Nav.action"
  "action/NavPath.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
