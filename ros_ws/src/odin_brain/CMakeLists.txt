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
project(odin_brain)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_interfaces
  std_msgs
  std_srvs
  tf2_ros
)

catkin_python_setup()

# catkin_package() must precede catkin_install_python() so CATKIN_PACKAGE_BIN_DESTINATION is defined.
catkin_package()

catkin_install_python(PROGRAMS
  scripts/brain
  scripts/brain_node
  scripts/brain_places
  scripts/check_llm
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
