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
project(sw01_ros_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Locate ASTRALL SDK (bundled under sdk/ by architecture, overridable via -DSW01_SDK_ROOT=<dir>)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(SW01_SDK_ARCH "ARM64")
else()
  set(SW01_SDK_ARCH "x86_64")
endif()

if(DEFINED SW01_SDK_ROOT AND NOT SW01_SDK_ROOT STREQUAL "")
  set(ASTRALL_SDK_INCLUDE_DIR "${SW01_SDK_ROOT}/include")
  set(ASTRALL_SDK_LIBRARY_DIR "${SW01_SDK_ROOT}/lib")
else()
  set(ASTRALL_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk/include")
  set(ASTRALL_SDK_LIBRARY_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/linux/${SW01_SDK_ARCH}")
endif()

find_path(ASTRALL_SDK_INCLUDE_HDR interface.h PATHS "${ASTRALL_SDK_INCLUDE_DIR}" NO_DEFAULT_PATH)
find_library(ASTRALL_SDK_LIBRARY NAMES ASTRALL_SDK PATHS "${ASTRALL_SDK_LIBRARY_DIR}" NO_DEFAULT_PATH)

if(NOT ASTRALL_SDK_INCLUDE_HDR OR NOT ASTRALL_SDK_LIBRARY)
  message(FATAL_ERROR
    "\nASTRALL SDK not found.\n"
    "  Header: ${ASTRALL_SDK_INCLUDE_DIR}/interface.h\n"
    "  Library dir: ${ASTRALL_SDK_LIBRARY_DIR}/libASTRALL_SDK.so\n"
    "Ensure sdk/ is present or pass -DSW01_SDK_ROOT=<SDK C++ dir>.\n"
    "See sw01_ros_control/sdk/README_SDK_SETUP.md for details.\n")
endif()
message(STATUS "ASTRALL SDK include : ${ASTRALL_SDK_INCLUDE_HDR}")
message(STATUS "ASTRALL SDK library : ${ASTRALL_SDK_LIBRARY}")

add_executable(sw01_cmd_vel_bridge src/sw01_cmd_vel_bridge.cpp)
target_include_directories(sw01_cmd_vel_bridge PRIVATE
  include
  ${ASTRALL_SDK_INCLUDE_DIR}
)
ament_target_dependencies(sw01_cmd_vel_bridge
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(sw01_cmd_vel_bridge
  ${ASTRALL_SDK_LIBRARY}
  pthread
)
target_link_options(sw01_cmd_vel_bridge PRIVATE
  "-Wl,-rpath,${ASTRALL_SDK_LIBRARY_DIR}"
  "-Wl,-rpath,\$ORIGIN"
)

install(TARGETS sw01_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
file(GLOB ASTRALL_SDK_RUNTIME_LIBS "${ASTRALL_SDK_LIBRARY_DIR}/libASTRALL_SDK.so*")
install(FILES ${ASTRALL_SDK_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
