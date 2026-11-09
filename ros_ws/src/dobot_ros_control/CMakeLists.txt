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
project(dobot_ros_control)

# Lumo SDK RPC headers and static library require C++17 (nlohmann/json)
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  diagnostic_msgs
)

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs diagnostic_msgs
)

# Locate Lumo SDK:
#   Default: <repo>/third_party/dobot/lumo_sdk (can be overridden via -DDOBOT_SDK_ROOT=/path/to/lumo_sdk)
#   If found: enables backend=sdk (official AlgsRpcClient)
#   If missing: builds raw backend only (built-in JSON-RPC)
set(DOBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/dobot/lumo_sdk"
    CACHE PATH "Lumo SDK root (contains rpc/, lib/libclient_rpc.a)")

find_path(DOBOT_RPC_INCLUDE_DIR
  NAMES rpc/algs_rpc_client.h
  PATHS "${DOBOT_SDK_ROOT}"
  NO_DEFAULT_PATH
)

find_library(DOBOT_RPC_LIBRARY
  NAMES client_rpc
  PATHS "${DOBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH
)

# glibc compatibility shim for Ubuntu 20.04 (__libc_single_threaded)
set(DOBOT_LIBC_SHIM
    "${CMAKE_CURRENT_SOURCE_DIR}/src/libc_single_threaded_shim.cpp")

if(DOBOT_RPC_INCLUDE_DIR AND DOBOT_RPC_LIBRARY)
  set(DOBOT_USE_SDK_DEFAULT ON)
  message(STATUS "dobot_ros_control: Found Lumo SDK -> ${DOBOT_SDK_ROOT}")
else()
  set(DOBOT_USE_SDK_DEFAULT OFF)
  message(STATUS "dobot_ros_control: Lumo SDK not found (building raw backend only)")
endif()
option(DOBOT_USE_SDK "Enable official SDK backend (libclient_rpc.a)" ${DOBOT_USE_SDK_DEFAULT})

set(DOBOT_SRCS
  src/dobot_rpc_raw.cpp
  src/dobot_bridge.cpp
  src/dobot_cmd_vel_bridge.cpp
)
set(DOBOT_LINK_LIBS
  ${catkin_LIBRARIES}
  Threads::Threads
)
set(DOBOT_INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS}
)

if(DOBOT_USE_SDK)
  if(NOT EXISTS "${DOBOT_RPC_LIBRARY}")
    message(FATAL_ERROR "DOBOT_USE_SDK=ON but ${DOBOT_SDK_ROOT}/lib/libclient_rpc.a was not found")
  endif()
  list(APPEND DOBOT_SRCS src/dobot_rpc_sdk.cpp)
  list(APPEND DOBOT_INCLUDE_DIRS "${DOBOT_RPC_INCLUDE_DIR}" "${DOBOT_SDK_ROOT}/thirdparty")
  list(APPEND DOBOT_LINK_LIBS "${DOBOT_RPC_LIBRARY}")
  add_definitions(-DDOBOT_USE_SDK=1)

  if(EXISTS "${DOBOT_LIBC_SHIM}")
    list(APPEND DOBOT_SRCS "${DOBOT_LIBC_SHIM}")
    message(STATUS "dobot_ros_control: Linked glibc shim -> ${DOBOT_LIBC_SHIM}")
  else()
    message(WARNING "glibc shim not found (${DOBOT_LIBC_SHIM}); if linking fails on Ubuntu 20.04, "
                    "install g++-11 and provide the shim.")
  endif()
else()
  add_definitions(-DDOBOT_USE_SDK=0)
endif()

add_executable(dobot_cmd_vel_bridge ${DOBOT_SRCS})
target_include_directories(dobot_cmd_vel_bridge PRIVATE ${DOBOT_INCLUDE_DIRS})
target_link_libraries(dobot_cmd_vel_bridge PRIVATE ${DOBOT_LINK_LIBS})

install(TARGETS dobot_cmd_vel_bridge RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Python JSON-RPC velocity bridge
catkin_install_python(PROGRAMS scripts/dobot_vel_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
