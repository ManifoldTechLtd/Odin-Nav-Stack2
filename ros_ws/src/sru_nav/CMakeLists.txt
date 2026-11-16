cmake_minimum_required(VERSION 3.0.2)
project(sru_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
)

# Installs the `sru` Python package via the root-level `sru` symlink to the
# single shared source tree at <repo>/sru/src — see setup.py).
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sru_nav_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)