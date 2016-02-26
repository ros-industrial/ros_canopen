cmake_minimum_required(VERSION 2.8.3)
project(joint_limits_controller)

find_package(catkin REQUIRED
  COMPONENTS
    controller_interface
    hardware_interface
    joint_limits_interface
    pluginlib
    urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    limited_joint_handle
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(limited_joint_handle
  src/limited_joint_handle.cpp
  src/limits.cpp
)
target_link_libraries(limited_joint_handle
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    limited_joint_handle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
