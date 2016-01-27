cmake_minimum_required(VERSION 2.8.3)
project(canopen_chain_node)

find_package(catkin REQUIRED
  COMPONENTS
    canopen_master
    diagnostic_updater
    pluginlib
    roscpp
    roslib
    socketcan_interface
    std_msgs
    std_srvs
)

find_package(Boost REQUIRED
  COMPONENTS
    filesystem
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    canopen_ros_chain
  CATKIN_DEPENDS
    canopen_master
    diagnostic_updater
    pluginlib
    roscpp
    socketcan_interface
    std_srvs
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# canopen_ros_chain
add_library(canopen_ros_chain
  src/ros_chain.cpp
)
target_link_libraries(canopen_ros_chain
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(canopen_ros_chain
  ${catkin_EXPORTED_TARGETS}
)

# canopen_chain_node
add_executable(canopen_chain_node
  src/chain_node.cpp
)
target_link_libraries(canopen_chain_node
  canopen_ros_chain
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    canopen_chain_node
    canopen_ros_chain
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

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
