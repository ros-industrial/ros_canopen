cmake_minimum_required(VERSION 3.0.2)
project(canopen_chain_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    canopen_master
    diagnostic_updater
    message_generation
    pluginlib
    rosconsole_bridge
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

add_service_files(DIRECTORY srv
                  FILES
                  GetObject.srv
                  SetObject.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    canopen_ros_chain
  CATKIN_DEPENDS
    canopen_master
    diagnostic_updater
    message_runtime
    pluginlib
    rosconsole_bridge
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
  src/rosconsole_bridge.cpp
)
target_link_libraries(canopen_ros_chain
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(canopen_ros_chain
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# canopen_chain_node
add_executable(${PROJECT_NAME}
  src/chain_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  canopen_ros_chain
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# canopen_sync_node
add_executable(canopen_sync_node
  src/rosconsole_bridge.cpp
  src/sync_node.cpp
)
target_link_libraries(canopen_sync_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
    canopen_ros_chain
    canopen_sync_node
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
