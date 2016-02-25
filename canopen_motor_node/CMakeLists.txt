cmake_minimum_required(VERSION 2.8.3)
project(canopen_motor_node)

find_package(catkin REQUIRED
  COMPONENTS
    canopen_402
    canopen_chain_node
    controller_manager
    controller_manager_msgs
    filters
    hardware_interface
    joint_limits_interface
    joint_limits_interface
    urdf
)

find_package(Boost REQUIRED
  COMPONENTS
    thread
)

find_package(PkgConfig)
pkg_check_modules (MUPARSER REQUIRED muparser)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MUPARSER_INCLUDE_DIRS}
)

# canopen_motor_node
add_executable(canopen_motor_node
  src/control_node.cpp
  src/robot_layer.cpp
)
target_link_libraries(canopen_motor_node
  ${catkin_LIBRARIES}
  ${MUPARSER_LIBRARIES}
)
add_dependencies(canopen_motor_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    canopen_motor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
