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

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    canopen_motor
  CATKIN_DEPENDS
    canopen_402
    canopen_chain_node
    controller_manager
    hardware_interface
    joint_limits_interface
    urdf
  DEPENDS
    Boost
    MUPARSER
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MUPARSER_INCLUDE_DIRS}
)

# canopen_motor
add_library(canopen_motor
  src/controller_manager_layer.cpp
  src/handle_layer.cpp
  src/motor_chain.cpp
  src/robot_layer.cpp
)
target_link_libraries(canopen_motor
  ${catkin_LIBRARIES}
  ${MUPARSER_LIBRARIES}
)
add_dependencies(canopen_motor
  ${catkin_EXPORTED_TARGETS}
)

# canopen_motor_node
add_executable(canopen_motor_node
  src/canopen_motor_chain_node.cpp
)
target_link_libraries(canopen_motor_node
  canopen_motor
  ${catkin_LIBRARIES}
)
add_dependencies(canopen_motor_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    canopen_motor
    canopen_motor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test_muparser
    test/test_muparser.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_muparser
    canopen_motor
    ${catkin_LIBRARIES}
    ${MUPARSER_LIBRARIES}
  )
endif()
