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
  src/joint_limiter.cpp
  src/limits.cpp
)
target_link_libraries(limited_joint_handle
  ${catkin_LIBRARIES}
)

add_library(joint_limits_controller
  src/joint_limits_controller.cpp
)

target_link_libraries(joint_limits_controller
  limited_joint_handle
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    joint_limits_controller
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

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_limits tests/test_limits.cpp)
  target_link_libraries(test_limits
    limited_joint_handle
    ${catkin_LIBRARIES}
  )

  find_package(rostest REQUIRED)
  add_rostest_gtest(test_urdf_yaml
    tests/test_urdf_yaml.test
    tests/test_urdf_yaml.cpp
  )
  target_link_libraries(test_urdf_yaml
    limited_joint_handle
    ${catkin_LIBRARIES}
  )
endif()
