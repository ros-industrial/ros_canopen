cmake_minimum_required(VERSION 3.5)
project(ros2_canopen)

include(FindPkgConfig)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra)# -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ros2_canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_control REQUIRED)

# Liblely is automake project, needs to be imported using pkg_check
pkg_check_modules(LIBLELY-IO2 REQUIRED liblely-io2)
pkg_check_modules(LIBLELY-COAPP REQUIRED liblely-coapp)

add_executable(canopen_master_node 
  src/canopen_master_node.cpp
  src/basic_device_driver.cpp
  src/basic_device_node.cpp
  src/proxy_device_node.cpp
  )

ament_target_dependencies(canopen_master_node
  LIBLELY-IO2 
  LIBLELY-COAPP 
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs 
  yaml_cpp_vendor 
  ros2_canopen_interfaces 
  std_msgs 
  std_srvs
  ros2_control
)
target_include_directories(canopen_master_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


add_executable(test_slave
  src/test_slave.cpp
)

ament_target_dependencies(test_slave 
  LIBLELY-IO2 
  LIBLELY-COAPP 
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs 
)

target_include_directories(test_slave PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


install(TARGETS canopen_master_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_slave
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
