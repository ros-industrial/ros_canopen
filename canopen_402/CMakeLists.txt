cmake_minimum_required(VERSION 3.8)
project(canopen_402)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros2_canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)
find_package(ros2_canopen_core REQUIRED)
find_package(pluginlib REQUIRED)

add_library(canopen_402 
  src/mc_device_driver.cpp
  src/mc_device_node.cpp
  src/mc_device.cpp
)

target_compile_features(canopen_402 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(canopen_402 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(canopen_402 PRIVATE "CANOPEN_402_BUILDING_LIBRARY")

ament_target_dependencies(canopen_402 
  ros2_canopen_interfaces 
  std_msgs 
  std_srvs
  boost_container
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs
  lely_core_libraries
  ros2_canopen_core
  pluginlib
)

pluginlib_export_plugin_description_file(ros2_canopen_core canopen_402.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS canopen_402
  DESTINATION lib
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_402
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  std_msgs 
  std_srvs
  lifecycle_msgs
  ros2_canopen_core
  ros2_canopen_interfaces
  lely_core_libraries
  boost_container
  pluginlib
)


ament_package()
