cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface)
find_package(pluginlib)
find_package(ros2_control_test_assets)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(std_srvs REQUIRED)


add_library(
  canopen_ros2_control
  SHARED
  src/canopen_system.cpp
)
target_compile_features(canopen_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(canopen_ros2_control PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(
  canopen_ros2_control
  PUBLIC
  include
)
ament_target_dependencies(
  canopen_ros2_control
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  canopen_core
  rclcpp_components
  canopen_proxy_driver
  std_srvs
)
# prevent pluginlib from using boost
target_compile_definitions(canopen_ros2_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface canopen_ros2_control.xml)

install(
  TARGETS
  canopen_ros2_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
target_include_directories(test_canopen_system PRIVATE include)
ament_target_dependencies(
  test_canopen_system
  hardware_interface
  pluginlib
  ros2_control_test_assets
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_ros2_control
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  canopen_core
  canopen_proxy_driver
  std_srvs
)

ament_package()
