cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib)
find_package(ros2_control_test_assets)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(std_srvs REQUIRED)

add_library(
  canopen_proxy_controller
  SHARED
  src/canopen_proxy_controller.cpp
)
target_include_directories(
  canopen_proxy_controller
  PRIVATE
  include
)
ament_target_dependencies(
  canopen_proxy_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

pluginlib_export_plugin_description_file(
  controller_interface canopen_ros2_controllers.xml)

install(
  TARGETS
  canopen_proxy_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


  ament_add_gmock(test_load_canopen_proxy_controller test/test_load_canopen_proxy_controller.cpp)
  target_include_directories(test_load_canopen_proxy_controller PRIVATE include)
  ament_target_dependencies(
    test_load_canopen_proxy_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_canopen_proxy_controller test/test_canopen_proxy_controller.cpp)
  target_include_directories(test_canopen_proxy_controller PRIVATE include)
  target_link_libraries(test_canopen_proxy_controller canopen_proxy_controller)
  ament_target_dependencies(
    test_canopen_proxy_controller
    controller_interface
    hardware_interface
  )

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_proxy_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)
