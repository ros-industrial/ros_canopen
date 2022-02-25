cmake_minimum_required(VERSION 3.8)
project(proxy_device_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(ros2_canopen_core REQUIRED)
find_package(base_device_driver REQUIRED)

set(node_plugins "")

add_library(proxy_device_driver SHARED
  src/proxy_device_driver.cpp
  )
target_compile_features(proxy_device_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(proxy_device_driver 
  PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(proxy_device_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  proxy_device_driver
  rclcpp
  rclcpp_components
  ros2_canopen_interfaces
  std_msgs
  std_srvs
  lely_core_libraries
  ros2_canopen_core
  base_device_driver
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(proxy_device_driver PRIVATE "PROXY_DEVICE_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(proxy_device_driver "ros2_canopen::ProxyDeviceDriver")
set(node_plugins "${node_plugins}ros2_canopen::ProxyDeviceDriver;$<TARGET_FILE:proxy_device_driver>\n")


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS proxy_device_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  proxy_device_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
