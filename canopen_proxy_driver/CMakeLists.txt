cmake_minimum_required(VERSION 3.8)
project(canopen_proxy_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(node_plugins "")

add_library(lifecycle_canopen_proxy_driver SHARED
  src/lifecycle_canopen_proxy_driver.cpp
)
target_compile_features(lifecycle_canopen_proxy_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_canopen_proxy_driver
  PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(lifecycle_canopen_proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  lifecycle_canopen_proxy_driver
  rclcpp
  rclcpp_components
  canopen_interfaces
  std_msgs
  std_srvs
  lely_core_libraries
  canopen_core
  canopen_base_driver
  rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_canopen_proxy_driver PRIVATE "CANOPEN_PROXY_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_canopen_proxy_driver "ros2_canopen::LifecycleProxyDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleProxyDriver;$<TARGET_FILE:lifecycle_canopen_proxy_driver>\n")


add_library(canopen_proxy_driver SHARED
  src/canopen_proxy_driver.cpp
)
target_compile_features(canopen_proxy_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(canopen_proxy_driver
  PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(canopen_proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  canopen_proxy_driver
  rclcpp
  rclcpp_components
  canopen_interfaces
  std_msgs
  std_srvs
  lely_core_libraries
  canopen_core
  canopen_base_driver
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(canopen_proxy_driver PRIVATE "CANOPEN_PROXY_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(canopen_proxy_driver "ros2_canopen::ProxyDriver")
set(node_plugins "${node_plugins}ros2_canopen::ProxyDriver;$<TARGET_FILE:canopen_proxy_driver>\n")


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS canopen_proxy_driver
  EXPORT export_canopen_proxy_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lifecycle_canopen_proxy_driver
  EXPORT export_lifecycle_canopen_proxy_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)

endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_proxy_driver
  lifecycle_canopen_proxy_driver
)
ament_export_targets(
  export_canopen_proxy_driver
  export_lifecycle_canopen_proxy_driver
)

ament_export_dependencies(
  canopen_base_driver
)

ament_package()
