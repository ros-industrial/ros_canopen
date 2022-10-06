cmake_minimum_required(VERSION 3.8)
project(canopen_proxy_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_core
  canopen_interfaces
  canopen_base_driver
  lely_core_libraries
  std_msgs
  std_srvs
)




add_library(node_canopen_proxy_driver
  src/node_interfaces/node_canopen_proxy_driver.cpp
)
target_compile_features(node_canopen_proxy_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_proxy_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  node_canopen_proxy_driver
  ${dependencies}
)

add_library(lifecycle_proxy_driver
  src/lifecycle_proxy_driver.cpp
  )
target_compile_features(lifecycle_proxy_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_proxy_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_proxy_driver
  node_canopen_proxy_driver
)
ament_target_dependencies(
  lifecycle_proxy_driver
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_proxy_driver PRIVATE "CANOPEN_PROXY_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_proxy_driver "ros2_canopen::LifecycleProxyDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleProxyDriver;$<TARGET_FILE:lifecycle_proxy_driver >\n")




add_library(proxy_driver 
  src/proxy_driver.cpp
  )
target_compile_features(proxy_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(proxy_driver PUBLIC -Wl,--no-undefined)
target_include_directories(proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(proxy_driver
  node_canopen_proxy_driver
)

ament_target_dependencies(
  proxy_driver
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(proxy_driver PRIVATE "CANOPEN_proxy_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(proxy_driver "ros2_canopen::ProxyDriver")
set(node_plugins "${node_plugins}ros2_canopen::ProxyDriver;$<TARGET_FILE:proxy_driver >\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lifecycle_proxy_driver proxy_driver node_canopen_proxy_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  lifecycle_proxy_driver
  proxy_driver
  node_canopen_proxy_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
