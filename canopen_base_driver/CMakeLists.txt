cmake_minimum_required(VERSION 3.8)
project(canopen_base_driver)

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
  lely_core_libraries
  std_msgs
  std_srvs
)


add_library(lely_driver_bridge
  src/lely_driver_bridge.cpp
)
target_compile_features(lely_driver_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lely_driver_bridge PUBLIC -Wl,--no-undefined)
target_include_directories(lely_driver_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lely_driver_bridge
  ${dependencies}
)


add_library(node_canopen_base_driver
  src/node_interfaces/node_canopen_base_driver.cpp
)
target_compile_features(node_canopen_base_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_base_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_base_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(node_canopen_base_driver
  lely_driver_bridge
)

ament_target_dependencies(
  node_canopen_base_driver
  ${dependencies}
)

add_library(lifecycle_base_driver
  src/lifecycle_base_driver.cpp
  )
target_compile_features(lifecycle_base_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_base_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_base_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_base_driver
  node_canopen_base_driver
  lely_driver_bridge
)
ament_target_dependencies(
  lifecycle_base_driver
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_base_driver PRIVATE "CANOPEN_base_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_base_driver "ros2_canopen::LifecycleBaseDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleBaseDriver;$<TARGET_FILE:lifecycle_base_driver >\n")




add_library(base_driver 
  src/base_driver.cpp
  )
target_compile_features(base_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(base_driver PUBLIC -Wl,--no-undefined)
target_include_directories(base_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(base_driver
  node_canopen_base_driver
  lely_driver_bridge
)

ament_target_dependencies(
  base_driver
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(base_driver PRIVATE "CANOPEN_base_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(base_driver "ros2_canopen::BaseDriver")
set(node_plugins "${node_plugins}ros2_canopen::BaseDriver;$<TARGET_FILE:base_driver >\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lifecycle_base_driver base_driver node_canopen_base_driver lely_driver_bridge
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
  lifecycle_base_driver
  base_driver
  node_canopen_base_driver
  lely_driver_bridge
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
