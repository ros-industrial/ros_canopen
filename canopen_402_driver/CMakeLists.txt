cmake_minimum_required(VERSION 3.8)
project(canopen_402_driver)

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
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_core
  canopen_interfaces
  canopen_base_driver
  canopen_proxy_driver
  lely_core_libraries
  std_msgs
  std_srvs
  sensor_msgs
)

add_library(lely_motion_controller_bridge
  src/motor.cpp
  src/lely_motion_controller_bridge.cpp
)
target_compile_features(lely_motion_controller_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lely_motion_controller_bridge PUBLIC -Wl,--no-undefined)
target_include_directories(lely_motion_controller_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lely_motion_controller_bridge
  ${dependencies}
)


add_library(node_canopen_cia402_driver
  src/node_interfaces/node_canopen_402_driver.cpp
)
target_compile_features(node_canopen_cia402_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_cia402_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_cia402_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_cia402_driver
  lely_motion_controller_bridge
)
ament_target_dependencies(
  node_canopen_cia402_driver
  ${dependencies}
  
)

add_library(lifecycle_cia402_driver
  src/lifecycle_cia402_driver.cpp
  )
target_compile_features(lifecycle_cia402_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_cia402_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_cia402_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_cia402_driver
  node_canopen_cia402_driver
  lely_motion_controller_bridge
)
ament_target_dependencies(
  lifecycle_cia402_driver
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_cia402_driver PRIVATE "CANOPEN_402_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_cia402_driver "ros2_canopen::LifecycleCia402Driver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleCia402Driver;$<TARGET_FILE:lifecycle_cia402_driver>\n")




add_library(cia402_driver 
  src/cia402_driver.cpp
  )
target_compile_features(cia402_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(cia402_driver PUBLIC -Wl,--no-undefined)
target_include_directories(cia402_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cia402_driver
  node_canopen_cia402_driver
  lely_motion_controller_bridge
)

ament_target_dependencies(
  cia402_driver
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cia402_driver PRIVATE "CANOPEN_cia402_driver_BUILDING_LIBRARY")

rclcpp_components_register_nodes(cia402_driver "ros2_canopen::Cia402Driver")
set(node_plugins "${node_plugins}ros2_canopen::Cia402Driver;$<TARGET_FILE:cia402_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lifecycle_cia402_driver cia402_driver node_canopen_cia402_driver lely_motion_controller_bridge
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
  lifecycle_cia402_driver
  cia402_driver
  node_canopen_cia402_driver
  lely_motion_controller_bridge
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
