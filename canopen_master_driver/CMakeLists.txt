cmake_minimum_required(VERSION 3.8)
project(canopen_master_driver)

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

set(dependencies 
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_core
  canopen_interfaces
)

add_library(lely_master_bridge
  src/lely_master_bridge.cpp
)
target_compile_features(lely_master_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lely_master_bridge PUBLIC -Wl,--no-undefined)
target_include_directories(lely_master_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lely_master_bridge
  ${dependencies}
)


add_library(node_canopen_basic_master
  src/node_interfaces/node_canopen_basic_master.cpp
)
target_compile_features(node_canopen_basic_master PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_basic_master PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_basic_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
  node_canopen_basic_master
  lely_master_bridge
)

ament_target_dependencies(
  node_canopen_basic_master
  ${dependencies}
)



add_library(lifecycle_master_driver
  src/lifecycle_master_driver.cpp
  )
target_compile_features(lifecycle_master_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_master_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_master_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  lifecycle_master_driver
  node_canopen_basic_master
  lely_master_bridge
)

ament_target_dependencies(
  lifecycle_master_driver
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_master_driver PRIVATE "CANOPEN_MASTER_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_master_driver "ros2_canopen::LifecycleMasterDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleMasterDriver;$<TARGET_FILE:lifecycle_master_driver >\n")




add_library(master_driver 
  src/master_driver.cpp
  )
target_compile_features(master_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(master_driver PUBLIC -Wl,--no-undefined)
target_include_directories(master_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  master_driver
  node_canopen_basic_master
  lely_master_bridge
)

ament_target_dependencies(
  master_driver
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(master_driver PRIVATE "CANOPEN_MASTER_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(master_driver "ros2_canopen::MasterDriver")
set(node_plugins "${node_plugins}ros2_canopen::MasterDriver;$<TARGET_FILE:master_driver >\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lifecycle_master_driver master_driver node_canopen_basic_master lely_master_bridge
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
  master_driver
  lifecycle_master_driver
  node_canopen_basic_master
  lely_master_bridge
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
