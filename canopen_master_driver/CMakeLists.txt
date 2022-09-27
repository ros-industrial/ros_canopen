cmake_minimum_required(VERSION 3.8)
project(canopen_master_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)




add_library(lifecycle_master_driver
  src/lifecycle_master_driver.cpp
  src/node_interfaces/node_canopen_basic_master.cpp
  )
target_compile_features(lifecycle_master_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_master_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_master_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lifecycle_master_driver
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_core
  canopen_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_master_driver PRIVATE "CANOPEN_MASTER_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_master_driver "ros2_canopen::LifecycleMasterDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleMasterDriver;$<TARGET_FILE:lifecycle_master_driver >\n")




add_library(master_driver 
  src/master_driver.cpp
  src/node_interfaces/node_canopen_basic_master.cpp
  )
target_compile_features(master_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(master_driver PUBLIC -Wl,--no-undefined)
target_include_directories(master_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  master_driver
  rclcpp
  canopen_core
  canopen_interfaces
  rclcpp_components
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
  TARGETS lifecycle_master_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS master_driver
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
  canopen_master_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
