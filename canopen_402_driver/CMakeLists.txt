cmake_minimum_required(VERSION 3.8)
project(canopen_402_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs)

add_library(${PROJECT_NAME} 
  src/canopen_402_driver.cpp
  src/motor.cpp
  )

target_compile_features(canopen_402_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(canopen_402_driver PUBLIC -Wno-unused-parameter)
target_include_directories(canopen_402_driver  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  canopen_402_driver
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "canopen_core"
  "canopen_proxy_driver"
  "canopen_base_driver"
  "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(canopen_402_driver  PRIVATE "CANOPEN_402_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(canopen_402_driver  "ros2_canopen::MotionControllerDriver")
set(node_plugins "${node_plugins}ros2_canopen::MotionControllerDriver;$<TARGET_FILE:canopen_402_driver >\n")


add_library(lifecycle_canopen_402_driver 
  src/lifecycle_canopen_402_driver.cpp
  src/motor.cpp
  )

target_compile_features(lifecycle_canopen_402_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(lifecycle_canopen_402_driver PUBLIC -Wno-unused-parameter)
target_include_directories(lifecycle_canopen_402_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  lifecycle_canopen_402_driver
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "canopen_core"
  "canopen_proxy_driver"
  "canopen_base_driver"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_canopen_402_driver PRIVATE "CANOPEN_402_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(lifecycle_canopen_402_driver "ros2_canopen::LifecycleMotionControllerDriver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleMotionControllerDriver;$<TARGET_FILE:canopen_402_driver >\n")


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS canopen_402_driver
  EXPORT export_canopen_402_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lifecycle_canopen_402_driver
  EXPORT export_lifecycle_canopen_402_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_402_driver
  lifecycle_canopen_402_driver
)
ament_export_targets(
  export_canopen_402_driver
  export_lifecycle_canopen_402_driver
)

ament_package()
