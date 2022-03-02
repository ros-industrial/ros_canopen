cmake_minimum_required(VERSION 3.8)
project(base_device_driver)

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

add_library(base_device_driver
  SHARED
  src/base_device_driver.cpp
  src/lely_bridge.cpp
)
target_compile_features(base_device_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(base_device_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  base_device_driver
  "rclcpp"
  "rclcpp_components"
  "ros2_canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "ros2_canopen_core"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(base_device_driver PRIVATE "BASE_DEVICE_DRIVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS base_device_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  base_device_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
