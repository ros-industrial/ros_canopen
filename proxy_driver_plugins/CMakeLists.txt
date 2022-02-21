cmake_minimum_required(VERSION 3.5)
project(proxy_driver_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_canopen_core REQUIRED)
find_package(lely_core_libraries REQUIRED)


set(node_plugins "")

add_library(proxy_driver SHARED
  src/proxy_driver.cpp)

target_include_directories(proxy_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(
  proxy_driver
  rclcpp
  rclcpp_components
  ros2_canopen_core
  lely_core_libraries
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(proxy_driver PRIVATE "PROXY_DRIVER_PLUGINS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(proxy_driver "ros2_canopen::ProxyDriverWrapper")
set(node_plugins "${node_plugins}ros2_canopen::ProxyDriverWrapper;$<TARGET_FILE:proxy_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS proxy_driver
  EXPORT export_${PROJECT_NAME}
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  proxy_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
