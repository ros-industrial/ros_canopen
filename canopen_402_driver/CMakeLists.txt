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
find_package(canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

add_library(${PROJECT_NAME} 
  src/canopen_402_driver.cpp
  src/motor.cpp
  )

target_compile_features(${PROJECT_NAME}  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME} 
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "canopen_core"
  "canopen_proxy_driver"
  "canopen_base_driver"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}  PRIVATE "CANOPEN_402_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(${PROJECT_NAME}  "ros2_canopen::MotionControllerDriver")
set(node_plugins "${node_plugins}ros2_canopen::MotionControllerDriver;$<TARGET_FILE:${PROJECT_NAME} >\n")


dcfgen(${CMAKE_CURRENT_SOURCE_DIR}/config/ simple_mc.yml ${CMAKE_BINARY_DIR}/config/)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS ${PROJECT_NAME} 
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
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME} 
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
