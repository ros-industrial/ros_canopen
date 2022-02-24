cmake_minimum_required(VERSION 3.5)
project(ros2_canopen_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra)# -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ros2_canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)



set (dependencies 
  lely_core_libraries
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
)



# device manager node
add_executable(device_manager_node 
  src/device_manager.cpp)

ament_target_dependencies(device_manager_node 
  ${dependencies}
)

target_include_directories(device_manager_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

# slave node
add_executable(slave_node 
  src/slave.cpp)

ament_target_dependencies(slave_node 
  ${dependencies}
)

target_include_directories(slave_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS device_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS slave_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
  #find_package(launch_testing_ament_cmake)
  #add_launch_test(tests/test_device_up.py)
endif()

ament_export_include_directories(
  include
)

ament_package()
