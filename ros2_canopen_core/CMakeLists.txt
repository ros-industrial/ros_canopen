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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ros2_canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(pluginlib REQUIRED)


# Add core library
add_library(ros2_canopen_core SHARED
  src/basic_device_driver.cpp
  src/basic_device_node.cpp
  src/proxy_device_node.cpp
)
target_compile_features(ros2_canopen_core PUBLIC c_std_99 cxx_std_17)
target_include_directories(ros2_canopen_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(ros2_canopen_core PRIVATE "ROS2_CANOPEN_CORE_BUILDING_LIBRARY")

ament_target_dependencies(ros2_canopen_core 
  ros2_canopen_interfaces 
  std_msgs 
  std_srvs
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs 
  lely_core_libraries
)

# Add master node
add_executable(canopen_master_node 
  src/canopen_master_node.cpp
  )

target_link_libraries(canopen_master_node
  ros2_canopen_core
)

ament_target_dependencies(canopen_master_node 
  yaml_cpp_vendor 
  ros2_canopen_interfaces 
  lifecycle_msgs 
  pluginlib
)



target_include_directories(canopen_master_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


add_executable(test_slave
  src/test_slave.cpp
)

ament_target_dependencies(test_slave 
  lely_core_libraries
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs 
)

target_include_directories(test_slave PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

add_executable(test_loop
  src/test_loop.cpp
)

ament_target_dependencies(test_loop 
  lely_core_libraries
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs 
)

target_include_directories(test_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS canopen_master_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_slave
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_loop
  DESTINATION lib/${PROJECT_NAME})

install(
    TARGETS ros2_canopen_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

install(DIRECTORY
  launch
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
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ros2_canopen_core
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
