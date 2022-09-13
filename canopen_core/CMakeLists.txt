cmake_minimum_required(VERSION 3.5)
project(canopen_core)

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
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(lely_core_libraries REQUIRED)
find_package(canopen_interfaces REQUIRED)

set (dependencies 
  lely_core_libraries
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_interfaces
)

add_library(configuration_manager
  SHARED
  src/configuration_manager.cpp
)
target_compile_features(configuration_manager  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(configuration_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(configuration_manager 
  rclcpp
  yaml_cpp_vendor
)

add_library(master_node
  SHARED
  src/master_node.cpp
  src/lely_master_bridge.cpp
  )

target_compile_features(master_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(master_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  master_node
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "lely_core_libraries"
  "yaml_cpp_vendor"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
)
rclcpp_components_register_nodes(master_node  "ros2_canopen::MasterNode")
set(node_plugins "${node_plugins}ros2_canopen::MasterNode;$<TARGET_FILE:master_node >\n")

add_library(lifecycle_master_node
  SHARED
  src/lifecycle_master_node.cpp
  src/lely_master_bridge.cpp
  )

target_compile_features(lifecycle_master_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(lifecycle_master_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  lifecycle_master_node
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "lely_core_libraries"
  "yaml_cpp_vendor"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
)
rclcpp_components_register_nodes(lifecycle_master_node  "ros2_canopen::LifecycleMasterNode")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleMasterNode;$<TARGET_FILE:lifecycle_master_node >\n")


# device manager node
add_library(lifecycle_device_manager_node
  SHARED
  src/lifecycle_device_manager_node.cpp
  )

target_compile_features(lifecycle_device_manager_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(lifecycle_device_manager_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_device_manager_node
  configuration_manager)

ament_target_dependencies(
  lifecycle_device_manager_node
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "std_srvs"
)
rclcpp_components_register_nodes(lifecycle_device_manager_node  "ros2_canopen::LifecycleDeviceManagerNode")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleDeviceManagerNode;$<TARGET_FILE:lifecycle_device_manager_node >\n")

# device container node
add_executable(lifecycle_device_container_node 
  src/lifecycle_device_container_node.cpp
  )
target_link_libraries(lifecycle_device_container_node
  configuration_manager
  lifecycle_device_manager_node
  lifecycle_master_node
)
ament_target_dependencies(lifecycle_device_container_node
  ${dependencies}
)
target_include_directories(lifecycle_device_container_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

# device container (manager)
add_library(device_container src/device_container_node.cpp src/lely_master_bridge.cpp)
target_compile_features(device_container PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(device_container PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(device_container PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(device_container
  configuration_manager
)
ament_target_dependencies(device_container
  ${dependencies}
)
target_include_directories(device_container PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

# device container node
add_executable(device_container_node
  src/device_container_executable_node.cpp
  )
target_link_libraries(device_container_node device_container)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS configuration_manager
  EXPORT export_configuration_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS master_node
  EXPORT export_master_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lifecycle_master_node
  EXPORT export_lifecycle_master_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lifecycle_device_manager_node
  EXPORT export_lifecycle_device_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS device_container
  EXPORT export_device_container
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lifecycle_device_container_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS device_container_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
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

ament_export_libraries(
  configuration_manager
  master_node
  lifecycle_master_node
  lifecycle_device_manager_node
  device_container
)
ament_export_targets(
  export_configuration_manager
  export_master_node
  export_lifecycle_master_node
  export_lifecycle_device_manager_node
  export_device_container
)

ament_export_dependencies(
  canopen_interfaces
  lely_core_libraries
  yaml_cpp_vendor
  rclcpp_lifecycle
  lifecycle_msgs
)

ament_package()
