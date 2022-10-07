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
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
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
  rclcpp_components
)


add_library(node_canopen_driver
  SHARED
  src/node_interfaces/node_canopen_driver.cpp
  src/driver_error.cpp
  src/driver_node.cpp
)
target_compile_features(node_canopen_driver  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_driver PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(node_canopen_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(node_canopen_driver 
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
)

add_library(node_canopen_master
  SHARED
  src/node_interfaces/node_canopen_master.cpp
  src/master_error.cpp
  src/master_node.cpp
)
target_compile_features(node_canopen_master  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_master PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(node_canopen_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(node_canopen_master 
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
)


add_library(device_container
  SHARED
  src/device_container.cpp
  src/configuration_manager.cpp
  src/device_container_error.cpp
  src/lifecycle_manager.cpp
)
target_compile_features(device_container  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(device_container PUBLIC -fPIC -Wl,--no-undefined)
target_include_directories(device_container PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(device_container 
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
  rclcpp_components
)
target_link_libraries(device_container
  node_canopen_master
  node_canopen_driver
)

add_executable(device_container_node
  src/device_container_node.cpp
  )
target_link_libraries(device_container_node
  device_container
  node_canopen_master
  node_canopen_driver
)
ament_target_dependencies(device_container_node
  ${dependencies}
)
target_include_directories(device_container_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(
  TARGETS node_canopen_driver node_canopen_master device_container
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



install(TARGETS device_container_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  node_canopen_driver
  node_canopen_master
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  canopen_interfaces
  lely_core_libraries
  yaml_cpp_vendor
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
)

ament_package()
