cmake_minimum_required(VERSION 3.5)
project(canopen_chain_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(canopen_master)

find_package(diagnostic_updater)
# find_package(rosidl_default_generators)
find_package(pluginlib)
find_package(rclcpp)
find_package(socketcan_interface)
find_package(std_msgs)
find_package(std_srvs)

find_package(Boost REQUIRED
  COMPONENTS
    filesystem
    chrono
    thread
)

# set(srv_files
#   "srv/GetObject.srv"
#   "srv/SetObject.srv"
# )
#
# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   ${srv_files}
#   ADD_LINTER_TESTS
# )

include_directories(include
  ${socketcan_interface_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# link_directories(
#   ${socketcan_interface_LIBRARY_DIRS}
#   ${Boost_LIBRARY_DIRS}
# )

# canopen_ros_chain
add_library(canopen_ros_chain
  src/ros_chain.cpp
  src/rosconsole_bridge.cpp
)
target_link_libraries(canopen_ros_chain
  ${Boost_LIBRARIES}
)

ament_target_dependencies(canopen_ros_chain
  canopen_master
  std_srvs
  std_msgs
  # ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# canopen_chain_node
add_executable(${PROJECT_NAME}
  src/chain_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  # canopen_ros_chain
  ${Boost_LIBRARIES}
)

# canopen_sync_node
add_executable(canopen_sync_node
  src/rosconsole_bridge.cpp
  src/sync_node.cpp
)
target_link_libraries(canopen_sync_node
  ${Boost_LIBRARIES}
)

ament_target_dependencies(canopen_sync_node
  canopen_master
  diagnostic_updater
  socketcan_interface
  rclcpp
)

ament_export_dependencies(
  rosidl_default_runtime
  ament_cmake
  Boost
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS
    ${PROJECT_NAME}
    canopen_ros_chain
    canopen_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
