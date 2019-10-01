cmake_minimum_required(VERSION 3.5)
project(socketcan_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(socketcan_interface REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
  system
  thread
)

find_package(Boost REQUIRED
  COMPONENTS
  system
  thread
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)

# socketcan_to_topic
add_library(socketcan_to_topic SHARED
  src/socketcan_to_topic.cpp
)
ament_target_dependencies(socketcan_to_topic
  "can_msgs"
  "socketcan_interface"
)
target_link_libraries(socketcan_to_topic
  socketcan_interface::socketcan_interface_plugin
  socketcan_interface::socketcan_interface_string
)

# topic_to_socketcan
add_library(topic_to_socketcan SHARED
  src/topic_to_socketcan.cpp
)
ament_target_dependencies(topic_to_socketcan
  "can_msgs"
  "socketcan_interface"
)
target_link_libraries(topic_to_socketcan
  socketcan_interface::socketcan_interface_plugin
  socketcan_interface::socketcan_interface_string
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
ament_target_dependencies(socketcan_to_topic_node
  "rclcpp"
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  ${Boost_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp
)
ament_target_dependencies(topic_to_socketcan_node
  "can_msgs"
  "rclcpp"
)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  ${Boost_LIBRARIES}
)

# socketcan_bridge_node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  socketcan_to_topic
  topic_to_socketcan
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_node
  "can_msgs"
  "rclcpp"
)

ament_export_interfaces(export_socketcan_to_topic HAS_LIBRARY_TARGET)
ament_export_interfaces(export_topic_to_socketcan HAS_LIBRARY_TARGET)

install(
  TARGETS
    ${PROJECT_NAME}_node
    socketcan_to_topic_node
    topic_to_socketcan_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS socketcan_to_topic
  EXPORT export_socketcan_to_topic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS topic_to_socketcan
  EXPORT export_topic_to_socketcan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # unit test for the can_msgs::Frame and can::Frame types.
  ament_add_gtest(test_conversion
    test/test_conversion.cpp
  )
  target_link_libraries(test_conversion
    topic_to_socketcan
    socketcan_to_topic
    ${Boost_LIBRARIES}
  )

#  add_rostest_gtest(test_to_socketcan
#    test/to_socketcan.test
#    test/to_socketcan_test.cpp
#  )
#  target_link_libraries(test_to_socketcan
#    topic_to_socketcan
#    ${catkin_LIBRARIES}
#  )
#
#  add_rostest_gtest(test_to_topic
#    test/to_topic.test
#    test/to_topic_test.cpp
#  )
#  target_link_libraries(test_to_topic
#    socketcan_to_topic
#    topic_to_socketcan
#    ${catkin_LIBRARIES}
#  )
endif()

ament_package()
