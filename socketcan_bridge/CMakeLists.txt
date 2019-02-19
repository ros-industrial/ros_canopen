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

include_directories(include
  ${socketcan_interface_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${can_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${socketcan_interface_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS}
  ${can_msgs_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# socketcan_to_topic
add_library(socketcan_to_topic SHARED
  src/socketcan_to_topic.cpp
)
target_link_libraries(socketcan_to_topic
  ${socketcan_interface_LIBRARIES}
  ${can_msgs_LIBRARIES}
)

# topic_to_socketcan
add_library(topic_to_socketcan SHARED
  src/topic_to_socketcan.cpp
)
target_link_libraries(topic_to_socketcan
  ${socketcan_interface_LIBRARIES}
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp
)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  ${rclcpp_LIBRARIES}
  ${can_msgs_LIBRARIES}
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
  socketcan_interface
  rclcpp
  can_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    socketcan_to_topic
    socketcan_to_topic_node
    topic_to_socketcan
    topic_to_socketcan_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include/
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
