cmake_minimum_required(VERSION 2.8.3)
project(socketcan_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    rosconsole_bridge
    roscpp
    socketcan_interface
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
  CATKIN_DEPENDS
    can_msgs
    rosconsole_bridge
    roscpp
    socketcan_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# socketcan_to_topic
add_library(socketcan_to_topic
  src/rosconsole_bridge.cpp
  src/socketcan_to_topic.cpp
)
target_link_libraries(socketcan_to_topic
  ${catkin_LIBRARIES}
)
add_dependencies(socketcan_to_topic
  ${catkin_EXPORTED_TARGETS}
)

# topic_to_socketcan
add_library(topic_to_socketcan
  src/rosconsole_bridge.cpp
  src/topic_to_socketcan.cpp
)
target_link_libraries(topic_to_socketcan
  ${catkin_LIBRARIES}
)
add_dependencies(topic_to_socketcan
  ${catkin_EXPORTED_TARGETS}
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp

)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  ${catkin_LIBRARIES}
)

# socketcan_bridge_node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  topic_to_socketcan
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    socketcan_to_topic
    socketcan_to_topic_node
    topic_to_socketcan
    topic_to_socketcan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)

  roslint_cpp()
  roslint_add_test()

  # unit test for the can_msgs::Frame and can::Frame types.
  catkin_add_gtest(test_conversion
    test/test_conversion.cpp
  )
  target_link_libraries(test_conversion
    topic_to_socketcan
    socketcan_to_topic
    ${catkin_LIBRARIES}
  )


  add_rostest_gtest(test_to_socketcan
    test/to_socketcan.test
    test/to_socketcan_test.cpp
  )
  target_link_libraries(test_to_socketcan
    topic_to_socketcan
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(test_to_topic
    test/to_topic.test
    test/to_topic_test.cpp
  )
  target_link_libraries(test_to_topic
    socketcan_to_topic
    topic_to_socketcan
    ${catkin_LIBRARIES}
  )

endif()
