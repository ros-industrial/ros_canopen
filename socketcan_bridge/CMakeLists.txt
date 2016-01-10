cmake_minimum_required(VERSION 2.8.3)
project(socketcan_bridge)

find_package(catkin REQUIRED COMPONENTS
  socketcan_interface
  roscpp
  can_msgs
  roslint
)

find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES socketcan_to_topic topic_to_socketcan socketcan_interface roscpp can_msgs
  CATKIN_DEPENDS 
  DEPENDS Boost
)



include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_library(socketcan_to_topic src/socketcan_to_topic.cpp)
target_link_libraries(socketcan_to_topic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(topic_to_socketcan src/topic_to_socketcan.cpp)
target_link_libraries(topic_to_socketcan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(socketcan_to_topic_node src/socketcan_to_topic_node.cpp)
target_link_libraries(socketcan_to_topic_node socketcan_to_topic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(topic_to_socketcan_node src/topic_to_socketcan_node.cpp)
target_link_libraries(topic_to_socketcan_node topic_to_socketcan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(socketcan_bridge_node src/socketcan_bridge_node.cpp)
target_link_libraries(socketcan_bridge_node topic_to_socketcan socketcan_to_topic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  socketcan_to_topic
  socketcan_to_topic_node
  topic_to_socketcan
  topic_to_socketcan_node
  socketcan_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

roslint_cpp()
#for use with catkin_make roslint_socketcan_bridge