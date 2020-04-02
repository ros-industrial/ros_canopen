cmake_minimum_required(VERSION 3.0.2)
project(canopen_master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    class_loader
    socketcan_interface
)

find_package(Boost REQUIRED
  COMPONENTS
    atomic
    chrono
    thread
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    socketcan_interface
  DEPENDS
    Boost
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/emcy.cpp
  src/node.cpp
  src/objdict.cpp
  src/pdo.cpp
  src/sdo.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_plugin
  src/master_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

# canopen_bcm_sync
add_executable(canopen_bcm_sync
  src/bcm_sync.cpp
)
target_link_libraries(canopen_bcm_sync
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    canopen_bcm_sync
    ${PROJECT_NAME}
   ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES
    master_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test_parser
    test/test_parser.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_parser
    ${PROJECT_NAME}
  )
endif()
