cmake_minimum_required(VERSION 2.8.3)
project(canopen_master)

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
    canopen_master
  CATKIN_DEPENDS
    socketcan_interface
  DEPENDS
    Boost
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(canopen_master
  src/emcy.cpp
  src/master.cpp
  src/node.cpp
  src/objdict.cpp
  src/pdo.cpp
  src/sdo.cpp
)
target_link_libraries(canopen_master
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(canopen_master_plugin
  src/master_plugin.cpp
)

target_link_libraries(canopen_master_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  canopen_master
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
    canopen_master
    canopen_master_plugin
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
    canopen_master
  )
endif()
