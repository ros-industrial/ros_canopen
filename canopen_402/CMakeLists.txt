cmake_minimum_required(VERSION 2.8.3)
project(canopen_402)

find_package(catkin REQUIRED
  COMPONENTS
    canopen_master
    class_loader
)

find_package(Boost REQUIRED
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    canopen_402
  CATKIN_DEPENDS
    canopen_master
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# canopen_402
add_library(canopen_402
  src/motor.cpp
)
target_link_libraries(canopen_402
  ${catkin_LIBRARIES}
)

# canopen_402_plugin
add_library(canopen_402_plugin
  src/plugin.cpp
)
target_link_libraries(canopen_402_plugin
   ${catkin_LIBRARIES}
   canopen_402
)

install(
  TARGETS
    canopen_402
    canopen_402_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(
  FILES
    canopen_402_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test_clamping
    test/clamping.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_clamping
    ${catkin_LIBRARIES}
  )
endif()
