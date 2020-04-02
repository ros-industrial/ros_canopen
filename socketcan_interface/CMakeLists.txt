cmake_minimum_required(VERSION 3.0.2)
project(socketcan_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    class_loader
)

find_package(console_bridge REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_string
  CATKIN_DEPENDS
  DEPENDS
    Boost
    console_bridge
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

# ${PROJECT_NAME}_string
add_library(${PROJECT_NAME}_string
    src/string.cpp
)

# socketcan_dump
add_executable(socketcan_dump
  src/candump.cpp
)

target_link_libraries(socketcan_dump
   ${PROJECT_NAME}_string
   ${console_bridge_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
)

# socketcan_bcm
add_executable(socketcan_bcm
  src/canbcm.cpp
)

target_link_libraries(socketcan_bcm
   ${PROJECT_NAME}_string
   ${console_bridge_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

# ${PROJECT_NAME}_plugin
add_library(${PROJECT_NAME}_plugin
  src/${PROJECT_NAME}_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${console_bridge_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    ${PROJECT_NAME}_plugin
    ${PROJECT_NAME}_string
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
    ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}-test_dummy_interface
    test/test_dummy_interface.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_dummy_interface
    ${PROJECT_NAME}_string
    ${console_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(${PROJECT_NAME}-test_delegates
    test/test_delegates.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_delegates
    ${PROJECT_NAME}_string
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(${PROJECT_NAME}-test_string
    test/test_string.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_string
    ${PROJECT_NAME}_string
    ${console_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(${PROJECT_NAME}-test_filter
    test/test_filter.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_filter
    ${PROJECT_NAME}_string
    ${console_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(${PROJECT_NAME}-test_dispatcher
    test/test_dispatcher.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_dispatcher
    ${PROJECT_NAME}_string
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  target_compile_options(${PROJECT_NAME}-test_dispatcher PRIVATE -Wno-deprecated-declarations)
endif()
