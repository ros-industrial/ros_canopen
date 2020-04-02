cmake_minimum_required(VERSION 3.0.2)
project(canopen_motor_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    canopen_402
    canopen_chain_node
    canopen_master
    controller_manager
    controller_manager_msgs
    filters
    hardware_interface
    joint_limits_interface
    roscpp
    urdf
)

find_package(Boost REQUIRED
  COMPONENTS
    thread
)

find_package(PkgConfig)
pkg_check_modules(PC_MUPARSER QUIET muparser)
set(MUPARSER_DEFINITIONS ${PC_MUPARSER_CFLAGS_OTHER})

find_path(MUPARSER_INCLUDE_DIR muParser.h
          HINTS ${PC_MUPARSER_INCLUDEDIR} ${PC_MUPARSER_INCLUDE_DIRS}
)

find_library(MUPARSER_LIBRARY NAMES muparser libmurser
             HINTS ${PC_MUPARSER_LIBDIR} ${PC_MUPARSER_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MUPARSER_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MUPARSER  DEFAULT_MSG
                                  MUPARSER_LIBRARY MUPARSER_INCLUDE_DIR)
mark_as_advanced(MUPARSER_INCLUDE_DIR MUPARSER_LIBRARY )

if(NOT ${MUPARSER_FOUND})
  message(FATAL_ERROR "muparser library not found")  
endif()

set(MUPARSER_LIBRARIES ${MUPARSER_LIBRARY} )
set(MUPARSER_INCLUDE_DIRS ${MUPARSER_INCLUDE_DIR} )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    canopen_motor
  CATKIN_DEPENDS
    canopen_402
    canopen_chain_node
    canopen_master
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    urdf
  DEPENDS
    Boost
    MUPARSER
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MUPARSER_INCLUDE_DIRS}
)

# canopen_motor
add_library(canopen_motor
  src/controller_manager_layer.cpp
  src/handle_layer.cpp
  src/motor_chain.cpp
  src/robot_layer.cpp
)
target_link_libraries(canopen_motor
  ${catkin_LIBRARIES}
  ${MUPARSER_LIBRARIES}
)
add_dependencies(canopen_motor
  ${catkin_EXPORTED_TARGETS}
)

# canopen_motor_node
add_executable(${PROJECT_NAME}
  src/canopen_motor_chain_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  canopen_motor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    canopen_motor
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test_muparser
    test/test_muparser.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_muparser
    canopen_motor
    ${catkin_LIBRARIES}
    ${MUPARSER_LIBRARIES}
  )
endif()
