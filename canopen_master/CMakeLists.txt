cmake_minimum_required(VERSION 2.8.3)
project(canopen_master)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  socketcan_interface
  class_loader
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS chrono atomic)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES canopen_master
  CATKIN_DEPENDS socketcan_interface
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(canopen_master
  src/objdict.cpp
  src/sdo.cpp
  src/pdo.cpp
  src/emcy.cpp
  src/node.cpp
  src/master.cpp
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

## Declare a cpp executable
# add_executable(canopen_master_node src/canopen_master_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(canopen_master_node canopen_master_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS canopen_master canopen_master_plugin
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES master_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_canopen_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
