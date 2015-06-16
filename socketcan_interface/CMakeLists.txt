cmake_minimum_required(VERSION 2.8.3)
project(socketcan_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS class_loader)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)


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
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(socketcan_interface
#   src/${PROJECT_NAME}/socketcan_interface.cpp
# )

## Declare a cpp executable
add_executable(socketcan_dump src/candump.cpp)
target_link_libraries(socketcan_dump
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_library(socketcan_interface_plugin src/socketcan_interface_plugin.cpp)
target_link_libraries(socketcan_interface_plugin
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(socketcan_interface_node socketcan_interface_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(socketcan_interface_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${boost_LIBRARIES}
# )

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
install(TARGETS socketcan_dump socketcan_interface_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES socketcan_interface_plugin.xml
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
if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}-test_dummy_interface test/test_dummy_interface.cpp)
  target_link_libraries(${PROJECT_NAME}-test_dummy_interface ${catkin_LIBRARIES})

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
