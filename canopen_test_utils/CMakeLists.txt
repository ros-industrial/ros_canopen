cmake_minimum_required(VERSION 2.8.3)
project(canopen_test_utils)

find_package(catkin REQUIRED
  COMPONENTS
    canopen_402
    canopen_master
)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# canopen_elmo_console
add_executable(canopen_elmo_console
  src/elmo_console.cpp
)
target_link_libraries(canopen_elmo_console
  ${catkin_LIBRARIES}
)
install(
  TARGETS
    canopen_elmo_console
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch/sim_rig1.launch)
  roslaunch_add_file_check(launch/sim_rig2.launch)
  roslaunch_add_file_check(launch/sim_rig12.launch)
  roslaunch_add_file_check(launch/sim_rig1_rig2.launch)

  roslaunch_add_file_check(launch/hw_rig1.launch)
  roslaunch_add_file_check(launch/hw_rig2.launch)
  roslaunch_add_file_check(launch/hw_rig12.launch)
  roslaunch_add_file_check(launch/hw_rig1_rig2.launch)
endif()
