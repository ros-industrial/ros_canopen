cmake_minimum_required(VERSION 3.5)
project(canopen_master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(socketcan_interface REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    atomic
    chrono
    thread
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/emcy.cpp
  src/node.cpp
  src/objdict.cpp
  src/pdo.cpp
  src/sdo.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "class_loader"
  "socketcan_interface"
)
target_link_libraries(${PROJECT_NAME}
  socketcan_interface::socketcan_interface_plugin
  socketcan_interface::socketcan_interface_string
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_plugin SHARED
  src/master_plugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}_plugin
  "class_loader"
  "socketcan_interface"
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} master_plugin.xml)

# canopen_bcm_sync
add_executable(canopen_bcm_sync
  src/bcm_sync.cpp
)
ament_target_dependencies(canopen_bcm_sync
  "class_loader"
  "socketcan_interface"
)
target_link_libraries(canopen_bcm_sync
  socketcan_interface::socketcan_interface_plugin
  socketcan_interface::socketcan_interface_string
  ${Boost_LIBRARIES}
)

ament_export_dependencies(
  ament_cmake
  socketcan_interface
  pluginlib
  Boost
)

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_interfaces(export_${PROJECT_NAME}_plugin HAS_LIBRARY_TARGET)

install(
  TARGETS canopen_bcm_sync
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parser
    test/test_parser.cpp
  )

  target_link_libraries(test_parser
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
  )

endif()

ament_package()
