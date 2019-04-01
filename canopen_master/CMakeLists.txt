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
  ${class_loader_INCLUDE_DIRS}
  ${socketcan_interface_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${socketcan_interface_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/emcy.cpp
  src/node.cpp
  src/objdict.cpp
  src/pdo.cpp
  src/sdo.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${class_loader_LIBRARIES}
  ${socketcan_interface_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  socketcan_interface
)


add_library(${PROJECT_NAME}_plugin
  src/master_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${class_loader_LIBRARIES}
  ${socketcan_interface_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} master_plugin.xml)

# canopen_bcm_sync
add_executable(canopen_bcm_sync
  src/bcm_sync.cpp
)
target_link_libraries(canopen_bcm_sync
  ${class_loader_LIBRARIES}
  ${socketcan_interface_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_export_dependencies(
  ament_cmake
  Boost
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS
    canopen_bcm_sync
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugin
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
