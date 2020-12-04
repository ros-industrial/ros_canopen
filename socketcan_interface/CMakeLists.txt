cmake_minimum_required(VERSION 3.5)
project(socketcan_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)
find_package(Threads REQUIRED)

include_directories(include
  ${class_loader_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ${PROJECT_NAME}_string
add_library(${PROJECT_NAME}_string SHARED
  src/string.cpp
)

# socketcan_dump
add_executable(socketcan_dump src/candump.cpp)
target_link_libraries(socketcan_dump
  ${PROJECT_NAME}_string
  ${class_loader_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# socketcan_bcm
add_executable(socketcan_bcm src/canbcm.cpp)
target_link_libraries(socketcan_bcm
  ${PROJECT_NAME}_string
  ${Boost_LIBRARIES}
)

# ${PROJECT_NAME}_plugin
add_library(${PROJECT_NAME}_plugin SHARED
  src/${PROJECT_NAME}_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${class_loader_LIBRARIES}
  ${Boost_LIBRARIES}
)

class_loader_hide_library_symbols(${PROJECT_NAME}_plugin)
install(FILES socketcan_interface_plugin.xml DESTINATION share/${PROJECT_NAME}/socketcan_interface_plugin.xml)
ament_index_register_resource(socketcan_interface__pluginlib__plugin CONTENT "share/${PROJECT_NAME}/socketcan_interface_plugin.xml\n")

ament_export_dependencies(
  class_loader
  Boost
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_string)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    ${PROJECT_NAME}_string
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

  ament_add_gtest(${PROJECT_NAME}-test_dummy_interface
    test/test_dummy_interface.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_dummy_interface
    ${PROJECT_NAME}_string
    ${Boost_LIBRARIES}
  )

  ament_add_gtest(${PROJECT_NAME}-test_string
    test/test_string.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_string
    ${PROJECT_NAME}_string
    ${Boost_LIBRARIES}
  )

  ament_add_gtest(${PROJECT_NAME}-test_filter
    test/test_filter.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_filter
    ${PROJECT_NAME}_string
    ${Boost_LIBRARIES}
  )

  ament_add_gtest(${PROJECT_NAME}-test_dispatcher
    test/test_dispatcher.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_dispatcher
    ${PROJECT_NAME}_string
    ${Boost_LIBRARIES}
  )
endif()

ament_package(
  CONFIG_EXTRAS socketcan_interface-extras.cmake
)
