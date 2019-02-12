cmake_minimum_required(VERSION 3.5)
project(socketcan_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)
find_package(Threads REQUIRED)

include_directories(include)

# ${PROJECT_NAME}_string
add_library(${PROJECT_NAME}_string SHARED src/string.cpp)

# socketcan_dump
add_executable(socketcan_dump src/candump.cpp)
ament_target_dependencies(socketcan_dump
  pluginlib
  Boost
)

# socketcan_bcm
add_executable(socketcan_bcm src/canbcm.cpp)
ament_target_dependencies(socketcan_bcm
  Boost
)
target_link_libraries(socketcan_bcm
  ${PROJECT_NAME}_string
)

# ${PROJECT_NAME}_plugin
add_library(${PROJECT_NAME}_plugin SHARED src/${PROJECT_NAME}_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}_plugin
  pluginlib
  Boost
)

pluginlib_export_plugin_description_file(${PROJECT_NAME}_plugin socketcan_interface_plugin.xml)

ament_export_include_directories(include)
ament_export_interfaces(${PROJECT_NAME}_plugin HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ament_cmake
  pluginlib
  Boost
)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    ${PROJECT_NAME}_plugin
    ${PROJECT_NAME}_string
  EXPORT
    ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    include/
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

  ament_add_gtest(${PROJECT_NAME}-test_delegates
    test/test_delegates.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_delegates
    ${PROJECT_NAME}_string
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
  )
  target_compile_options(${PROJECT_NAME}-test_dispatcher PRIVATE -Wno-deprecated-declarations)
endif()

ament_package()
