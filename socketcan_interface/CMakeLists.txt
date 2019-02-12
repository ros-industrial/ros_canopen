cmake_minimum_required(VERSION 3.5)
project(socketcan_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)
find_package(Threads REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)

# ${PROJECT_NAME}_string
ament_auto_add_library(${PROJECT_NAME}_string src/string.cpp)

target_link_libraries(${PROJECT_NAME}_string
  ${Boost_LIBRARIES}
)

# socketcan_dump
ament_auto_add_executable(socketcan_dump src/candump.cpp)

target_link_libraries(socketcan_dump
  ${PROJECT_NAME}_string
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# socketcan_bcm
ament_auto_add_executable(socketcan_bcm src/canbcm.cpp)

target_link_libraries(socketcan_bcm
  ${PROJECT_NAME}_string
  ${Boost_LIBRARIES}
)

# ${PROJECT_NAME}_plugin
ament_auto_add_library(${PROJECT_NAME}_plugin src/${PROJECT_NAME}_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
  ${Boost_LIBRARIES}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME}_plugin socketcan_interface_plugin.xml)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(class_loader)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    ${PROJECT_NAME}_plugin
    ${PROJECT_NAME}_string
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
  )

  ament_add_gtest(${PROJECT_NAME}-test_filter
    test/test_filter.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_filter
    ${PROJECT_NAME}_string
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
