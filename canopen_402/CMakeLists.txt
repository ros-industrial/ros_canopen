cmake_minimum_required(VERSION 3.5)
project(canopen_402)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(canopen_master REQUIRED)
find_package(class_loader REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    atomic
    chrono
    thread
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# canopen_402
add_library(${PROJECT_NAME} SHARED
  src/motor.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "canopen_master"
)

# canopen_402_plugin
add_library(${PROJECT_NAME}_plugin SHARED
  src/plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
   ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  "canopen_master"
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} canopen_402_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_plugin)

install(
  TARGETS
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

  ament_add_gtest(test_clamping
    test/clamping.cpp
  )

  target_link_libraries(test_clamping
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
  )
endif()

ament_package()
