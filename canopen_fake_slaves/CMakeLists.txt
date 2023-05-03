cmake_minimum_required(VERSION 3.8)
project(canopen_fake_slaves)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
  lely_core_libraries
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
)

add_executable(
  basic_slave_node
  "src/basic_slave.cpp"
)

target_compile_features(basic_slave_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(basic_slave_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  basic_slave_node
  ${dependencies}
)


add_executable(
  cia402_slave_node
  "src/cia402_slave.cpp"
  "src/motion_generator.cpp"
)
target_compile_features(cia402_slave_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cia402_slave_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  cia402_slave_node
  ${dependencies}
)

install(TARGETS basic_slave_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cia402_slave_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ${dependencies}
)

ament_package()
