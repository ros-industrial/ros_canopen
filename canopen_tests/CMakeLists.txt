cmake_minimum_required(VERSION 3.8)
project(canopen_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)

dcfgen(${CMAKE_CURRENT_SOURCE_DIR}/config/simple/ simple.yml ${CMAKE_BINARY_DIR}/config/simple/)
dcfgen(${CMAKE_CURRENT_SOURCE_DIR}/config/cia402/ cia402.yml ${CMAKE_BINARY_DIR}/config/cia402/)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY
  launch_tests/
  DESTINATION share/${PROJECT_NAME}/launch_tests/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY
  ${CMAKE_BINARY_DIR}/config/
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

ament_package()
