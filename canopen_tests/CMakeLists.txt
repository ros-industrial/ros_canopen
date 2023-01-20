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


generate_dcf(simple)
generate_dcf(cia402)
generate_dcf(cia402_lifecycle)
generate_dcf(simple_lifecycle)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY
  launch_tests/
  DESTINATION share/${PROJECT_NAME}/launch_tests/
)


if(BUILD_TESTING)
  if(CANOPEN_ENABLED)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(launch_tests/test_proxy_driver.py)
    add_launch_test(launch_tests/test_proxy_lifecycle_driver.py)
  endif()
endif()

ament_package()
