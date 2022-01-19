cmake_minimum_required(VERSION 3.5)
project(ros2_canopen)

include(FindPkgConfig)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra)# -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Liblely is automake project, needs to be imported using pkg_check
pkg_check_modules(LIBLELY-IO2 REQUIRED liblely-io2)
pkg_check_modules(LIBLELY-COAPP REQUIRED liblely-coapp)

add_executable(test_node 
  src/test_node.cpp
  )
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${yaml_cpp_vendor_INCLUDE_DIR}
  )


target_link_libraries(test_node PRIVATE
  ${LIBLELY-IO2_LIBRARIES}
  ${LIBLELY-COAPP_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  ${yaml_cpp_vendor_LIBRARIES}
  )

install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
