cmake_minimum_required(VERSION 3.8)
project(canopen_base_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(canopen_core REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/canopen_base_driver.cpp
  src/lely_bridge.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-unused-parameter)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "canopen_core"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CANOPEN_BASE_DRIVER_BUILDING_LIBRARY")




add_library(lifecycle_canopen_base_driver
  SHARED
  src/lifecycle_canopen_base_driver.cpp
  src/lely_bridge.cpp
)
target_compile_features(lifecycle_canopen_base_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-unused-parameter)
target_include_directories(lifecycle_canopen_base_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  lifecycle_canopen_base_driver
  "rclcpp"
  "rclcpp_components"
  "canopen_interfaces"
  "std_msgs"
  "std_srvs"
  "lely_core_libraries"
  "canopen_core"
  "rclcpp_lifecycle"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lifecycle_canopen_base_driver PRIVATE "CANOPEN_BASE_DRIVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lifecycle_canopen_base_driver
  EXPORT export_lifecycle_canopen_base_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  lifecycle_canopen_base_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
  export_lifecycle_canopen_base_driver
)

ament_export_dependencies(
  canopen_core
)



ament_package()
