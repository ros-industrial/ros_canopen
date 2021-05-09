cmake_minimum_required(VERSION 3.5)
project(can_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

set(msg_files
  "msg/Frame.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(
    message_runtime
    std_msgs
)

ament_package(
  CONFIG_EXTRAS
)
