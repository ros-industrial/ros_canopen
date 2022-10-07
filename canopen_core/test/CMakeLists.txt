ament_add_gmock(test_node_canopen_driver
    test_node_canopen_driver.cpp
)
ament_target_dependencies(test_node_canopen_driver
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
)
target_include_directories(test_node_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_canopen_driver
    node_canopen_driver
)

ament_add_gmock(test_node_canopen_master
  test_node_canopen_master.cpp
)
ament_target_dependencies(test_node_canopen_master
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
)
target_include_directories(test_node_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_canopen_master
    node_canopen_master
)


ament_add_gmock(test_device_container
  test_device_container.cpp
)
ament_target_dependencies(test_device_container
  ${dependencies}
)
target_include_directories(test_device_container PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_device_container
    node_canopen_master
    node_canopen_driver
    device_container
)
FILE(COPY bus_configs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

ament_add_gmock(test_canopen_driver
test_canopen_driver.cpp
)
ament_target_dependencies(test_canopen_driver
  ${dependencies}
)
target_include_directories(test_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_canopen_driver
    node_canopen_driver
)

ament_add_gmock(test_lifecycle_canopen_driver
test_lifecycle_canopen_driver.cpp
)
ament_target_dependencies(test_lifecycle_canopen_driver
  ${dependencies}
)
target_include_directories(test_lifecycle_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_lifecycle_canopen_driver
    node_canopen_driver
)

ament_add_gmock(test_canopen_master
test_canopen_master.cpp
)
ament_target_dependencies(test_canopen_master
  ${dependencies}
)
target_include_directories(test_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_canopen_master
    node_canopen_master
)

ament_add_gmock(test_lifecycle_canopen_master
test_lifecycle_canopen_master.cpp
)
ament_target_dependencies(test_lifecycle_canopen_master
  ${dependencies}
)
target_include_directories(test_lifecycle_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_lifecycle_canopen_master
    node_canopen_master
)

ament_add_gmock(test_lifecycle_manager
test_lifecycle_manager.cpp
)
ament_target_dependencies(test_lifecycle_manager
  ${dependencies}
)
target_include_directories(test_lifecycle_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_lifecycle_manager
  device_container
)

ament_add_gmock(test_errors
test_errors.cpp
)
ament_target_dependencies(test_errors
  ${dependencies}
)
target_include_directories(test_errors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_errors
  device_container
  node_canopen_master
  node_canopen_driver
)