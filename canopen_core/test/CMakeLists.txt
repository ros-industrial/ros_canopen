ament_add_gtest(test_node_canopen_driver
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

ament_add_gtest(test_node_canopen_master
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

ament_add_gtest(test_device_container
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

FILE(COPY master.dcf bus.yml simple.eds DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
