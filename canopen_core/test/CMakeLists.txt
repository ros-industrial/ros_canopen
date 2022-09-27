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


ament_add_gtest(test_canopen_driver
    test_canopen_driver.cpp
)
ament_target_dependencies(test_canopen_driver
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
  yaml_cpp_vendor
  canopen_interfaces
)
target_include_directories(test_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_canopen_driver
    node_canopen_driver
)