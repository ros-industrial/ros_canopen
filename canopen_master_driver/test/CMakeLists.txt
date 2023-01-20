ament_add_gtest(test_node_canopen_basic_driver
  test_node_canopen_basic_master.cpp
)
ament_target_dependencies(test_node_canopen_basic_driver
  ${dependencies}
)
target_include_directories(test_node_canopen_basic_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_canopen_basic_driver
  master_driver
  node_canopen_basic_master
  lely_master_bridge
)

ament_add_gtest(test_node_canopen_basic_driver_ros
test_node_canopen_basic_master_ros.cpp
)
ament_target_dependencies(test_node_canopen_basic_driver_ros
  ${dependencies}
)
target_include_directories(test_node_canopen_basic_driver_ros PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_canopen_basic_driver_ros
  master_driver
  node_canopen_basic_master
  lely_master_bridge
)

ament_add_gtest(test_master_driver_component
test_master_driver_component.cpp
)
ament_target_dependencies(test_master_driver_component
  ${dependencies}
)
target_include_directories(test_master_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_master_driver_component
  master_driver
  lifecycle_master_driver
  lely_master_bridge
)

FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
