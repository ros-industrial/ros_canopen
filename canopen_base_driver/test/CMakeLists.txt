ament_add_gtest(test_node_canopen_base_driver_ros
test_node_canopen_base_driver_ros.cpp
)
ament_target_dependencies(test_node_canopen_base_driver_ros
  ${dependencies}
)
target_include_directories(test_node_canopen_base_driver_ros PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_canopen_base_driver_ros
  node_canopen_base_driver
)




ament_add_gtest(test_base_driver_component
test_base_driver_component.cpp
)
ament_target_dependencies(test_base_driver_component
  ${dependencies}
)
target_include_directories(test_base_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )


FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
