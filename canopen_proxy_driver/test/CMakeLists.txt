ament_add_gtest(test_node_interface
test_node_interface.cpp
)
ament_target_dependencies(test_node_interface
  ${dependencies}
)
target_include_directories(test_node_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )
target_link_libraries(test_node_interface
  node_canopen_proxy_driver
)




ament_add_gtest(test_driver_component
test_driver_component.cpp
)
ament_target_dependencies(test_driver_component
  ${dependencies}
)
target_include_directories(test_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
  )


FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
