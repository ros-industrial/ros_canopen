echo "Activating test slave as simple device"
ros2 param set canopen_test_slave test "simple"
ros2 param set canopen_test_slave eds "/home/christoph/ws_ros2/src/ros2_canopen/ros2_canopen_core/resources/simple.eds"
ros2 lifecycle set canopen_test_slave configure
ros2 lifecycle set canopen_test_slave activate

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure

echo "Activating ros2_canopen_node"
ros2 lifecycle set canopen_master activate

