echo "Activating test slave as simple device"
ros2 lifecycle set canopen_test_slave configure
ros2 param set canopen_test_slave test "simple"
ros2 lifecycle set canopen_test_slave activate

echo "Setting parameters for ros2_canopen_node"
ros2 param set canopen_master dcf_path $1/simple.dcf > /dev/null
ros2 param set canopen_master yaml_path $1/simple.yaml  > /dev/null

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure

echo "Activating ros2_canopen_node"
ros2 lifecycle set canopen_master activate

echo "Publish TPDO"
ros2 topic pub basic_device_2/tpdo ros2_canopen_interfaces/msg/COData "{index: 0x4000, subindex: 0, data: 4, type: 32}"