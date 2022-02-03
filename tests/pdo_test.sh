# Parameter 1 is the path to the spec files.

echo "Activating test slave as pdo_counter"
ros2 lifecycle set canopen_test_slave configure
ros2 param set canopen_test_slave test "pdo_counter"
ros2 lifecycle set canopen_test_slave activate

echo "Setting parameters for ros2_canopen_node"
ros2 param set canopen_master dcf_path $1/simple.dcf
ros2 param set canopen_master yaml_path $1/simple.yaml

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure
echo "Configuring BasicDeviceNode"
ros2 lifecycle set basic_device_2 configure

echo "Activating ros2_canopen_node"
ros2 lifecycle set canopen_master activate
echo "Activating BasicDeviceNode"
ros2 lifecycle set basic_device_2 activate

