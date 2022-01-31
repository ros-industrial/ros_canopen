# Parameter 1 is the path to the spec files.

echo "Activating test slave as simple device"
ros2 lifecycle set canopen_test_slave configure
ros2 param set canopen_test_slave test "simple"
ros2 lifecycle set canopen_test_slave activate

echo "Setting parameters for ros2_canopen_node"
ros2 param set canopen_master dcf_path $1/master.dcf
ros2 param set canopen_master yaml_path $1/bus.yaml

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure
echo "Configuring BasicDeviceNode"
ros2 lifecycle set BasicDevice2 configure

echo "Activating ros2_canopen_node"
ros2 lifecycle set canopen_master activate
echo "Activating BasicDeviceNode"
ros2 lifecycle set BasicDevice2 activate

echo "Resetting device"
ros2 service call BasicDevice2/nmt_reset_node std_srvs/srv/Trigger