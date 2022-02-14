# Parameter 1 is the path to the spec files.

echo "Activating test slave as simple device"
ros2 lifecycle set canopen_test_slave configure
ros2 param set canopen_test_slave test "simple"
ros2 lifecycle set canopen_test_slave activate 

echo "Setting parameters for ros2_canopen_node"
ros2 param set canopen_master dcf_path $1/simple.dcf 
ros2 param set canopen_master yaml_path $1/simple.yaml 

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure

echo "Activating ros2_canopen_node"
ros2 lifecycle set canopen_master activate

echo "Testing NMT by ID"
ros2 service call /canopen_master/set_nmt ros2_canopen_interfaces/srv/CONmtID "{nmtcommand: 0x81, nodeid: 2}"
