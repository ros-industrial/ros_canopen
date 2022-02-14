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


echo "Service call device"
for i in {100..110}
do 
    ros2 service call basic_device_2/sdo_write ros2_canopen_interfaces/srv/COWrite "{index: 0x1017, subindex: 0, data: $i, type: 16}"
    ros2 service call basic_device_2/sdo_read ros2_canopen_interfaces/srv/CORead "{index: 0x1017, subindex: 0, type: 16}" 
done
