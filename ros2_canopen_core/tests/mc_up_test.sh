echo "Setting parameters for ros2_canopen_node"
ros2 param set canopen_master dcf_path $1/technosoft.dcf
ros2 param set canopen_master yaml_path $1/technosoft.yaml

echo "Configuring ros2_canopen_node"
ros2 lifecycle set canopen_master configure
# echo "Configuring BasicDeviceNode"
# ros2 lifecycle set mc_device_2 configure

# echo "Activating ros2_canopen_node"
# ros2 lifecycle set canopen_master activate
# echo "Activating BasicDeviceNode"
# ros2 lifecycle set basic_device_2 activate
