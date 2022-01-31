ros2 lifecycle set canopen_test_slave configure
ros2 param set canopen_test_slave test "pdo_counter"
ros2 lifecycle set canopen_test_slave activate


ros2 param set canopen_master dcf_path /home/christoph/ws_ros2/src/ros2_canopen/ressources/master.dcf
ros2 param set canopen_master yaml_path /home/christoph/ws_ros2/src/ros2_canopen/ressources/bus.yaml

ros2 lifecycle set canopen_master configure
ros2 lifecycle set BasicDevice2 configure

ros2 lifecycle set canopen_master activate
ros2 lifecycle set BasicDevice2 activate

