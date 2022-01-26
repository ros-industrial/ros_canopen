# Prerquisits

Currently lelycore needs to be installed seperately as it is an automake project and not easily integrated into ROS workflow.
Probably need to create some kind of ROS package wrapper at some point.

Please follow: https://opensource.lely.com/canopen/docs/installation/

# How to run tests

1. Setup a vcan device
```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
``` 
2. Change the simple test slave for your setup.

In src/test_slave.cpp add the absolute path to simple.eds.

3. Compile Code with:
```
colcon build
```

4. Run test_slave and candump:

Terminal 1:
```
ros2 run ros2_canopen test_slave
```
Terminal 2:
```
candump vcan0
```

3. Run the master node:
Treminal 3
```
ros2 run ros2_canopen ros2_canopen_node
```

4. Set parameters "dcf_path" and "yaml_path" to point to ressources/master.dcf and ressources/bus.yaml respectively. Probably needs to be an absolute path.

Terminal 4:
```
ros2 param set /test_node dcf_path [Absolute_Path]
ros2 param set /test_node yaml_path [Absolute_Path]
```

5. Activate master node and driver:

Terminal 4:
```
ros2 lifecycle set /canopen_master configure
ros2 lifecycle set /BasicDevice2 configure
ros2 lifecycle set /canopen_master activate
ros2 lifecycle set /BasicDevice2 activate
```

6. Observe some stuff:

Topic nmt_state should show state of driver device.

Service nmt_reset_node is a std_srv/srv/Trigger and should simply reset the device. This should reflect in nmt_state.