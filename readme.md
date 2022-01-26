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

5. Activate master node:

Terminal 4:
```
ros2 lifecycle set /canopen_master configure
ros2 lifecycle set /canopen_master activate
```

6. Try some services:

Terminal 4:

Reading SDO:
```
ros2 service call /master_read16_sdo ros2_canopen_interfaces/srv/MasterReadSdo16 "{nodeid: 2, index: 0x1017, subindex: 0}"
```

Writing SDO: (Example sets Heartbeat, data is in ms, check candump)
```
ros2 service call /master_write16_sdo ros2_canopen_interfaces/srv/MasterWriteSdo16 "{nodeid: 2, index: 0x1017, subindex: 0, data: 1000}"
```