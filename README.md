# ROS2 CANopen

## Status

[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)

The stack is currently under development and not yet ready for production use.

## Documentation
The documentation consists of two parts: a manual and an api reference.
The documentation is built for rolling (master), iron and humble and hosted on github pages.
Older ROS 2 releases are EOL and are not supported anymore.

### Rolling
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/rolling/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/rolling/

### Iron
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/iron/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/iron/

### Humble
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/humble/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/humble/

## Features
These are some of the features this stack implements. For further information please refer to the documentation.

* **YAML-Bus configuration**
  This canopen stack enables you to configure the bus using a YAML file. In this file you define the nodes that are connected to the bus by specifying their node id, the corresponding EDS file and the driver to run for the node. You can also specify further parameters that overwrite EDS parameters or are inputs to the driver.
* **Service based operation**
  The stack can be operated using standard ROS2 nodes. In this case the device container will load the drivers for master and slave nodes. Each driver will be visible as a
  node and expose a ROS 2 interface. All drivers are brought up when the device manager is launched.
* **Managed service based operation**
  The stack can be opeprated using managed ROS2 nodes. In
  this case the device container will load the drivers for master and slave nodes based on the bus configuration. Each driver will be a lifecycle node and expose a ROS 2 interface. The lifecycle manager can be used to bring all
  device up and down in the correct sequence.
* **ROS2 control based operation**
  Currently, multiple ros2_control interfaces are available. These can be used for controlling CANopen devices. The interfaces are:
  * canopen_ros2_control/CANopenSystem
  * canopen_ros2_control/CIA402System
  * canopen_ros2_control/RobotSystem
* **CANopen drivers**
  Currently, the following drivers are available:
    * ProxyDriver
    * Cia402Driver


## Post testing
To test stack after it was built from source you should first setup a virtual can network.
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
```
Then you can launch a managed example
```bash
ros2 launch canopen_tests cia402_lifecycle_setup.launch.py
ros2 lifecycle set /lifecycle_manager configure
ros2 lifecycle set /lifecycle_manager activate
```

Or you can launch a standard example
```bash
ros2 launch canopen_tests cia402_setup.launch.py
```

Or you can launch a ros2_control example
```bash
ros2 launch canopen_tests robot_control_setup.launch.py
```

## Contributing
This repository uses `pre-commit` for code formatting.
This program has to be setup locally and installed inside the repository.
For this execute in the repository folder following commands:
```
sudo apt install -y pre-commit
pre-commit install
```
The checks are automatically executed before each commit.
This helps you to always commit well formatted code.
To run all the checks manually use `pre-commit run -a` command.
For the other options check `pre-commit --help`.

In a case of an "emergency" you can avoid execution of pre-commit hooks by adding `-n` flag to `git commit` command - this is NOT recommended to do if you don't know what are you doing!
