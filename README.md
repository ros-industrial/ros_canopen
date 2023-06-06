# ROS2 CANopen

[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)


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

## Status
Currently under development. Not for production use.

**Available Features:**
* Device Manager (using rclcpp::components)
* MasterDriver (Service Interface)
* ProxyDriver (Service Interface)
* Cia402Driver (Service Interface)
* Generic ros2_control Interface (implementing `hardware_interface::SystemInterface`) - check https://control.ros.org for more details


**Post build testing**
To test stack after it was built from source you should first setup a virtual can network.
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
```
Then you can run the integration tests contained in canopen_tests package.
```bash
launch_test src/ros2_canopen/canopen_tests/launch_tests/test_proxy_lifecycle_driver.py
launch_test src/ros2_canopen/canopen_tests/launch_tests/test_proxy_driver.py
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
