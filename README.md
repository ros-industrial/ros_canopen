# ROS2 CANopen

[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)


## Documentation
The documentation is generated using sphinx and doxygen. 
The current manual can be found [here](https://ros-industrial.github.io/ros2_canopen/manual/).
The current api reference can be found [here](https://ros-industrial.github.io/ros2_canopen/api/).

## Status
Currently under development. Not for production use.

**Available Features:**
* Device Manager (using rclcpp::components)
* MasterDriver (Service Interface)
* ProxyDriver (Service Interface)
* Cia402Driver (Service Interface)
* Generic ros2_control Interface (implementing `hardware_interface::SystemInterface`) - check https://control.ros.org for more details
