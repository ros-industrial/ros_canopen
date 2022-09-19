# ROS2 CANopen

[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/foxy/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/galactic/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/humble/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)


| Package         | License     |
|--------------|-----------|
| canopen | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0) |
| canopen_core      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |
| canopen_interfaces      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |
| canopen_base_driver      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |
| canopen_proxy_driver      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |
| canopen_402_driver      | [![license - LGPLv3](https://img.shields.io/:license-LGPL%203.0-yellow.svg)](https://opensource.org/licenses/LGPL-3.0)  |
| canopen_utils      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |
| lely_core_libraries      | [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)  |

## Documentation
The documentation is generated using sphinx and doxygen. The current version for master can be found [here](https://ros-industrial.github.io/ros2_canopen/).

## Status
Currently under development. Not for production use.

**Available Features:**
* Device Manager (using rclcpp::components)
* CANopen Master (Service Interface)
* ProxyDriver (Service Interface)
* MotionControllerDriver (Service Interface)
* Generic ros2_control Interface (implementing `hardware_interface::SystemInterface`) - check https://control.ros.org for more details
