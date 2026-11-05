## Overview

The canopen_motor_node package provides a
[ros_control](https://index.ros.org/p/ros_control) interface to
\[canopen_402\]\] compliant motors. It is based on
[canopen_chain_node](https://index.ros.org/p/canopen_chain_node) and
inherits all its interfaces, so this site just focuses on the additional
interfaces.

## Concepts

### controller manager

The node includes a
[controller_manager](https://index.ros.org/p/controller_manager)
instance that can be used to spawn
[controller_interface](https://index.ros.org/p/controller_interface)
compliant controllers. Depending on the motor device different
interfaces are support.

For each joint a `hardware_interface::JointStateHandle` is registered
with the position, velocity and effort of the specific drive. The actual
value is determined by the conversion functions.

In addition handles for the following command interfaces are available
if the given modes are supported by the device:

- `hardware_interface::PositionJointInterface`:
- Profiled Position
- Interpolated Position
- Cyclic Synchronous Position
- `hardware_interface::VelocityJointInterface`:
- Velocity
- Profiled Velocity
- Cyclic Synchronous Velocity
- `hardware_interface::EffortJointInterface`:
- Profiled Torque
- Cyclic Synchronous Torque

#### joint limits

The output from the controller is limited by
[joint_limits_interface](https://index.ros.org/p/joint_limits_interface)
and not passed directly to the motor. The limits can be read from URDF
or the ParameterServer:
<https://github.com/ros-controls/ros_control/wiki/joint_limits_interface>.

If certain limits are specified, the driver registers appropriate
handles for soft and saturation limits.

#### mode switching

The driver does support mode switching at run-time and mixed modes for
different joints. In order to determine the mode, a special parameter
"required_drive_mode" must be added to each controller. It should
contain the [CANopen 402 mode id](https://index.ros.org/p/canopen_402).

Since mode switching might be restricted to specific drive states by the
manufacturer, the drive state for mode switching can be specified for
each mode.

### unit conversions

The driver features [muparser](http://muparser.beltoforion.de/) based
conversion functions.

In addition to the predefined
[functions](http://muparser.beltoforion.de/mup_features.html#idDef2) and
[operators](http://muparser.beltoforion.de/mup_features.html#idDef3),
the following functions are available:

- rad2deg(radians): convert from radians to degrees
- deg2rad(degrees): convert from degrees to radians
- norm(value, min, max): normalize value to the range \[min, max)
- smooth(value, old_value, alpha):
  [exponential smoothing](https://en.wikipedia.org/wiki/Exponential_smoothing),
  returns alpha\*val + (1.0-alpha)\*old_val
- avg(val1,val2,\...): average functions that stops on the first NaN
  value

Further two constants are defined for convenience: `pi` and `nan`.

The conversion functions from ROS the device units can read the commands
from the respective `pos`,`vel` and `eff` variables.

The conversion functions from device to ROS supports an extended notion
with obj prefixes: E.g. `obj6064` will read the actual position and
`obj6091sub1` will read the gear ration nominator.



```wiki
Please note that you can write to these variables, but you should avoid it, these writes are not passed to the device anyway and will overwritten in the next call!
```



You can define more variables by just using them, they will be
initialized to NaN values.

## Configuration

The basic configuration is described in
[canopen_chain_node](https://index.ros.org/p/canopen_chain_node).

For the canopen_motor_node to work each node is given a `joint`
parameter that defaults to the CANopen node name and is used as joint
name in the [ros_control](https://index.ros.org/p/ros_control)
interfaces.

For each motor node a `motor_allocator` can be specified which is used
to load a motor plugin with
[pluginlib](https://index.ros.org/p/pluginlib), it defaults to the
[canopen_402](https://index.ros.org/p/canopen_402) implementation. All
settings contained in the 'motor_layer' parameter will be passed to
the motor layer instance.

In summary the following parameters are supported in `defaults`
section and per node: 

```highlight
defaults: # optional, all defaults can be overwritten per node
  # canopen_chain_node settings ..
  motor_allocator: canopen::Motor402::Allocator # select allocator for motor layer
  motor_layer: settings passed to motor layer (plugin-specific)
    switching_state: 5 # (Operation_Enable), state for mode switching
  pos_to_device: "rint(rad2deg(pos)*1000)" # rad -> mdeg
  pos_from_device: "deg2rad(obj6064)/1000" # actual position [mdeg] -> rad
  vel_to_device: "rint(rad2deg(vel)*1000)" # rad/s -> mdeg/s
  vel_from_device: "deg2rad(obj606C)/1000" # actual velocity [mdeg/s] -> rad/s
  eff_to_device: "rint(eff)" # just round to integer
  eff_from_device: "0" # unset
```

 If omitted, the listed
values are used.

``The objects used in the conversion functions and the drive modes
should be mapped to PDOs for best performance.``

## ROS API

The canopen_motor_node exposes the ROS interfaces of
[canopen_chain_node](https://index.ros.org/p/canopen_chain_node).



```wiki
Please note that the `init` service will try to perform the drive homing if it's supported.
```



In addition the interfaces of
[controller_manager](https://index.ros.org/p/controller_manager) are
available after initialization. However, the controller_manager is
stopped when shutdown service ist called.

The
[joint_limits_interface](https://index.ros.org/p/joint_limits_interface)
will read its limits from the URDF and from the !ParameterServer. The
latter takes precedence but does not support soft limits.

The spawned controllers will provide additional interfaces.

## C++ API

The current implementation does not expose a C++ API, but it is planned
for the future:
[Issue #101](https://github.com/ros-industrial/ros_canopen/issues/101)
