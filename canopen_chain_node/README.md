## Overview

This packages contains the ROS interface for
[socketcan_interface](https://index.ros.org/p/socketcan_interface) and
[canopen_master](https://index.ros.org/p/canopen_master). It can be used
as a stand-alone ROS node, but as well as a base class for profile
specific ROS interfaces, e.g.
[canopen_motor_node](https://index.ros.org/p/canopen_motor_node).

It manages a CANopen bus with one or more nodes, which are configures
with CANopen EDS/DCF files. The ROS node runs a control loop with
CANopen SYNC interval (or with an alternative update interval if the
SYNC protocol is not used).

## Configuration

The configuration can be split into different parts. First the bus has
to be set-up, these settings are shared for all node. Then all must be
configured with their CANopen and ROS interfaces. All parameters must be
loaded into the nodes private namespace

### CANopen Bus layer

The bus settings consists of a CAN interface
([socketcan_interface](https://index.ros.org/p/socketcan_interface)) and
the shared bus settings for
[canopen_master](https://index.ros.org/p/canopen_master).



```highlight
bus:
  device: can0 # socketcan network
  # loopback: false # socket should loop back messages
  # driver_plugin: can::SocketCANInterface
  # master_allocator: canopen::SimpleMaster::Allocator
sync:
  interval_ms: 10 # set to 0 to disable sync
  # update_ms: <interval_ms> #update interval of control loop, must be set explecitly if sync is disabled
  overflow: 0 # overflow sync counter at value or do not set it (0, default)
heartbeat: # simple heartbeat producer, optional!
  rate: 20 # heartbeat rate
  msg: "77f#05" # message to send, cansend format: heartbeat of node 127 with status 5=Started
```



### Node layer

All nodes are listed in the nodes parameter, either as list or a struct,
and require a node ID, a name and the path to the EDS/DCF file.



```highlight
# struct syntax
nodes:
  node1:
    id: 1
    eds_pkg: my_config_package # optionals package  name for relative path
    eds_file: "my_config.dcf" # path to EDS/DCF file
  node2:
    id: 85
    name: other_mode # optional name
    eds_file: "/path/to/my_config.eds" # absolute name
```

 

```highlight
# list syntax
nodes:
  - name: node1
    id: 1
    eds_pkg: my_config_package # optionals package  name for relative path
    eds_file: "my_config.eds" # path to EDS/DCF file
  - name: node2
    id: 85
    eds_file: "/path/to/my_config.dcf" # absolute name
```

 Additional parameters
can set either for each node or in a defaults section:



```highlight
defaults: # optional, all defaults can be overwritten per node
  eds_pkg: my_config_package # optional package  name for relative path
  eds_file: "my_config.dcf" # path to EDS/DCF file
  dcf_overlay: # "ObjectID": "ParameterValue" (both as strings)
    "1016sub1" : "0x7F0064" # heartbeat timeout of 100 ms for master at 127
    "1017": "100" # heartbeat producer
```

 The dcf_overlay can be used to overwrite
specific settings in the EDS/DCF.

### ROS layer

Each node (or in the defaults) can be passed a publish parameter with a
list of object names. The topic types correspond to the object types,
e.g. UNSIGNED16 to
[std_msgs/msg/UInt16](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt16.html).
It is meant for debugging or for simple CANopen interfaces, since it
blocks the control loop. Each entry can have an exclamation mark
appended that forces the driver to reread the object form the node in
each step.

For example: 

```highlight
defaults: # optional, all defaults can be overwritten per node
  publish: ["1001"] # list of objects to be published (one topic per node)
nodes:
  node1:
    # ...
    publish: ["1003sub0!"] # list of objects to be published (one topic per node)
  node2:
    # ...
  node3:
    # ...
```

 will publish on three topics with
the sync/update_rate:

- node1_1003sub0
  ([std_msgs/msg/UInt8](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt8.html)):
  number of errors for node1 (not cached)
- node2_1001
  ([std_msgs/msg/UInt8](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt8.html)):
  error register of node2 (cached)
- node3_1001
  ([std_msgs/msg/UInt8](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt8.html)):
  error register of node3 (cached)

The [diagnostic_updater](https://index.ros.org/p/diagnostic_updater) can
be passed the diagnostic_period parameter in fractional seconds.

## ROS API

### canopen_chain_node

#### Published Topics

- `/diagnostics`
  ([diagnostic_msgs/msg/DiagnosticStatus](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticStatus.html)):
  diagnostic information for the chain and each individual node

#### Services

- `driver/init`
  ([std_svrs/srv/Trigger](https://docs.ros.org/en/rolling/p/std_svrs/srv/Trigger.html)):
  initialize the driver
- `driver/halt`
  ([std_svrs/srv/Trigger](https://docs.ros.org/en/rolling/p/std_svrs/srv/Trigger.html)):
  halt movements
- `driver/recover`
  ([std_svrs/srv/Trigger](https://docs.ros.org/en/rolling/p/std_svrs/srv/Trigger.html)):
  recover from emergency stops etc.
- `driver/shutdown`
  ([std_svrs/srv/Trigger](https://docs.ros.org/en/rolling/p/std_svrs/srv/Trigger.html)):
  shutdown all driver connections
- `driver/get_object`
  ([canopen_chain_node/srv/GetObject](https://docs.ros.org/en/rolling/p/canopen_chain_node/srv/GetObject.html)):
  read CANOpen object. Specify the node using the joint name. If
  cached=False an SDO is sent to retrieve the current value, otherwise
  returns the last value received.
- `driver/set_object`
  ([canopen_chain_node/srv/SetObject](https://docs.ros.org/en/rolling/p/canopen_chain_node/srv/SetObject.html)):
  write CANOpen register value.

#### Parameters

- `~diagnostic_period` (`double`, default: `1.0`): period for
  diagnostics updater
- `~bus` (`struct`): bus settings
- `~sync` (`struct`): SYNC interval settings
- `~heartbeat` (`struct`): heartbeat settings
- `~defaults` (`struct`): default settings for all nodes
- `~nodes` (`struct/list`): settings for the nodes

Further topics will be published according to the publish parameters.

## C++ API

This package provides a `canopen::RosChain` class that implements a
`canopen::LayerStack` interface and takes care of the parameter
parsing and the life-cycle. It can be customized via its virtual
interface, especially with `RosChain::nodeAdded`
