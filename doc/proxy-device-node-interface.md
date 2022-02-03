# Interface description of CANopen Master Node
The CANopen Master node is a lifecycle node.

## Lifecycle States
| State                   |  Description       |
|---------------------------|-----------------------|
| Unconfigured | Directly after start of the node. |
| Configured | Node is configured. |
| Active | All services available |

## Lifecycle Tranisitions
| State                   |  Description       |
|---------------------------|-----------------------|
| Configure | Nothing to do here. |
| Activate | Enables publishers and services. |
| Deactivate | Disables publishers and services. |
| Cleanup | Nothing to do here. |
| Shutdown | TBD |

## ROS Services
| Service                   | Type                  | Description       |
|---------------------------|-----------------------|------------------|
| /proxy_device_[nodeid]/nmt_reset_node | Trigger | Resets CANopen Device the Proxy Device Node manages. |
| /proxy_device_[nodeid]/sdo_read | CORead | Reads an SDO object from the specified index, subindex and datatype of the remote device. |
| /proxy_device_[nodeid]/sdo_write | COWrite | Writes data to an SDO object on the specified index, subindex and datatype of the remote device. |

## ROS Publishers

| Topic                   | Type                  | Description       |
|---------------------------|-----------------------|------------------|
| /proxy_device_[nodeid]/nmt_state | String | Publishes NMT state on change |
| /proxy_device_[nodeid]/rpdo | COData | Publishes received PDO objects on reception |

## ROS Subscriptions

| Topic                   | Type                  | Description       |
|---------------------------|-----------------------|------------------|
| /proxy_device_[nodeid]/tpdo | COData | Writes received data to remote device if the specified object is RPDO mapped on remote device. |



