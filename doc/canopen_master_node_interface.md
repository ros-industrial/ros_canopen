# Interface description of CANopen Master Node
The CANopen Master node is a lifecycle node.

## Lifecycle States
| State                   |  Description       |
|---------------------------|-----------------------|
| Unconfigured | Directly after start of the node. Time to set the configuraiton paramters. |
| Configured | Node is configured. CANopen Event Loop is not running. |
| Active | CANopen Event loop is running. |

## Lifecycle Tranisitions
| State                   |  Description       |
|---------------------------|-----------------------|
| Configure | Reads configuration files and instatiates and registers specified device drivers and associated nodes. Prepares the CANopen Event Loop for activation |
| Activate | Enables the CANopen Event Loop. |
| Deactivate | Disables the CANopen Event Loop. |
| Cleanup | Deregisters and Deletes device drivers and associated nodes. Deletes CANopen Context. |
| Shutdown | TBD |

## ROS Services
| Service                   | Type                  | Description       |
|---------------------------|-----------------------|------------------|
| /canopen_master/read_sdo | COReadID | Reads an SDO object specified by Index, Subindex and Datatype of the device with the specified nodeid. |
| /canopen_master/write_sdo | COWriteID | Writes Data to an SDO object specified by Index, Subindex and Datatype on the device with the specified nodeid. |
| /canopen_master/set_heartbeat | COHeartbeatID | Sets the heartbeat of the device with the specified nodeid to the heartbeat value (ms) |
| /canopen_master/set_nmt | CONmtID | Sends the NMT command to the device with the specified nodeid |






