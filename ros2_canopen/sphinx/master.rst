Master Node
===========

The CANopen Master node is a lifecycle node.

Services
--------

.. list-table::
  :widths: 30 20 50
  :header-rows: 1

  * - Services
    - Type
    - Description
  * - /canopen_master/read_sdo  
    - COReadID
    - Reads an SDO object specified by Index, Subindex and Datatype of the device with the specified nodeid.
  * - /canopen_master/write_sdo 
    - COWriteID
    - Writes Data to an SDO object specified by Index, Subindex and Datatype on the device with the specified nodeid.
  * - /canopen_master/set_heartbeat
    - COHeartbeatID
    - Sets the heartbeat of the device with the specified nodeid to the heartbeat value (ms)
  * - /canopen_master/set_nmt 
    - CONmtID
    - Sends the NMT command to the device with the specified nodeid