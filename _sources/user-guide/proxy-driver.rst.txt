Proxy Driver
===================
A proxy driver which simply forwards CANopen functionality for a specific device via ROS2 services and messages.

Services
--------

.. list-table:: 
  :widths: 30 20 50
  :header-rows: 1
  :align: left

  * - Services
    - Type
    - Description
  * - ~/nmt_reset_node  
    - Trigger
    - Resets CANopen Device the Proxy Device Node manages.
  * - ~/sdo_read 
    - CORead
    - Reads an SDO object from the specified index, subindex and datatype of the remote device. 
  * - ~/sdo_write
    - COWrite
    - Writes data to an SDO object on the specified index, subindex and datatype of the remote device.


Publishers
----------

.. list-table:: 
  :widths: 30 20 50
  :header-rows: 1
  :align: left

  * - Topic
    - Type
    - Description
  * - ~/nmt_state  
    - String
    - Publishes NMT state on change
  * - ~/rpdo 
    - COData
    - Publishes received PDO objects on reception 

Subscribers
-----------

.. list-table:: 
  :widths: 30 20 50
  :header-rows: 1

  * - Topic
    - Type
    - Description
  * - ~/tpdo  
    - COData
    - Writes received data to remote device if the specified object is RPDO mapped on remote device.

