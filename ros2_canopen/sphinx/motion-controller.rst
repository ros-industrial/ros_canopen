Motion Controller Driver
========================

The Motion Controller Driver implements the CIA402 profile and enables setting
the drive status, operation mode and sending target values to the motion controller.


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
  * - ~/init
    - Trigger
    - Initialises motion controller including referencing
  * - ~/recover
    - Trigger
    - Recovers motion controller
  * - ~/halt
    - Trigger
    - Stops motion controller
  * - ~/position_mode
    - Trigger
    - Switches to profiled position mode
  * - ~/velocity_mode
    - Trigger
    - Switches to profiled velocity mode
  * - ~/torque_mode
    - Trigger
    - Switches to profiled torque mode
  * - ~/cyclic_position_mode
    - Trigger
    - Switches to cyclic position mode
  * - ~/cyclic_velocity_mode
    - Trigger
    - Switches to cyclic velocity mode
  * - ~/target
    - CODouble
    - Sets the target value. Only accepted when an operation mode is set.

Publishers
----------


Subscribers
-----------

.. list-table:: 
  :widths: 30 20 50
  :header-rows: 1

  * - Topic
    - Type
    - Description
  * - ~/target
    - COTargetDouble
    - Sets target value.

