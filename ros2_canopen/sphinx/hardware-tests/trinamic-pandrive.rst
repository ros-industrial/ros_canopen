Trinamic PANdrive PD42-1-1270
=============================

Test setup
----------

.. list-table:: 
  :widths: 50 50
  :header-rows: 1
  :align: left

  * - Equipment
    - Type
  * - PC
    - Lenovo ThinkPad T490
  * - Operating System
    - Ubuntu 20.04
  * - ROS2 Distro
    - Galactic
  * - CANopen Interface
    - PeakCan
  * - Motor
    - Trinamic PD42-1-1270
  * - ros2_canopen version 
    - devel


Tested service calls
--------------------

.. list-table:: 
  :widths: 30 20 50
  :header-rows: 1
  :align: left

  * - Tested Service Call
    - Status
    - Comment
  * - ~/nmt_reset_node  
    - OK
    - None
  * - ~/sdo_read 
    - OK
    - None
  * - ~/sdo_write
    - OK
    - None
  * - ~/init
    - OK
    - No comments
  * - ~/halt
    - OK
    - No comments
  * - ~/position_mode
    - OK
    - No comments
  * - ~/velocity_mode
    - OK
    - No comments
  * - ~/target
    - OK
    - Tested for profiled velocity and position mode.