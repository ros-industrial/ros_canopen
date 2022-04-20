Trinamic PANdrive PD42-1-1270
=============================

Test setup
----------


.. csv-table:: Tests
    :header: "Equipment", "Description"
    :delim: ;
    
    Test systen; Lenovo ThinkPad
    Operating System; Ubuntu 20.04 on VirtualBox
    ROS2 Distro; ROS2 Galactic
    CANopen Interface; PeakCAN
    CANopen Devices; Single Trinamic PD42 Drive
    Stack Version; 


Tested service calls
--------------------

.. csv-table:: Tests
    :header: "Name", "Status", "Comment"
    :delim: ;

    Launching device manager (no lazy load); OK; No problems, everything works fine.
    Initialise devices; OK; Trinamic PD42 is initialised and configuration from YAML is correctly loaded
    Profiled Velocity ; OK; Works fine. Velocity can be set via target service call.
    Profiled Position; OK; Works fine. Position can be set via target service call.