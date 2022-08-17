Alpha Tester Guide
==================

Setup
-----

Inorder to test the alpha with your CANopen device you need 
to execute the following steps.

1. Gather the EDS files for the CANopen devices that you want to test and have connected to your bus.

2. Create a configuration package following the guide (:doc:`configuration-package`), an example configuration package can be found here (https://github.com/ipa-cmmmh/trinamic_pd42_can) 

Tests
-----
The following table describes the tests that you could execute with your device
to help us check the functionalities of the package.
Currently, we recommend running the tests as super user.

.. csv-table:: Tests
    :header: "Name", "Description"
    :delim: ;

    Launching device manager (no lazy load); Run your launch script, that you created as described in the Setup section. Once the setup is done, check with ros2 node list, that device_container_node, master and all devices you specified in your bus configuration are present.
    Initialise devices; For each driver node call the init service. The driver node should now have brought the device into operational state and have executed the standard home method. Homing method needs to be set correctly, potentially set it in bus configuration file via SDO call.
    Operational modes; For each driver check that the operation modes of the device can be activated using the operation mode services exposed. Also Check if you can set a target using the target service. Set necessary parameters for movements in bus configuration via SDO.

Profiled Velocity Mode Test
++++++++++++++++++++++++++++
.. code_block:: 

    $ ros2 service call /trinamic_pd42/init std_srvs/srv/Trigger
    $ ros2 service call /trinamic_pd42/velocity_mode std_srvs/srv/Trigger
    $ ros2 service call /trinamic_pd42/target canopen_interfaces/srv/COTargetDouble "{target: [speed]}"

Profiled Position Mode Test
++++++++++++++++++++++++++++
.. code_block::

    $ ros2 service call /trinamic_pd42/init std_srvs/srv/Trigger
    $ ros2 service call /trinamic_pd42/position_mode std_srvs/srv/Trigger
    $ ros2 service call /trinamic_pd42/target canopen_interfaces/srv/COTargetDouble "{target: [position]}"


Documentation
-------------

1. If you have discovered a bug, please report it as an issue on GitHub.
2. You should also document the tests you ran even if everything works. In order to do so, please use the Alpha Test Template provided in canopen/sphinx/templates. Create a copy of the file with a good name in the folder hardware tests, add the information about your tests (you can also add entries to the table) and create a pull request with your additions.