Bus Configuration
=================

The ros2_canopen stack relies on configuration files that describe the bus topology.
In total three different configuration file types are used.

.. figure:: images/configuration-flow.png
    :width: 600
    :alt: Image of configuration flow
    
    ros2_canopen's configuration flow

**Device EDS or DCF file**
      For each devices that is connected to the bus the respective EDS/DCF file is needed for configuring the ros2_canopen master to communicate correctly with these devices.

**Bus configuration file**
      This is a YAML file that describes the devices connected to the CAN-Bus, their node ids and which driver can be used to controll them.

**Master DCF file**
      This file is generated from the bus configuration file using the dcfgen tool. It is used to configure the ros2_canopen master node.





