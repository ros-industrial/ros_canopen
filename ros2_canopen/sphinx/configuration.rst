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


Creating a CANopen configuration package
========================================

Creating CANopen Configuration
------------------------------

1. **Create a configuration package**
    .. code-block:: console

      $ ros2 pkg create --build-type ament_cmake <package_name>
      $ cd <package_name>
      $ mkdir resource

2. **Gather required information**
    Gather all EDS files of the devices connected to the bus and store them
    in your configuration package. 

3. **Writing your bus.yml file** 
    First create the configuration yml file.
    .. code-block:: console

      $ touch bus.yml

    Open the file in the editor of your choice and create the master description.
    .. code-block:: 

      master:
        node_id: [node id]
    
    And add other configuration data as necessary. A documentation of configuration options
    available can be found here `here`_.

    Once you have defined the configuration of your master, add your slaves. The following
    describes the mandatory data per slave. Further configuration options can be found `here`_.

    .. code-block:: 

      [unique slave name]:
        node_id: [node id]
        package: [ros2 package where to find the driver] 
        driver: [qualified name of the driver]

.. _here: https://opensource.lely.com/canopen/docs/dcf-tools/
 

3. **Generating your master.dcf file**
    .. code-block:: console

      $ dcfgen -r -S bus.yml


Creating ROS2 Launch file
-------------------------



