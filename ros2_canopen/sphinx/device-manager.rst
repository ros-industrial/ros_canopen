Device Manger
=============

The device manager implements ROS2 component manager interface and can therefore be handled similar to a
container node. This enables flexibly loading and unloading the driver nodes necessary for commanding
each device on the bus. The user can decide when the device should be activated. 


.. figure:: images/device-manager-dynamic-driver-loading.png
    :width: 600 px
    :alt: Dynamic driver loading procedure
    
    ros2_canopen's driver loading procedure

Configuration
-------------

.. csv-table:: Parameters
   :header: "Parameter", "Type", "Description"
   :width: 400 px

    enable_lazy_loading, bool, Enables or disables lazy loading.
    bus_conf, string, path to the bus configuration YAML-file
    master_dcf, string, path to the DCF file to be used by the master node



 