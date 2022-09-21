Creating a new device driver
============================

Creating your own device driver is fairly easy in ros2_canopen. You should do this if you
need to create a driver for a specific device or a specific device profile. If you create
a driver for a device profile we are happy to integrate the package into this repository - simply create
a PR.

Option 1: From BaseDriver
-------------------------------
Derive your driver from BaseDriver, which has callbacks for rpdo and nmt changes, but no ros services
whatsoever.

.. doxygenclass:: ros2_canopen::BaseDriver
   :project: ros2_canopen
   :protected-members:


Option 2: From ProxyDriver
--------------------------------
Derive your driver from ProxyDriver, which has services for nmt, sdo and publishers and subscribers for pdo included.

.. doxygenclass:: ros2_canopen::ProxyDriver
   :project: ros2_canopen
   :protected-members:



Option 3: From scratch
----------------------
All ros2_canopen drivers need to implement the DriverInterface. This makes them a ROS2 Node that we are
able to load using plugin lib.

.. doxygenclass:: ros2_canopen::DriverInterface
   :project: ros2_canopen
   :members:


For implementation of your driver you can use the LelyBridge Class which implements a lely-driver that
is easy to use from a ROS node. Or write a similar Class yourself, for example using lely's LoopDriver. 

.. doxygenclass:: ros2_canopen::LelyBridge
   :project: ros2_canopen
   :members:

