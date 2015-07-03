^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package socketcan_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.4 (2015-07-03)
------------------
* added missing include, revised depends etc.


0.6.3 (2015-06-30)
------------------
* dependencies revised
* reordering fix for `#87 <https://github.com/ros-industrial/ros_canopen/issues/87>`_
* intialize structs
* tostring fixed for headers
* removed empty test
* added DummyInterface with first test
* added message string helper
* added missing include
* install socketcan_interface_plugin.xml
* migrated to class_loader for non-ROS parts
* moved ThreadedInterface to dedicated header
* removed bitrate, added loopback to DriverInterface::init
* added socketcan plugin
* CommInterstate and StateInterface are now bases of DriverInterface.
  Therefore DispatchedInterface was moved into AsioBase.
* remove debug prints
* shutdown asio driver in destructor
* proper mask shifts
* Contributors: Mathias Lüdtke

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* fixed catkin_lint errors
* added descriptions and authors
* renamed ipa_can_interface to socketcaninterface
* Contributors: Florian Weisshardt, Mathias Lüdtke
