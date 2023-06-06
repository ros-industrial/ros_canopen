^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adapt package xml
* Remove type indication from msg and srv interfaces (`#112 <https://github.com/ros-industrial/ros2_canopen/issues/112>`_)
  * Get slave eds and bin in node_canopen_driver
  * Add dictionary to base driver
  * Enable dictionary in proxy drivers
  * Add a few test objects
  * Add pdo checks
  * Adjust 402 driver
  * Fix tests
  * rename to get_xx_queue
  * Add typed sdo operations
  * Remove object datatype where possible
  ---------
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Apply suggestions from code review
* Remove pedantic cmake flags.
* Add lifecycle to service-based operation (`#34 <https://github.com/ros-industrial/ros2_canopen/issues/34>`_)
  * Add check if remote object already exists to avoid multiple objects with same target.
  * Renaming and changes to MasterNode
  * restrucutring for lifecycle support
  * changes to build
  * Add lifecycle to drivers, masters and add device manager
  * Add lifecycled operation canopen_core
  * Added non lifecycle stuff to canopen_core
  * Add lifecyle to canopen_base_driver
  * Add lifecycle to canopen_proxy_driver
  * restructured canopen_core for lifecycle support
  * restructured canopen_base_driver for lifecycle support
  * Restrucutured canopen_proxy_driver for lifecycle support
  * Restructured canopen_402_driver for lifecycle support
  * Add canopen_mock_slave add cia402 slave
  * add canopen_tests package for testing canopen stack
  * Disable linting for the moment and some foxy compat changes
  * Further changes for foxy compatability
* Apply suggestions from code review
* Remove pedantic cmake flags.
* Merge branch 'licenses' into 'master'
  add licenses to each package
  See merge request ipa326/ros-industrial/ros2_canopen!22
* rename canopen
* add licenses to each package
* Merge branch 'renaming' into 'master'
  Update package names to fit ROS2 naming rules better
  See merge request ipa326/ros-industrial/ros2_canopen!21
* rename packages to fit ROS2 conventions better
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro
