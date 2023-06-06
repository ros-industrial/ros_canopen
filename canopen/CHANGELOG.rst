^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adapt package xml
* Reduce processor load (`#111 <https://github.com/ros-industrial/ros2_canopen/issues/111>`_)
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
  * Add plain operation mode setting + switchingstate
  * Add robot system interface
  * Add robot system controller
  * Add robot_system_tests
  * Add a bit of documentation
  * Add in code documentation
  * Fix bug
  * Add examples section
  * Fix set_target for interpolated mode
  * Switch to rclcpp::sleep_for
  * Fix initialization for state and command interface variables
  * Add remade robot system interfce
  * Add copyright info
  * Fix missing return statement
  * processing behavior improvement
  * Minor changes to make things work
  * Add poll_timer_callback
  * Fix format
  * Add polling mode variable for config.
  ---------
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
* Correct repo link (`#94 <https://github.com/ros-industrial/ros2_canopen/issues/94>`_)
* Add Interpolated Position Mode (linear only, no PT or PVT) (`#90 <https://github.com/ros-industrial/ros2_canopen/issues/90>`_)
  * Add Interpolated Position Mode (linear only, no PT or PVT)
  * add interpolated position mode to system interface
  * Add interpolated position mode to controllers.
  * Add to interpolated position mode to documentation
  ---------
* Merge branch 'destogl-patch-2'
* Format updates
* Merge branch 'master' into patch-2
* Merge branch 'bjsowa-master'
* add short documentation
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Documentation for streamlined design (`#67 <https://github.com/ros-industrial/ros2_canopen/issues/67>`_)
  * Add canopen_core tests (90% coverage)
  * Restructure and add plantuml
  * Changes to quickstart/configuration
  * Revert "Add canopen_core tests (90% coverage)" as it is not needed.
  This reverts commit 771c498347f190777fb28edfd5044b96cbfd7bf0.
  * Create custom driver documentation
  * Remove breathe api reference and use doxygen
  * Update interface and naming information for drivers
  * Update  test documentation
* Update deployment
* undo renaming can_interface_name -> can_interface
* Integration with ros2_control
* Restructure documentation (`#37 <https://github.com/ros-industrial/ros2_canopen/issues/37>`_)
  * Document device container node
  * Document lely_master_bridge
  * Document Lifecycle Device Container
  * Document Lifecycle Device Manager
  * Document LifecyleMasterNode
  * Document Master Node
  * Fix error
  * Document lifecycle base driver
  * Document lely bridge
  * Document canopen_proxy_driver
  * Document canopen_402_driver
  * Update sphinx documentation
* make documentation on test with ros2_control more detailed
  Make the test documentation a small example with explanations of the functionality.
* Merge branch 'canopen-system-interface' into canopen-controller
* Merge pull request `#33 <https://github.com/ros-industrial/ros2_canopen/issues/33>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic system interface for ros2_control
* Add documentation about testing ros2_control generic interface.
* Update dcfgen cmake integration (`#41 <https://github.com/ros-industrial/ros2_canopen/issues/41>`_)
* Update configuration-package.rst
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
* Update running-configuration-package.rst (`#29 <https://github.com/ros-industrial/ros2_canopen/issues/29>`_)
  Add setup steps for candleLight USB-CAN adapter
* Add information about root rights (`#28 <https://github.com/ros-industrial/ros2_canopen/issues/28>`_)
  * add information about root rights
  * Add information about running a configuration pkg
* Add configuration (`#17 <https://github.com/ros-industrial/ros2_canopen/issues/17>`_)
* Further additions to documentation (`#16 <https://github.com/ros-industrial/ros2_canopen/issues/16>`_)
  * change section titles in configuration package.rst
  * add system-interface graphic and other change to configure-package.rst
  * add description to Proxy Driver
  * Add ros2_canopen logo
* Configuration manager integration (`#14 <https://github.com/ros-industrial/ros2_canopen/issues/14>`_)
  * Add longer startup delay and test documentation
  * Add speed and position publisher
  * Create Configuration Manager
  * make MasterNode a component and add configuration manager functionalities
  * add configuration manager functionalities
  * add configuration manger functionalities
  * Add documentation for Configuration Manager
  * add info messages and documentation
  * update launch files and configuration fiels
  * add can_utils package
  * add info text
  * simplify dependencies
  * remove tests from can_utils
  * avoid tests for canopen_utils
  * changes info logging and adds nmt and sdo tests
  * add tests
  * remove launch_tests from cmake
* Update configuration package tutorial (`#5 <https://github.com/ros-industrial/ros2_canopen/issues/5>`_)
  * Update configuration package tutorial
  * improve bus configuration, development objectives and other documentation and add css for tables
  * Reviewed Configuration Package Guide
  * Add alpha test description
  * Add test decriptions
  * Update device manager description with new graphics
* Update documentation
* Merge branch 'licenses' into 'master'
  add licenses to each package
  See merge request ipa326/ros-industrial/ros2_canopen!22
* rename ros2_canopen package to canopen
* add licenses to each package
* Merge branch 'renaming' into 'master'
  Update package names to fit ROS2 naming rules better
  See merge request ipa326/ros-industrial/ros2_canopen!21
* rename packages to fit ROS2 conventions better
* Contributors: Borong Yuan, Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Vishnuprasad Prachandabhanu
