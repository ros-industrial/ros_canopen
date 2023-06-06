^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_402_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Add driver dictionaries (`#110 <https://github.com/ros-industrial/ros2_canopen/issues/110>`_)
  * Get slave eds and bin in node_canopen_driver
  * Add dictionary to base driver
  * Enable dictionary in proxy drivers
  * Add a few test objects
  * Add pdo checks
  * Adjust 402 driver
  * Fix tests
  * rename to get_xx_queue
  * Add typed sdo operations
  ---------
* Motor Profile Updates (`#101 <https://github.com/ros-industrial/ros2_canopen/issues/101>`_)
  * Extend and fix info statement.
  * Fix service handler overwriting.
  * Consider enum 3 as profiled velocity. Remove some code duplication by reusing private setters in service cbs. Create setter for interpolated position mode.
  * Fix cyclic position mode.
  * Simplify write method cases defined by mode of op.
* Merge pull request `#100 <https://github.com/ros-industrial/ros2_canopen/issues/100>`_ from ipa-vsp/bugfix/fix-vel-pos-scaling
  Proper scaling from the device
* proper vel and pos scaling from device
* Implemented thread-safe queue for rpdo and emcy listener (`#97 <https://github.com/ros-industrial/ros2_canopen/issues/97>`_)
  * Boost lock free queue implemetation
  * include boost libraries in CMakelists
  * Testing rpdo/tpdo ping pond
  * pre-commit changes
  * Bugfix: implemented timeout for wait_and_pop to avoid thread blocking
  * Fixed typo
  * pre-commit update
  * FIxed: properly export Boost libraries
  * Update code documentation
* Add Interpolated Position Mode (linear only, no PT or PVT) (`#90 <https://github.com/ros-industrial/ros2_canopen/issues/90>`_)
  * Add Interpolated Position Mode (linear only, no PT or PVT)
  * add interpolated position mode to system interface
  * Add interpolated position mode to controllers.
  * Add to interpolated position mode to documentation
  ---------
* Simplify 402 driver (`#89 <https://github.com/ros-industrial/ros2_canopen/issues/89>`_)
  * Split motor.hpp and motor.cpp into different files
  * Fix missing symbol error
  ---------
* Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Remove false license statements (`#76 <https://github.com/ros-industrial/ros2_canopen/issues/76>`_)
  * Remove false license statements
* Scaling factors for position and velocity (`#74 <https://github.com/ros-industrial/ros2_canopen/issues/74>`_)
  * Introduce scaling factors
* Merge branch 'livanov93-motor-profile'
* Set target based on condition.
* Expose 402 main functionalities to ros2_control system interface.
* Extend 402 functions via public methods - same as callback based ones.
* Adapt 402 hardware interface to device container getter.
* Add position and speed getter.
* Add unit tests for canopen_core (`#64 <https://github.com/ros-industrial/ros2_canopen/issues/64>`_)
  * Testing changes to canopen_core
  * Testing changes to canopen_base_driver and canopen_402_driver
  * Add canopen_core tests (90% coverage)
  * Fix DriverException error in canopen_402_driver
  * Catch errors in nmt and rpdo listeners
  * Fix naming issues
  * Fix deactivate transition
  * Fix unclean shutdown
* Publish to joint_states, not joint_state (`#63 <https://github.com/ros-industrial/ros2_canopen/issues/63>`_)
  Co-authored-by: G.A. vd. Hoorn <g.a.vanderhoorn@tudelft.nl>
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* Fix 402 issues from testing
* Streamline logging
* Fix node_canopen_402_drivers add_to_master and handlers
* Fix get speed and get position
* Feature parity for lifecycle nodes
* Add 402 driver functions for ros2_control
* Add master dcfs and remove from gitignore
* Add device container and general changes to make things work.
* canopen_402_driver adaption to new framework
* Add in code documentation for canopen_core (`#53 <https://github.com/ros-industrial/ros2_canopen/issues/53>`_)
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
* Add configuration parameter passthrough (`#52 <https://github.com/ros-industrial/ros2_canopen/issues/52>`_)
* Publish joint state instead of velocity topics (`#47 <https://github.com/ros-industrial/ros2_canopen/issues/47>`_)
  * disable loader service
  * add custom target/command and install to macro
  * publish jointstate
  * correct variable name squiggle
  * Minor changes to driver and slave
  * Update lely core library
  * Add sensor_msgs to dependencies
  * Remove artifacts
  * Remove some artifacts
* Solve Boot Error (`#49 <https://github.com/ros-industrial/ros2_canopen/issues/49>`_)
* Remove some unecessary changes.
* Remove pedantic cmake flags.
* fix ament_export_libraries (`#45 <https://github.com/ros-industrial/ros2_canopen/issues/45>`_)
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
* Fix remote object bug in canopen_402_driver. (`#40 <https://github.com/ros-industrial/ros2_canopen/issues/40>`_)
* Merge pull request `#1 <https://github.com/ros-industrial/ros2_canopen/issues/1>`_ from livanov93/canopen-system-interface
  [WIP] Add ros2_control system interface wrapper for ros2_canopen functionalities
* Remove pedantic cmake flags.
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
* Merge branch 'licenses' into 'master'
  add licenses to each package
  See merge request ipa326/ros-industrial/ros2_canopen!22
* update package descriptions
* add licenses to each package
* Merge branch 'renaming' into 'master'
  Update package names to fit ROS2 naming rules better
  See merge request ipa326/ros-industrial/ros2_canopen!21
* add missing variable for cyclic position mode
* store tests of proxy driver in canopen_proxy_driver
* rename packages to fit ROS2 conventions better
* Contributors: Borong Yuan, Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, G.A. vd. Hoorn, Lovro, Vishnuprasad Prachandabhanu, livanov93
