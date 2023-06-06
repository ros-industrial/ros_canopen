^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_ros2_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add license files
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
* Motor Profile Updates (`#101 <https://github.com/ros-industrial/ros2_canopen/issues/101>`_)
  * Extend and fix info statement.
  * Fix service handler overwriting.
  * Consider enum 3 as profiled velocity. Remove some code duplication by reusing private setters in service cbs. Create setter for interpolated position mode.
  * Fix cyclic position mode.
  * Simplify write method cases defined by mode of op.
* Add Interpolated Position Mode (linear only, no PT or PVT) (`#90 <https://github.com/ros-industrial/ros2_canopen/issues/90>`_)
  * Add Interpolated Position Mode (linear only, no PT or PVT)
  * add interpolated position mode to system interface
  * Add interpolated position mode to controllers.
  * Add to interpolated position mode to documentation
  ---------
* Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Merge branch 'livanov93-motor-profile'
* Fix proxy test.
* Fix internal launch test.
* Fix joint states scaling.
* Update runtime deps.
* Better handling of base class on_methods.
* Add services for one shot interfaces in cia402 profile.
* State and command interfaces.
* Add base function ret values first.
* Prepare cia 402 device controller.
* Fix feedback for services for proxy driver and controlller.
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* Integration with ros2_control
* Merge pull request `#54 <https://github.com/ros-industrial/ros2_canopen/issues/54>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic Controller for Canopen
* Implement review requests regarding tests.
* Introduce tests. Adapt proxy controller for easier testing.
* Add service qos specific profile.
* Correct macro names.
* Apply suggestions from code review
  Co-authored-by: Christoph Hellmann Santos <51296695+ipa-cmh@users.noreply.github.com>
* Merge pull request `#2 <https://github.com/ros-industrial/ros2_canopen/issues/2>`_ from livanov93/canopen-controller
  Canopen controller
* Add service one shot mechanisms.
* Add publishing of rpdo and nmt state.
* Expose controller plugin. Start canopen proxy controller instance in example.
* Add dummy services, rt publishers and subscribers to proxy controller.
* Add template for canopen proxy controller.
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Dr.-Ing. Denis Štogl, Lovro, livanov93
