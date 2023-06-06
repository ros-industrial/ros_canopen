^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Remove scalers
* Fix internal launch test.
* Add virtual can example for cia 402.
* Fix joint states scaling.
* Update runtime deps.
* State and command interfaces.
* Fix feedback for services for proxy driver and controlller.
* Handle init, recover, halt. Switch modes.
* Set target based on condition.
* Duplicate some code for configure, init, write phase from proxy driver.
* Add basic read and write. Divide targets into position, velocity, effort interfaces.
* Update proxy canopen system.
* Add vel and pos interfaves.
* Expose 402 main functionalities to ros2_control system interface.
* Prepare read/write/
* Adapt 402 hardware interface to device container getter.
* Fix public fcn visibility.
* To protected members for easier inheritance policy.
* Update dependencies.
* State and command interfaces.
* Add position and speed getter.
* Add bare-bone 402 profile system interface.
* Rename canopen_mock_slave package to canopen_fake_slaves (`#66 <https://github.com/ros-industrial/ros2_canopen/issues/66>`_)
  * Testing changes to canopen_core
  * Testing changes to canopen_base_driver and canopen_402_driver
  * Add canopen_core tests (90% coverage)
  * Fix DriverException error in canopen_402_driver
  * Catch errors in nmt and rpdo listeners
  * Fix naming issues
  * Fix deactivate transition
  * Fix unclean shutdown
  * Rename canopen_mock_slave to canopen_fake_slaves
  * Build flage CANOPEN_ENABLED for disabling tests on CI.
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* undo renaming can_interface_name -> can_interface
* Undo formatting in ros2_control
* Integration with ros2_control
* Merge pull request `#54 <https://github.com/ros-industrial/ros2_canopen/issues/54>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic Controller for Canopen
* Apply suggestions from code review
  Co-authored-by: Christoph Hellmann Santos <51296695+ipa-cmh@users.noreply.github.com>
* Merge pull request `#2 <https://github.com/ros-industrial/ros2_canopen/issues/2>`_ from livanov93/canopen-controller
  Canopen controller
* Merge branch 'canopen-system-interface' into canopen-controller
* Merge pull request `#33 <https://github.com/ros-industrial/ros2_canopen/issues/33>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic system interface for ros2_control
* Integrate launch files into existing launch files in canopen_mock_slaves & canopen_tests
  Merge pull request `#6 <https://github.com/ros-industrial/ros2_canopen/issues/6>`_ from ipa-cmh/canopen-system-interface
* Adapt canopen_system.launch.py for 2 nodes
* Remove bus.yml
* further changes
* update package.xml
* Reorganise test launch system
* Apply suggestions from code review
* Add service one shot mechanisms.
* Expose controller plugin. Start canopen proxy controller instance in example.
* Simplify configuration folder and use existing .eds, .dcf file. Improve test launch file. Update runtime deps.
* Disable test because they can not be eaisly tested.
* Update visibility-control macros.
* Merge remote-tracking branch 'livanov/from-init-to-configure' into canopen-system-interface
* Merge remote-tracking branch 'livanov/fix-dependencies' into canopen-system-interface
* Rename "device manager" to "device container" and disable test because it is now working in the current setup.
* Remove unneccesary deps.
* Add missig dependencies and execute tests only when testing.
* Apply suggestions from code review
* Add internal caching structures for canopen nodes.
* Add nmt and rpdo callbacks.
* Start device manager in system interface.
* Add device manager and executor.
* Print config paths on init.
* Enable easy testing temporarily.
* Introduce canopen system interface.
* Move device manager instantation into on_config.
* Merge pull request `#4 <https://github.com/ros-industrial/ros2_canopen/issues/4>`_ from livanov93/fix-dependencies
  [canopen_ros2_control] Dependency fix
* Fix dependencies for canopen_ros2_control.
* Merge pull request `#1 <https://github.com/ros-industrial/ros2_canopen/issues/1>`_ from livanov93/canopen-system-interface
  [WIP] Add ros2_control system interface wrapper for ros2_canopen functionalities
* Apply suggestions from code review
* Add internal caching structures for canopen nodes.
* Add nmt and rpdo callbacks.
* Start device manager in system interface.
* Add device manager and executor.
* Print config paths on init.
* Enable easy testing temporarily.
* Introduce canopen system interface.
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro, livanov93
