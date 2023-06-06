^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Enable simplified bus.yml format (`#115 <https://github.com/ros-industrial/ros2_canopen/issues/115>`_)
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
  * Add cogen
  * Add example usage for cogen
  * Remove explicit path
  ---------
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
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
* Include rpdo/tpdo test in launch_test. (`#98 <https://github.com/ros-industrial/ros2_canopen/issues/98>`_)
  * Implement rpdo/tpdo test
  * Removed unnecessary files
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
* Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
* Merge branch 'master' into patch-2
* Merge branch 'bjsowa-master'
* Add dcf_path to bus.ymls
* Merge branch 'master' of github.com:bjsowa/ros2_canopen into bjsowa-master
* Remove references to sympy.true (`#84 <https://github.com/ros-industrial/ros2_canopen/issues/84>`_)
  Co-authored-by: James Ward <j.ward@sydney.edu.au>
* Use options section in test bus config files
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Use @BUS_CONFIG_PATH@ variable in bus configuration files
* intra_process_comms
* intra_process_comms
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
* Fix integration tests
* Integration with ros2_control
* Add device container and general changes to make things work.
* Merge branch 'canopen-system-interface' into canopen-controller
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
* Merge pull request `#33 <https://github.com/ros-industrial/ros2_canopen/issues/33>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic system interface for ros2_control
* Integrate launch files into existing launch files in canopen_mock_slaves & canopen_tests
  Merge pull request `#6 <https://github.com/ros-industrial/ros2_canopen/issues/6>`_ from ipa-cmh/canopen-system-interface
* further changes
* Reorganise test launch system
* Add tests to canopen_tests
* Add tests for lifecycle and normal operation (`#44 <https://github.com/ros-industrial/ros2_canopen/issues/44>`_)
* Update dcfgen cmake integration (`#41 <https://github.com/ros-industrial/ros2_canopen/issues/41>`_)
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
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, James Ward, Vishnuprasad Prachandabhanu
