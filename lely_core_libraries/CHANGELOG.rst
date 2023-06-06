^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lely_core_libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Bump lely_core_librries to version 2.3.2
* Improve lely compilation time
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
* Merge branch 'master' into patch-2
* Merge branch 'bjsowa-master'
* Merge branch 'master' of github.com:bjsowa/ros2_canopen into bjsowa-master
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Substitute @BUS_CONFIG_PATH@ in bus configuration file
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
* Update dcfgen cmake integration (`#41 <https://github.com/ros-industrial/ros2_canopen/issues/41>`_)
* Merge branch 'licenses' into 'master'
  add licenses to each package
  See merge request ipa326/ros-industrial/ros2_canopen!22
* update package descriptions
* add licenses to each package
* Merge branch 'renaming' into 'master'
  Update package names to fit ROS2 naming rules better
  See merge request ipa326/ros-industrial/ros2_canopen!21
* store tests of proxy driver in canopen_proxy_driver
* Merge branch 'cmh/restructured_master_and_slaves' into 'master'
  Complete Restructuring
  See merge request ipa326/ros-industrial/ros2_canopen!15
* make lely_core_library tools available
* Add automake etc.
* add autotools-dev to lelycore build dependencies
* change false dependencies
* Merge branch 'ros_canopen-merge' into 'master'
  Merge in canopen_402 from ros_canopen
  See merge request ipa326/ros-industrial/ros2_canopen!6
* Merge in canopen_402 from ros_canopen
* lely_core_libraries wrapper tested and building
* Contributors: Błażej Sowa, Christoph Hellmann Santos
