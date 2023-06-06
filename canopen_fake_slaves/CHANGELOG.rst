^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_fake_slaves
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Remove references to sympy.true (`#84 <https://github.com/ros-industrial/ros2_canopen/issues/84>`_)
  Co-authored-by: James Ward <j.ward@sydney.edu.au>
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Clean cia402 fake shutdown (`#72 <https://github.com/ros-industrial/ros2_canopen/issues/72>`_)
  * adapt fake cia402 slave
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
* Contributors: Błażej Sowa, Christoph Hellmann Santos, James Ward, Vishnuprasad Prachandabhanu
