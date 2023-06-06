^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_proxy_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* Feature parity for lifecycle nodes
* Add master dcfs and remove from gitignore
* Integration with ros2_control
* Add device container and general changes to make things work.
* Update header guards
* Add canopen_proxy_driver with new framework
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
* Merge pull request `#33 <https://github.com/ros-industrial/ros2_canopen/issues/33>`_ from StoglRobotics-forks/canopen-system-interface
  Add generic system interface for ros2_control
* Merge remote-tracking branch 'livanov/fix-dependencies' into canopen-system-interface
* Remove unneccesary deps.
* Apply suggestions from code review
* Expose necessary stuff from proxy driver.
* Add nmt and rpdo callbacks.
* Start device manager in system interface.
* Enable easy testing temporarily.
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
* Merge pull request `#1 <https://github.com/ros-industrial/ros2_canopen/issues/1>`_ from livanov93/canopen-system-interface
  [WIP] Add ros2_control system interface wrapper for ros2_canopen functionalities
* Apply suggestions from code review
* Expose necessary stuff from proxy driver.
* Add nmt and rpdo callbacks.
* Enable easy testing temporarily.
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
* use proxy.yml instead simple.yml in CMakeLists.txt
* store tests of proxy driver in canopen_proxy_driver
* rename packages to fit ROS2 conventions better
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro, Vishnuprasad Prachandabhanu
