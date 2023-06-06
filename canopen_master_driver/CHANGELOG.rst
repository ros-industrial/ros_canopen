^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_master_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* undo renaming can_interface_name -> can_interface
* Streamline logging
* Fix canopen_master_driver tests
* Fix canopen_master_driver for explicit instantiation
* Feature parity for lifecycle nodes
* Add master dcfs and remove from gitignore
* Add device container and general changes to make things work.
* Add canopen_master_driver package and contents
* Contributors: Błażej Sowa, Christoph Hellmann Santos
