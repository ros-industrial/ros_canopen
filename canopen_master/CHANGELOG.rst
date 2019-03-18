^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_master
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.10 (2019-03-18)
-------------------
* require minimum version of class_loader and pluginlib
* Contributors: Mathias Lüdtke

0.7.9 (2018-05-23)
------------------
* provided KeyHash
  for use with unordered containers
* added c_array access functons to can::Frame
* Contributors: Mathias Lüdtke

0.7.8 (2018-05-04)
------------------
* Revert "pull make_shared into namespaces"
  This reverts commit 9b2cd05df76d223647ca81917d289ca6330cdee6.
* Contributors: Mathias Lüdtke

0.7.7 (2018-05-04)
------------------
* added types for all function objects
* pull make_shared into namespaces
* added types for all shared_ptrs
* migrate to new classloader headers
* throw bad_cast if datatype is not supported
* special handling of std::bad_cast
* address catkin_lint errors/warnings
* removed IPC/SHM based sync masters
* Contributors: Mathias Lüdtke

0.7.6 (2017-08-30)
------------------

0.7.5 (2017-05-29)
------------------
* added EMCYHandler::resetErrors
* added VectorHelper::callFunc
  generalized call templates
* Contributors: Mathias Lüdtke

0.7.4 (2017-04-25)
------------------

0.7.3 (2017-04-25)
------------------
* enforce boost::chrono-based timer
* Contributors: Mathias Lüdtke

0.7.2 (2017-03-28)
------------------
* fix: handle EMCY as error, not as warning
* Contributors: Mathias Lüdtke

0.7.1 (2017-03-20)
------------------
* refactored EMCY handling into separate layer
* print EMCY to stdout
* send node start on recover
  needed for external sync to work properly
* pass halt on error unconditionally
* added canopen_bcm_sync
* implemented ExternalMaster
* added object access services
* implemented ObjectStorage::getStringReader
* Contributors: Mathias Lüdtke

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* Merge pull request `#179 <https://github.com/ipa-mdl/ros_canopen/issues/179>`_ from ipa-mdl/mixed_case_access
  support mixed-case access strings in EDS
* decouple listener initialization from 1003 binding
* introduced THROW_WITH_KEY and ObjectDict::key_info
* added access type tests
* convert access string to lowercase
* Do not remove shared memory automatically
* hardened code with the help of cppcheck
* throw verbose exception if AccessType is missing (`#64 <https://github.com/ipa-mdl/ros_canopen/issues/64>`_)
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* canopen_master needs to depend on rosunit for gtest
* update package URLs
* fixed typo
* do not reset PDO COB-ID if it is not writable
* Do not recurse into sub-objects, handle them as simple data
* strip string before searching for $NODEID
* added NodeID/hex parser test
* do full recover if if driver is not ready
* wait for driver to be shutdown in run()
* limit SDO reader to size of 1
* do not send abort twice
* removed unnecessary sleep (added for tests only)
* catch all std exceptions in layer handlers
* migrated SDOClient to BufferedReader
* getter for LayerState
* fixed lost wake-up condition, unified SDO accessors
* minor NMT improvements
* removed cond from PDOMapper, it does not wait on empty buffer anymore
* Simple master counts nodes as well
* throw exception on read from empty buffer
* proper initialisation of PDO data from SDOs
* change sync subscription only on change
* shutdown and restart CAN layer on recover
* canopen::Exception is now based on std::runtime_error
* Merge pull request `#109 <https://github.com/ipa-mdl/ros_canopen/issues/109>`_ from ipa-mdl/shutdown-crashes
  Fix for pluginlib-related crashes on shutdown
* stop after heartbeat was disabled, do not wait for state switch
* added virtual destructor to SyncCounter
* Use getHeartbeatInterval()
* minor shutdown improvements
* removed unstable StateWaiter::wait_for
* Revert change to handleShutdown
* Heartbeat interval is uint16, not double
* Added validity check to heartbeat\_ (Some devices do not support heartbeat)
* Contributors: Florian Weisshardt, Mathias Lüdtke, Michael Stoll

0.6.4 (2015-07-03)
------------------
* added missing include, revised depends etc.

0.6.3 (2015-06-30)
------------------
* added Settings class
* added SimpleMaster
* remove boost::posix_time::milliseconds from SyncProperties
* removed support for silence_us since bus timing cannot be guaranteed
* properly handle cases where def_val == init_val
* implemented plugin-based Master allocators, defaults to LocalMaster
* moved master/synclayer base classes to canopen.h
* added support for non-continuous PDO ranges
* added has() check to object dictionary interface
* improved ObjectStorage entry interface
* verbose out_of_range exception
* improved timer: duration cast, autostart flag
* reset sync waiter number after timeout
* verbose timeout exception
* little fix im EMCY diagnostics
* string instead of mulit-char constant
* Merge branch 'hwi_switch' into muparser
* added std::string converters to ObjectDict::Key
* do not warn on profile-only errors
* added get_abs_time without parameter
* link against boost_atomic for platforms with lock-based implementation
* reset sent Reset and Reset_Com, c&p bug
* stop heartbeat after node shutdown
* protect reads of LayerState
* protect layers in VectorHelper
* protect buffer data
* set error only if generic error bit is set, otherwise just warn about it
* Fixes https://github.com/ipa320/ros_canopen/issues/81
* Update emcy.cpp
* removed debug outputs
* refactored Layer mechanisms
* simplified init
* simplified EMCY handling
* improved hearbeat handling
* do not stop master on slave timeout
* improved pending handling in complex layers
* added set_cached for object entries
* removed IPCLayer sync listener, loopback is disabled per default
* Merge branch 'dummy_interface' into indigo_dev
  Conflicts:
  canopen_master/src/objdict.cpp
* added sync silence feature
* Merge remote-tracking branch 'origin/fix32bit' into indigo_dev
* require message strings for error indicators, added missing strings, added ROS logging in sync loop
* fix ambiguous buffer access with 32bit compilers
* pad octet strings if necessary
* reset pending to layers.begin()
* enforce RPDO (device-side) transmimssion type to 1 if <=240
* introduced LayerVector to unify pending support
* introduced read_integer to enfoce hex parsing, closes `#74 <https://github.com/ros-industrial/ros_canopen/issues/74>`_
* clear layer before plugin loader is deleted
* Merge branch 'indigo_dev' of https://github.com/ipa320/ros_canopen into indigo_dev
* Merge pull request `#70 <https://github.com/ros-industrial/ros_canopen/issues/70>`_ from ipa-mdl/pluginlib
  added plugin feature to socketcan_interface
* exception-aware get functions
* removed RPDO sync timeout in favour of LayerStatus
* added message string helper
* EDS files are case-insensitive, so switching to iptree
* handle errors entries that are not in the dictionary
* sub entry number must be hex coded
* do not send initilized-only PDO data
* init entries if init value was given and default value was not
* implemented threading in CANLayer
* removed bitrate, added loopback to DriverInterface::init
* removed SimpleLayer, migrated to Layer
* Layer::pending and Layer::halt are now virtual pure as well
* schunk version of reset
* Merge branch 'elmo_console' of https://github.com/ipa-mdl/ros_canopen into dcf_overlay
* remove debug prints
* resize buffer if needed in expedited SDO upload
* fix SDO segment download
* only access EMCY errors if available
* added ObjectStorage:Entry::valid()
* added ObjectDict overlay feature
* Fixes the bus controller problems for the Elmo chain
* Work-around for Elmo SDO bug(?)
* improved PDO buffer initialization, buffer if filled per SDO if needed
* pass permission object
* disable threading interrupts while waiting for SDO response
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* Contributors: Mathias Lüdtke, Thiago de Freitas Oliveira Araujo, ipa-cob4-2, ipa-fmw, thiagodefreitas

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* added descriptions and authors
* renamed ipa_canopen_master to canopen_master
* Contributors: Florian Weisshardt, Mathias Lüdtke
