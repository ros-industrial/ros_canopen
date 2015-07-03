^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_master
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
