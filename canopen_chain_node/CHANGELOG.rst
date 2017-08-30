^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_chain_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2017-08-30)
------------------

0.7.5 (2017-05-29)
------------------
* added reset_errors_before_recover option
* Contributors: Mathias Lüdtke

0.7.4 (2017-04-25)
------------------

0.7.3 (2017-04-25)
------------------

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------
* refactored EMCY handling into separate layer
* do not reset thread for recover
* properly stop run thread if init failed
* deprecation warning for SHM-based master implementations
* implemented canopen_sync_node
* wait only if sync is disabled
* added object access services
* implement level-based object logging
* added node name lookup
* Contributors: Mathias Lüdtke

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* protect MotorChain setup with RosChain lock
* added include to <boost/scoped_ptr.hpp>; solving `#177 <https://github.com/ipa-mdl/ros_canopen/issues/177>`_
* default to canopen::SimpleMaster::Allocator (`#71 <https://github.com/ipa-mdl/ros_canopen/issues/71>`_)
* exit code for generic error should be 1, not -1
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* update package URLs
* moved roslib include into source file
* renamed chain_ros.h to ros_chain.h, fixes `#126 <https://github.com/ipa-mdl/ros_canopen/issues/126>`_
* Use of catkin_EXPORTED_TARGETS
  Install target for canopen_ros_chain
* Splitted charn_ros.h into chain_ros.h and ros_chain.cpp
* Revert "stop heartbeat after stack was shutdown"
  This reverts commit de985b5e9664edbbcc4f743fff3e2a2391e1bf8f.
* improve failure handling in init service callback
* improved excetion handling in init and recover callback
* Merge pull request `#109 <https://github.com/ipa-mdl/ros_canopen/issues/109>`_ from ipa-mdl/shutdown-crashes
  Fix for pluginlib-related crashes on shutdown
* catch std::exception instead of canopen::Exception (`#110 <https://github.com/ipa-mdl/ros_canopen/issues/110>`_)
* call to detroy is not needed anymore
* added GuardedClassLoader implementation
* minor shutdown improvements
* Contributors: Mathias Lüdtke, Michael Stoll, xaedes

0.6.4 (2015-07-03)
------------------

0.6.3 (2015-06-30)
------------------
* added motor_layer settings
* remove boost::posix_time::milliseconds from SyncProperties
* removed support for silence_us since bus timing cannot be guaranteed
* implemented plugin-based Master allocators, defaults to LocalMaster
* set initialized to false explicitly if init failed
* include for std_msgs::String was missing
* Merge remote-tracking branch 'origin/std_trigger' into new_402
  Conflicts:
  canopen_chain_node/CMakeLists.txt
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
* halt explicitly on shutdown
* stop heartbeat after stack was shutdown
* migrated to Timer instead of ros::Timer to send heartbeat even after ros was shutdown
* run loop even if ros is shutdown
* improved chain shutdown behaviour
* fix for g++: proper message generation
* Merge branch 'publisher' into muparser
  Conflicts:
  canopen_motor_node/src/control_node.cpp
* added generic object publishers
* migrated to std_srvs/Trigger
* use atomic flag instead of thread pointer for synchronization
* do not run diagnostics if chain was not initalized, output warning instead
* Changes Layer Status to Warning during the service calls
* refactored Layer mechanisms
* heartbeat works now
* check XmlRpcValue types in dcf_overlay
* removed IPCLayer sync listener, loopback is disabled per default
* added simple heartbeat timer
* added sync silence feature
* parse sync properties only if sync_ms is valid
* require message strings for error indicators, added missing strings, added ROS logging in sync loop
* skip "eds_pkg" if not provided
* clear layer before plugin loader is deleted
* implemented node list as struct
* 'modules' was renamed to 'nodes'
* removed chain name
* added driver_plugin parameter for pluginlib look-up
* implemented threading in CANLayer
* removed bitrate, added loopback to DriverInterface::init
* allow dcf_overlay in defaults as well
* recursive merge of MergedXmlRpcStruct
* added dcf_overlay parameter
* Merge branch 'auto_scale' into indigo_dev
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
* Merge remote-tracking branch 'ipa320/indigo_dev' into indigo_dev
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_motor_node/src/control_node.cpp
* catch exceptions during master creation
* removed MasterType form template
* added master_type parameter
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* added MergedXmlRpcStruct as replacement for read_xmlrpc_or_praram
* Contributors: Mathias Lüdtke, thiagodefreitas

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* added descriptions and authors
* renamed ipa_canopen_chain_ros to canopen_chain_node
* Contributors: Florian Weisshardt, Mathias Lüdtke
