^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_chain_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
