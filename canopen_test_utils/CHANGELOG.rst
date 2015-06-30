^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_test_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2015-06-30)
------------------
* added missing dependency on cob_description
* added remarks for all configuration options
* depend on xacro was missing
* remove boost::posix_time::milliseconds from SyncProperties
* removed support for silence_us since bus timing cannot be guaranteed
* implemented plugin-based Master allocators, defaults to LocalMaster
* removed SM-based 402 implementation
* added timestamp to Elmo mapping
* added generic object publishers
* Removed overloaded functions and makes the handle functions protected
* Removes some logs
* Homing integrated
* example config to prevent homing
* c&p bug
* switch to new format, added heartbeat configuration
* use ring-buffer for IP mode
* removed target_interpolated_velocity from PDO mapping
* minor fixes for schunk dcf
* drive trajectory in IP mode
* bug fix in readable.py
* Schunk does not set operation mode via synchronized RPDO
* Merge remote-tracking branch 'mdl/indigo_dev' into refactor_sm
  Conflicts:
  canopen_402/include/canopen_402/canopen_402.h
  canopen_402/src/canopen_402/canopen_402.cpp
  canopen_motor_node/src/control_node.cpp
* Separates the hw with the SM test
  Advance on the Mode Switching Machine
* added sync silence feature
* simplified elmo_console
* Merge branch 'new_mapping' into indigo_dev
  Conflicts:
  canopen_test_utils/config/Elmo.dcf
* improved socketcan restart in prepare.sh
* removed 0x6081 (profile_velocity) from PDO mapping and added 0x6060 (op_mode)
* remove unused PDO map entries
* readable script with mapping loader
* new mapping scripts with PDO dictionaries
* corrected IP period, added 0x1014
* improved prepare script
* implemented threading in CANLayer
* moved ThreadedInterface to dedicated header
* removed bitrate, added loopback to DriverInterface::init
* * Eliminates Internal State conflict
  * Treats exceptions inside the state machine
* added canopen_elmo_console
* added dcf_overlay parameter
* updated joint configurations for new script server
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* example config for unit factors
* add install tags
* Contributors: Florian Weisshardt, Mathias Lüdtke, thiagodefreitas

0.6.2 (2014-12-18)
------------------
* add dep
* Contributors: Florian Weisshardt

0.6.1 (2014-12-15)
------------------
* rename node
* remove ipa_* and IPA_* prefixes
* added descriptions and authors
* renamed ipa_canopen_test to canopen_test_utils
* Contributors: Florian Weisshardt, Mathias Lüdtke
