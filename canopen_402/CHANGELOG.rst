^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_402
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.4 (2015-07-03)
------------------

0.6.3 (2015-06-30)
------------------
* improved PP mode
* do not quickstop in fault states
* do not diable selected mode on recover, just restart it
* initialize to No_Mode
* removed some empty lines
* implemented support for mode switching in a certain 402 state, defaults to Operation_Enable
* pass LayerStatus to switchMode
* remove enableMotor, introduced switchState
* added motor_layer settings
* fixed PP mode
* explicit find for class_loader
* fail-safe setting of op_mode to No_Mode
* Improved behaviour of concurrent commands
* added alternative Transitions 7 and 10 via Quickstop
* added alternative success condition to waitForNewState
* make motor init/recover interruptable
* changed maintainer
* removed SM-based 402 implementation
* added Motor402 plugin
* added new 402 implementation
* added MotorBase
* Added validity checks
* Removed overloaded functions and makes the handle functions protected
* Removes test executable
* Removes unnecessary configure_drive and clears the set Targets
* Some position fixes
* Removed timeout
  Reduced recover trials
* Removes some logs
* Homing integrated
* handleRead, handleWrite fixes
* Merge remote-tracking branch 'mdl/indigo_dev' into refactor_sm
  Conflicts:
  canopen_402/include/canopen_402/canopen_402.h
  canopen_402/src/canopen_402/canopen_402.cpp
  canopen_motor_node/src/control_node.cpp
* Moved supported_drive_modes to ModeSpecificEntries
* * Init, Recover, Halt for SCHUNK
  * Removed sleeps from the state machine
  * Now works as reentrant states
* refactored Layer mechanisms
* Recover failure
* Merge remote-tracking branch 'mdl/indigo_dev' into refactor_sm
  Conflicts:
  canopen_402/include/canopen_402/canopen_402.h
  canopen_402/src/canopen_402/canopen_402.cpp
* Removing some unnecessary couts
* First version with Recover
  * Tested on SCHUNK LWA4D
* Initializing all modules at once
* Moving SCHUNK using the IP mode sub-state machine
* Schunk does not set operation mode via synchronized RPDO
* initialize homing_needed to false
* Working with the guard handling and some scoped_locks to prevent unwanted access
* Passing ``state_`` to motor machine
* Fixes crash for unitialized boost pointer for ``target_vel_`` and ``target_pos_``
* Thread running
* Separates the hw with the SM test
  Advance on the Mode Switching Machine
* Organizing IPMode State Machine
* Adds mode switch and a pre-version of the turnOn sequence
* Event argument passed to the Motor state machine
* Adds the internal actions
* Control_word is set from motor state machine
* Motor abstraction on higher level machine
  Some pointers organization
* * Begins with the Higher Level Machine
  * Separates the status and control from the 402 node
* Ip mode sub-machine
* Organizing the status and control machine
* do not read homing method if homing mode is not supported
* inti ``enter_mode_failure_`` to false
* require message strings for error indicators, added missing strings, added ROS logging in sync loop
* Merge pull request `#75 <https://github.com/ros-industrial/ros_canopen/issues/75>`_ from mistoll/indigo_release_candidate
  Move ip_mode_sub_mode to configureModeSpecificEntries
* Fixed tabs/spaces
* bind IP sub mode only if IP is supported
* Move ip_mode_sub_mode to configureModeSpecificEntries
* Update state_machine.h
* Ongoing changes for the state machine
* * Eliminates Internal State conflict
  * Treats exceptions inside the state machine
* Cleaning the 402.cpp file
* Test file
* Adds state machine definition
* Adds state machine simple test
* Some cleaning necessary to proceed
* Header files for isolating the 402 state machine
* Effort value
* Merge pull request `#6 <https://github.com/ros-industrial/ros_canopen/issues/6>`_ from ipa-mdl/indigo_dev
  Work-around for https://github.com/ipa320/ros_canopen/issues/62
* Merge branch 'indigo_dev' of https://github.com/ipa-mdl/ros_canopen into indigo_dev
* fixed unintialized members
* Mode Error priority
* Order issue
* Merge branch 'indigo_dev' of https://github.com/ipa-mdl/ros_canopen into indigo_dev
  Conflicts:
  canopen_motor_node/CMakeLists.txt
* Error status
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* Contributors: Florian Weisshardt, Mathias Lüdtke, Michael Stoll, Thiago de Freitas Oliveira Araujo, thiagodefreitas

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* added descriptions and authors
* renamed ipa_canopen_402 to canopen_402
* Contributors: Florian Weisshardt, Mathias Lüdtke
