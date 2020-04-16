^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_402
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2019-11-04)
------------------
* enable rosconsole_bridge bindings
* switch to new logging macros
* Contributors: Mathias Lüdtke

0.8.1 (2019-07-14)
------------------
* Set C++ standard to c++14
* Contributors: Harsh Deshpande

0.8.0 (2018-07-11)
------------------
* handle invalid supported drive modes object
* made Mode402::registerMode a variadic template
* use std::isnan
* migrated to std::atomic
* migrated to std::unordered_map and std::unordered_set
* migrated to std pointers
* fix initialization bug in ProfiledPositionMode
* Contributors: Mathias Lüdtke

0.7.8 (2018-05-04)
------------------
* Revert "pull make_shared into namespaces"
  This reverts commit 9b2cd05df76d223647ca81917d289ca6330cdee6.
* Contributors: Mathias Lüdtke

0.7.7 (2018-05-04)
------------------
* Added state_switch_timeout parameter to motor.
* added types for all function objects
* pull make_shared into namespaces
* added types for all shared_ptrs
* migrate to new classloader headers
* address catkin_lint errors/warnings
* Contributors: Alexander Gutenkunst, Mathias Lüdtke

0.7.6 (2017-08-30)
------------------

0.7.5 (2017-05-29)
------------------

0.7.4 (2017-04-25)
------------------
* use portable boost::math::isnan
* Contributors: Mathias Lüdtke

0.7.3 (2017-04-25)
------------------

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------
* do quickstop for halt only if operation is enabled
* Contributors: Mathias Lüdtke

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* stop on internal limit only if limit was not reached before
* hardened code with the help of cppcheck
* Do not send control if it was not changed
  Otherwise invalid state machine transitions might get commanded
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* update package URLs
* added option to turn off mode monitor
* reset Fault_Reset bit in output only
* enforce rising edge on fault reset bit on init and recover
* Revert "Enforce rising edge on fault reset bit on init and recover"
* enforce rising edge on fault reset bit on init and recover
* Merge pull request `#117 <https://github.com/ipa-mdl/ros_canopen/issues/117>`_ from ipa-mdl/condvars
  Reviewed condition variables
* use boost::numeric_cast for limit checks since it handles 64bit values right
* add clamping test
* enforce target type limits
* ModeTargetHelper is now template
* readability fix
* simplified State402::waitForNewState and Motor402::switchState
* Set Halt bit unless current mode releases it
* Contributors: Mathias Lüdtke

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
