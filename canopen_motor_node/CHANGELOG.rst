^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_motor_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* use urdf::JointConstSharedPtr
* Contributors: Mathias Lüdtke

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------
* Decouble RobotLayer by introducing HandleLayerBase
* Split layers into different headers and compile units
* do not call handleReadread in HandleLayer::handleRecover
  this prevents a race condition, it is not needed anyway.
* protect ObjectVariables with mutex
* added test for norm function
* fix for joint limit handling
* introduced per-controller enforce_limits parameter
* implemented per-joint limits handling
* check if hardware interface matches mode
* implemented mixed-mode switching (`#197 <https://github.com/ipa-mdl/ros_canopen/issues/197>`_)
* introduced joint reference for *res_it
* Contributors: Mathias Lüdtke, Michael Stoll

0.7.0 (2016-12-13)
------------------
* multi-mode controllers are not supported (`#197 <https://github.com/ros-industrial/ros_canopen/issues/197>`_)
* Adaption to https://github.com/ros-controls/ros_control/commit/afaf9403d1daf6e7f0a93e4a06aa9695e2883632
* Contributors: Mathias Lüdtke, Michael Stoll

0.6.5 (2016-12-10)
------------------
* protect MotorChain setup with RosChain lock
* Merge pull request `#153 <https://github.com/ipa-mdl/ros_canopen/issues/153>`_ from ipa-mdl/deprecated-canswitch
  deprecated canSwitch
* fix for issue `#171 <https://github.com/ipa-mdl/ros_canopen/issues/171>`_
* Merge pull request `#168 <https://github.com/ipa-mdl/ros_canopen/issues/168>`_ from ipa-mdl/state-filters
  added filter chain for state values
* do not start driver if filter config fails
* added filter chain for state values
* log control period settings
* use update_period\_ for controll unless use_realtime_period is set true
* better initialize last_time\_
* removed canSwitch implementation, added compile-time check for prepareSwitch
* exit code for generic error should be 1, not -1
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* update package URLs
* foward commands ony if enabled in doSwitch
* moved switch implemenation to non-RT prepareSwitch
* migrated to non-const prepareSwitch
* Splitted control_node.cpp into control_node.cpp, robot_layer.cpp and robot_layer.h
* renamed chain_ros.h to ros_chain.h, fixes `#126 <https://github.com/ipa-mdl/ros_canopen/issues/126>`_
* added strictness to service call, extend error message for doSwitch fails
* stop controllers that failed switching via service call
* stop all cotnroller joints if one failed to switch
* check for ready state before controller/mode switching
* improved init bevaviour:
  * URDF is not read again (was not needed anyway=
  * register interfaces only of first init
* remove unnecessary atomic reads
* halt motor if switch failed
* Fix for switching controllers with same mode
* More expressive comments for compile-time check
* Contributors: Mathias Lüdtke, Michael Stoll

0.6.4 (2015-07-03)
------------------

0.6.3 (2015-06-30)
------------------
* added motor prefix to allocator entry
* only register limit interfaces with actual limits
* added motor_layer settings
* Migrated to ClassAllocator helper
* do not run controller manager on shutdown
* migrated to motor plug-in
* working compile-time check
* reset commands without controllers to current value
* got rid of getModeMask
* added check for old unit factors
* added closing braces in default conversion strings
* forgot var_func assignment in constructor
* ensured UnitConverter access to factory is valid during lifetime
* add unit conversion based on muparser
* dependency on muparser
* Refer to ipa320/ros_control overlay
* migrated to new hwi switch interface
* atomic joint handle pointer
* test if mode is support, add No_Mode
* enabled limit enforcing again
* removed debug output
* Fixes https://github.com/ipa320/ros_canopen/issues/81
* Enforce limits and current_state necessary for writing
* Merge remote-tracking branch 'mdl/indigo_dev' into refactor_sm
  Conflicts:
  canopen_402/include/canopen_402/canopen_402.h
  canopen_402/src/canopen_402/canopen_402.cpp
  canopen_motor_node/src/control_node.cpp
* refactored Layer mechanisms
* Fixes crash for unitialized boost pointer for ``target_vel_`` and ``target_pos_``
* MotorChain is now a template
* early check if joint is listed in URDF
* introduced 'joint' parameter (defaults to 'name')
* 'modules' was renamed to 'nodes'
* Merge branch 'indigo_dev' of https://github.com/ipa320/ros_canopen into indigo_dev
* Merge pull request `#70 <https://github.com/ros-industrial/ros_canopen/issues/70>`_ from ipa-mdl/pluginlib
  added plugin feature to socketcan_interface
* compile-time check for ros_control notifyHardwareInterface supportcompü
* added driver_plugin parameter for pluginlib look-up
* implemented threading in CANLayer
* removed SimpleLayer, migrated to Layer
* Layer::pending and Layer::halt are now virtual pure as well
* * Eliminates Internal State conflict
  * Treats exceptions inside the state machine
* keep loop running
* proper locking for hardware interface switch (might fix `#61 <https://github.com/ros-industrial/ros_canopen/issues/61>`_)
* Merge branch 'auto_scale' into indigo_dev
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
* Merge remote-tracking branch 'ipa320/indigo_dev' into indigo_dev
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_motor_node/src/control_node.cpp
* removed MasterType form template
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* added unit factor parameter parsing
* Scale factor acquired from yaml file
* Contributors: Mathias Lüdtke, thiagodefreitas

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* fixed catkin_lint errors
* added descriptions and authors
* renamed ipa_canopen_motor_control to canopen_motor_node
* Contributors: Florian Weisshardt, Mathias Lüdtke
