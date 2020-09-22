^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package socketcan_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.5 (2020-09-22)
------------------

0.8.4 (2020-08-22)
------------------
* pass settings from ROS node to SocketCANInterface
* Contributors: Mathias Lüdtke

0.8.3 (2020-05-07)
------------------
* add includes to <memory>
* Bump CMake version to avoid CMP0048 warning
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* Contributors: Mathias Lüdtke, ahcorde

0.8.2 (2019-11-04)
------------------
* fix roslint errors in socketcan_bridge
* run roslint as part of run_tests
* enable rosconsole_bridge bindings
* Contributors: Mathias Lüdtke

0.8.1 (2019-07-14)
------------------
* Added configurable queue sizes
* Set C++ standard to c++14
* implemented create\*ListenerM helpers
* Replacing FastDelegate with std::function and std::bind.
* Contributors: Harsh Deshpande, JeremyZoss, Joshua Whitley, Mathias Lüdtke, rchristopher

0.8.0 (2018-07-11)
------------------
* keep NodeHandle alive in socketcan_bridge tests
* migrated to std::function and std::bind
* migrated to std pointers
* compare can_msgs::Frame and can::Frame properly
* Contributors: Mathias Lüdtke

0.7.8 (2018-05-04)
------------------
* Revert "pull make_shared into namespaces"
  This reverts commit 9b2cd05df76d223647ca81917d289ca6330cdee6.
* Contributors: Mathias Lüdtke

0.7.7 (2018-05-04)
------------------
* pull make_shared into namespaces
* added types for all shared_ptrs
* address catkin_lint errors/warnings
* protect tests from accessing front() or back() of empty list
* added checkMaskFilter for socketcan_bridge
* remove isValid work-around
* added unit test for can id pass filter
* add CAN filter to socketcan_bridge nodes
* Contributors: Benjamin Maidel, Mathias Lüdtke

0.7.6 (2017-08-30)
------------------

0.7.5 (2017-05-29)
------------------

0.7.4 (2017-04-25)
------------------

0.7.3 (2017-04-25)
------------------

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* hamonized versions
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* Removes gtest from test dependencies.
  This dependency is covered by the rosunit dependency.
* Removes dependency on Boost, adds rosunit dependency.
  The dependency on Boost was unnecessary, rosunit is required for gtest.
* Improves StateInterface implementation of the DummyInterface.
  The doesLoopBack() method now returns the correct value. A state change is
  correctly dispatched when the init() method is called.
* Changes the exit code of the nodes if device init fails.
  Now exits with 1 if the initialization of the can device fails.
* Changes the frame field for the published messages.
  An empty frame name is more commonly used to denote that there is no frame
  associated with the message.
* Changes return type of setup() method.
  Setup() calls the CreateMsgListener and CreateStateListener, it does not attempt
  to verify if the interface is ready, which makes void more applicable.
* Cleanup, fixes and improvements in CmakeLists.
  Adds the REQUIRED COMPONENTS packages to the CATKIN_DEPENDS.
  Improves add_dependency on the messages to be built.
  Removes unnecessary FILES_MATCHING.
  Moves the roslint_cpp macro to the testing block.
* Finalizes work on the socketcan_bridge and can_msgs.
  Readies the packages socketcan_bridge and can_msgs for the merge with ros_canopen.
  Bumps the version for both packages to 0.1.0. Final cleanup in CMakeLists, added
  comments to the shell script and launchfile used for testing.
* Adds tests for socketcan_bridge and bugfixes.
  Uses rostests and the modified DummyInterface to test whether behaviour
  is correct. Prevented possible crashes when can::tostring was called on
  invalid frames.
* Adds conversion test between msg and SocketCAN Frame.
  This test only covers the conversion between the can_msgs::Frame message and can::Frame from SocketCAN.
* Introduces topic_to_socketcan and the bridge.
  Adds TopicToSocketCAN, the counterpart to the SocketCANToTopic class.
  Also introduces a node to use this class and a node which combines the two
  classes into a bidirectional bridge.
  Contrary to the previous commit message the send() method appears to be
  available from the ThreadedSocketCANInterface afterall.
* Introduces socketcan_to_topic and its node.
  This is based on the ThreadedSocketCANInterface from the socketcan_interface package. Sending might become problematic with this class however, as the send() method is not exposed through the Threading wrappers.
* Contributors: Ivor Wanders, Mathias Lüdtke
