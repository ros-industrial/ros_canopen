^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package socketcan_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.10 (2019-03-18)
-------------------
* require minimum version of class_loader and pluginlib
* Contributors: Mathias Lüdtke

0.7.9 (2018-05-23)
------------------
* introduced ROSCANOPEN_MAKE_SHARED
* added c_array access functons to can::Frame
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
* fix catkin_lint warnings in filter tests
* migrate to new classloader headers
* find and link the thread library properly
* compile also with boost >= 1.66.0
* explicitly include iostream to compile with boost >= 1.65.0
* address catkin_lint errors/warnings
* added test for FilteredFrameListener
* fix string parsers
* default to relaxed filter handling
  works for standard and extended frames
* fix string handling of extended frames
* added filter parsers
  should work for vector<unsigned int>, vector<string> and custom vector-like classes
* implemented mask and range filters for can::Frame
* Contributors: Lukas Bulwahn, Mathias Lüdtke

0.7.6 (2017-08-30)
------------------
* make can::Header/Frame::isValid() const
* Contributors: Mathias Lüdtke

0.7.5 (2017-05-29)
------------------
* fix rosdep dependency on kernel headers
* Contributors: Mathias Lüdtke

0.7.4 (2017-04-25)
------------------

0.7.3 (2017-04-25)
------------------

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------
* stop CAN driver on read errors as well
* expose socketcan handle
* implemented BCMsocket
* introduced BufferedReader::readUntil
* Contributors: Mathias Lüdtke

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* removed Baseclass typedef since its use prevented virtual functions calls
* add missing chrono dependency
* Added catch-all features in BufferedReader
* hardened code with the help of cppcheck
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* update package URLs
* Improves StateInterface implementation of the DummyInterface.
  The doesLoopBack() method now returns the correct value. A state change is
  correctly dispatched when the init() method is called.
* Changes inheritance of DummyInterface to DriverInterface.
  Such that this interface can also be used for tests requiring a DriverInterface
  class.
  Test results of the socketcan_interface tests are unchanged by this
  modification as it only uses the CommInterface methods.
* added socketcan_interface_string to test
* moved string functions into separate lib
* Introduced setNotReady, prevent enqueue() to switch from closed to open
* Reading state\_ should be protected by lock
* improved BufferedReader interface and ScopedEnabler
* added flush() and max length support to BufferedReader
* added BufferedReader
* wake multiple waiting threads if needed
* pad hex buffer strings in all cases
* removed unstable StateWaiter::wait_for
* Contributors: Ivor Wanders, Mathias Lüdtke, Michael Stoll

0.6.4 (2015-07-03)
------------------
* added missing include, revised depends etc.


0.6.3 (2015-06-30)
------------------
* dependencies revised
* reordering fix for `#87 <https://github.com/ros-industrial/ros_canopen/issues/87>`_
* intialize structs
* tostring fixed for headers
* removed empty test
* added DummyInterface with first test
* added message string helper
* added missing include
* install socketcan_interface_plugin.xml
* migrated to class_loader for non-ROS parts
* moved ThreadedInterface to dedicated header
* removed bitrate, added loopback to DriverInterface::init
* added socketcan plugin
* CommInterstate and StateInterface are now bases of DriverInterface.
  Therefore DispatchedInterface was moved into AsioBase.
* remove debug prints
* shutdown asio driver in destructor
* proper mask shifts
* Contributors: Mathias Lüdtke

0.6.2 (2014-12-18)
------------------

0.6.1 (2014-12-15)
------------------
* remove ipa_* and IPA_* prefixes
* fixed catkin_lint errors
* added descriptions and authors
* renamed ipa_can_interface to socketcaninterface
* Contributors: Florian Weisshardt, Mathias Lüdtke
