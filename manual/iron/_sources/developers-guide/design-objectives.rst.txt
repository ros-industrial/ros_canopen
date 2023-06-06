Design Objectives
======================

The ros_canopen stack had a number of drawbacks especially when it came
to maintainablity and complexity. In order to address these drawbacks, we
have decided to redesign the ros2_canopen stack completely with the following
development goals.


.. csv-table:: Development Objectives
  :header-rows: 1
  :class: longtable
  :delim: ;
  :widths: 1 2

  Objective; Description
  Understandability and Extendability; One major drawback of ros_canopen was that actually extending it with new drivers required to understand the complex stack with its different layers.
  Robust Parallel Requests; When multiple nodes are running on the same bus, it should be possible to make requests to the nodes concurrently from ROS2 and have the canopen master handle the sequencing.
  Easy Maintenance; Maintenance effort should be reduced as much as possible. Therefore, a clean and clear code structure and documentation is needed and only funcitonalities that are not already available from other high quality open source libraries should be self implemented.
  Enable controlling drives via ros2_control; A driver for CIA402 and a ros2_control interface need to be developed.
  Enable controlling drives via ros2 ervice interface; A driver for CIA402 and a service interface need to be developed.
  Enable proxy functionalities via ros2 interface; For debugging purposes a proxy driver should be developed, which enables sending and receiving CANopen objects via a ros2 interface.
  Good enough documentation; Write documentation for using and understanding the ros2_canopen stack.
