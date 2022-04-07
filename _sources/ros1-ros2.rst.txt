Requirements
============

The ros2_canopen stack is completely redesigned mainly to simplify maintenance of the stack.
For the redesign a number of requirements was collected from ROS-Industrial partner companies.


**1. ros2_canopen shall be easy to understand and extend**

One major drawback of ros_canopen was that actually extending it with new drivers required to
understand the stack with its different layers.


**2. ros2_canopen should handle concurrent requests on the bus**

When multiple nodes are running on the same bus, it should be possible to make requests to the nodes 
concurrently from ROS2 and have the canopen master handle the sequencing.


**3. ros2_canopen should be easily maintenable**

Maintenance effort should be reduced as much as possible. Therefore, a clean and clear
code structure and documentation is needed and only funcitonalities that are not already
available from other high quality open source libraries should be self implmented.


**4. ros2_canopen should provide means to control drives**

A driver for the cia402 profile should be provided.


**5. ros2_canopen should enable controlling drives without using ros2_control**

A ros2_control interface for drives should be provided.


**6. ros2_canopen should enable writing sdo and nmt directly (proxy mode)**

An NMT and SDO interface should be provided for master and drivers.
