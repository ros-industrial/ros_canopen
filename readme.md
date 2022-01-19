# Prerquisits

Currently lelycore needs to be installed seperately as it is an automake project and not easily integrated into ROS workflow.
Probably need to create some kind of ROS package wrapper at some point.

Please follow: https://opensource.lely.com/canopen/docs/installation/

# Status
* Figured out CMAKE commands to build ros2 node with lelycore inside.
* test_node.cpp shows a lifecycle node that uses lelycore's asynchmaster. Currently only sends reset command and waits for nodes to answer. To run ressources/master.dcf has to be set as absolute path in test_node.cpp...