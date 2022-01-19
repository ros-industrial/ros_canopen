# Prerquisits

Currently lelycore needs to be installed seperately as it is an automake project and not easily integrated into ROS workflow.
Probably need to create some kind of ROS package wrapper at some point.

Please follow: https://opensource.lely.com/canopen/docs/installation/

# Status
* Figured out CMAKE commands to build ros2 node with lelycore inside.
* test_node.cpp shows a lifecycle node that uses lelycore's asynchmaster. To run it you need to :
  * setup a vcan device (see https://opensource.lely.com/canopen/docs/cmd-tutorial/). 
  * Start testnode. 
  * Set parameter "dcf_path" and point it as absolute path to "ressources/master.dcf" (`ros2 param set /test_node dcf_path [Absolute_Path]`)
  * Set parameter "yaml_path" and point it as absolute path to "ressources/bus.yaml" (`ros2 param set /test_node yaml_path [Absolute_Path]`)
  * Then `run ros2 lifecyle set configure`
  * Then `run ros2 lifecycle set activate`
  * Now run candump and you will see that test_node is sending NMT reset command.
