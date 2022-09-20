ros2_control SystemInterface test
=================================

Test details
------------

.. csv-table:: Tests
    :header: "Detail", "Information"
    :delim: ;

    Package; canopen_ros2_control
    Test file; launch/canopen_system.launch.py
    Description; Create an exemplary ros2_control SystemInterface with CAN master and communicates to a slave node.
    Prequisites; vcan0 must be available
    
 
Explanation of the test
------------------------

The test is starting generic system interface and generic controller for CanOpen devices.
Generic system interface enables intergration of values from the CAN Bus into ros2_control framework and the controller enables you to send and receive data from the CAN bus through ros2_control to ROS2.

The next few lines show you some command to have exemplary usage of the ros2_control integration:

1. After the test is started check ``/dynamic_joint_states`` to show internal data from CAN nodes in ros2_control system.
  
   .. code-block:: bash
   
      ros2 topic echo /dynamic_joint_states
   

2. Open a new terminal and echo data from the topic ``/node_1_controller/rpdo``.
   
   .. code-block:: bash
   
      ros2 topic echo /node_1_controller/rpdo
      
   
3. In a new terminal publish some data to the controller to write them to the CAN bus:
   
   .. code-block:: bash
      ros2 topic pub -r 100 node_1_controller/rpdo canopen_interfaces/msg/COData "
        index: 25
        subindex: 35
        data: 238
        type: 8"

   Now watch how data in the first two opened terminals are changing.


