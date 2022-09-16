ros2_control SystemInterface test
=================================

Test details
------------

.. csv-table:: Tests
    :header: "Detail", "Information"
    :delim: ;

    Package; canopen_ros2_control
    Test file; launch/canopen_system.launch.py
    Description; Create an exemplary ros2_control SystemInterface with CAN master and communicates to a slave node. After start check ``/dynamic_joint_states`` to show internal data from CAN nodes in ros2_control system.
    Prequisites; vcan0 must be available
