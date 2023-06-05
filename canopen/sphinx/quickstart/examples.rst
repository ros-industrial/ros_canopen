Examples
========

In order to tryout the library a few examples are provided in the ``canopen_tests`` directory.
You can run them if you have started the vcan0 interface.

Service Interface
---------------------

.. code-block:: bash

    ros2 launch canopen_tests ci402_setup.launch.py


Managed Service Interface
-------------------------

.. code-block:: bash

    ros2 launch canopen_tests ci402_lifecycle_setup.launch.py

ROS2 Control
------------

.. code-block:: bash

    ros2 launch canopen_tests robot_control_setup.launch.py
