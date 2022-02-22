Installation
===============================
Clone ros2_canopen into your ROS2 workspace's source folder, install dependencies and
build with colcon and your done.

.. code-block:: console

   $ git clone https://gitlab.cc-asp.fraunhofer.de/ipa326/ros-industrial/ros2_canopen
   $ cd ..
   $ rosdep install --from-paths src/ros2_canopen --ignore-src -r -y
   $ colcon build
