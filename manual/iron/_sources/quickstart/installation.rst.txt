Installation
===============================
Clone ros2_canopen into your ROS2 workspace's source folder, install dependencies and
build with colcon and your done.

.. code-block:: console

   $ git clone https://github.com/ros-industrial/ros2_canopen.git
   $ cd ..
   $ rosdep install --from-paths src/ros2_canopen --ignore-src -r -y
   $ colcon build
