Starting a Configuration Package
================================

Setup CAN Controller
--------------------

**Option 1**: Virtual CANController

.. code-block:: console

  $ sudo modprobe vcan
  $ sudo ip link add dev vcan0 type vcan
  $ sudo ip link set vcan0 txqueuelen 1000
  $ sudo ip link set up vcan0

**Option 2**: Peak CANController

.. code-block:: console

  $ sudo modprobe peak_usb
  $ sudo ip link set can0 up type can bitrate 1000000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 3**: candleLight USB-CAN Adapter

.. code-block:: console

  $ sudo modprobe gs_usb
  $ sudo ip link set can0 up type can bitrate 500000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 4**: Adapt these steps to other socketcan devices

Launch configuration
--------------------

Once the CANController is setup you can go on and launch your configuration package.

.. code-block:: console

  $ ros2 launch [package] [launchfile]
