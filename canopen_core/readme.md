# ROS2 canopen {#mainpage}

Welcome to the ROS2 canopen documentation.

## About
ROS2 CANopen is being developed and maintainted by the [ROS-Industrial Consortium](rosindustrial.org). The package heavily builds on top of [Lely's canopen stack](https://opensource.lely.com/canopen/).


## Getting started
Clone the ros2_canopen repository into your workspace's source directory and build:

      git clone git@gitlab.cc-asp.fraunhofer.de:ipa326/ros-industrial/ros2_canopen.git

      cd ..

      colcon build

      . install/setup.sh

Once built, you are ready to use the package but you need to setup the can interface.


## Setting up your CANopen network configuration
In order to use the package for your CANopen network, you need to create a network configuration. We recommend the tools Lely CANopen provides for this (dcfgen). How to do this:

1. Gather the EDS files for the CANopen devices connected to your network and put them into a folder.

2. Create a yaml description of your network. Find an example below. Further options can be found and documentation of the yaml file structure can be found [here](https://opensource.lely.com/canopen/docs/dcf-tools/). ROS2 canopen extends the slave description by the tag driver, which specifies the driver to be used for the specific device. The driver is later loaded using pluginlib.

        master:
        node_id: 1

        motioncontroller_1:
        node_id: 2
        dcf: "simple.eds"
        driver: "BasicDevice"


3. Generate a master.dcf file using Lely CANopen's dcfgen tool. This outputs a master.dcf.

        dcfgen -r [path to yaml file]

4. Create a ROS parameters YAML file setting the path to your created dcf and yaml file:

        canopen_master:
          ros__parameters:
            dcf_path: [Absolute path to your master dcf]
            yaml_path: [Absolute path to your bus yaml]
            can_interface_name: [interface name]



## ROS2 canopen (preliminary)
1.

        ros2 launch bring_up_master.launch.py parameter_file_path:=[Path to your config yaml]


2. Set configuration files and can interface.
sh

        ros2 param set /canopen_master/dcf_path [path to dcf]
        ros2 param set /canopen_master/yaml_path [path to yaml]
        ros2 param set /canopen_master/can_interface_name [if_name]

3. Now you can configure the canopen_master via lifecycle management. The master will now load the configuration files and spawn the necessary driver nodes for each device on the network. You can check the new nodes with ros2 node command.

        ros2 lifecycle set /canopen_master configure
        ros2 node list


4. Configure all driver nodes with lifecycle management

5. Activate canopen_master node and then all driver nodes.

6. The system should now be running and you should be able to use the driver nodes to communicate with your devices.
