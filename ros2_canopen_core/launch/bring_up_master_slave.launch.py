
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    ld = launch.LaunchDescription()

    master_node = launch_ros.actions.Node(
        name="device_manager_node",
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="device_manager_node",
        parameters= [{
            "dcf_config": os.path.join(path_to_test, ".." ,  "resources" , "proxy.yml"),
            "dcf_txt": os.path.join(path_to_test, ".." , "resources" , "master.dcf"),
            "can_interface_name": "vcan0"}
        ],
    )

    slave_node = launch_ros.actions.Node(
        name="slave_node", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="slave_node",
        parameters=[{
                "eds": os.path.join(path_to_test, ".." , "resources" , "slave.eds"),
                "slave_id": 2}
            ],
    )

    ld.add_action(slave_node)
    ld.add_action(master_node)

    return ld
