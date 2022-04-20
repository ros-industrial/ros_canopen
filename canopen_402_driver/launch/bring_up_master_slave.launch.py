
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    launch_path = os.path.dirname(__file__)
    config_path = os.path.join(launch_path, ".." ,  "config")
    os.chdir(config_path)
    ld = launch.LaunchDescription()

    master_launch = LaunchConfiguration('master_launch')
    arg1 = DeclareLaunchArgument(
            'master_launch',
            default_value='false',
            description='Defines if master is launched')

    master_node = launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(master_launch),
        name="device_manager",
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="device_manager_node",
        parameters= [{
                    "bus_config": os.path.join(config_path, "simple_mc.yml"),
                    "master_config": os.path.join(config_path, "master.dcf"),
                    "can_interface_name": "vcan0"
                }
        ],
    )
    slave_node = launch_ros.actions.LifecycleNode(
        name="slave_node", 
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="slave_node",
        parameters=[{
                "eds": os.path.join(config_path, ".." , "config" , "technosoft.eds"),
                "slave_id": 2}
            ],
    )
    slave_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_{}' reached the 'inactive' state, 'activating'.".format(2)),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(
                        slave_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    slave_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                slave_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld.add_action(arg1)
    ld.add_action(master_node)
    ld.add_action(slave_inactive_state_handler)
    ld.add_action(slave_node)
    ld.add_action(slave_configure)
    

    return ld
