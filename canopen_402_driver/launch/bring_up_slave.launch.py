
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

    parameter_file = launch.substitutions.LaunchConfiguration('parameter_file_path')
    parameter_file_arg = launch.actions.DeclareLaunchArgument(
        'parameter_file_path',
        default_value=''
    )
    ld = launch.LaunchDescription()

    master_node = launch_ros.actions.LifecycleNode(
        name="canopen_master",
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="canopen_master_node",
        parameters= [{
                    "yaml_path": os.path.join(path_to_test, ".." ,  "resources" , "simple_mc.yml"),
                    "dcf_path": os.path.join(path_to_test, ".." , "resources" , "simple_mc.dcf"),
                }
        ],
    )

    master_node_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=master_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'master_node' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(master_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    master_node_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(master_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_node_2 = launch_ros.actions.LifecycleNode(
        name="slave_node_2", 
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, ".." , "resources" , "technosoft.eds"),
                "slave_id": 2
            }
            ],
    )

    slave_2_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_2, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_2' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_2),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_2),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld.add_action(slave_2_inactive_state_handler)
    ld.add_action(slave_node_2)
    ld.add_action(slave_2_configure)

    return ld
