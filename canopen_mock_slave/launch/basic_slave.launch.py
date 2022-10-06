
import os
import sys

from sympy import true
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    node_id_arg = DeclareLaunchArgument(
      'node_id', default_value=TextSubstitution(text='2'), description="CANopen node id the mock slave shall have."
    )

    slave_config_arg = DeclareLaunchArgument(
      'slave_config', 
      default_value=TextSubstitution(text=os.path.join(path_to_test, ".." , "config" , "simple_slave.eds")), 
      description="Path to eds file to be used for the slave."
    )

    can_interface_name_arg = DeclareLaunchArgument(
      'can_interface_name', 
      default_value=TextSubstitution(text="vcan0"), 
      description="CAN interface to be used by mock slave."
    )

    node_name_arg = DeclareLaunchArgument(
      'node_name', 
      default_value=TextSubstitution(text="basic_slave_node"), 
      description="Name of the node."
    )

    slave_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("node_name"), 
        namespace="", 
        package="canopen_mock_slave", 
        output="screen", 
        executable="basic_slave_node",
        parameters=[
                {
                    "slave_config": LaunchConfiguration("slave_config"),
                    "node_id": LaunchConfiguration("node_id"),
                    "can_interface_name": LaunchConfiguration("can_interface_name")
                }
            ],
    )
    slave_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node, 
            goal_state='inactive',
            handle_once=true,
            entities=[
                launch.actions.LogInfo(
                    msg="node 'basic_slave_node' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(
                        slave_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            
        ),

    )
    slave_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                slave_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld = launch.LaunchDescription()
    ld.add_action(node_id_arg)
    ld.add_action(slave_config_arg)
    ld.add_action(can_interface_name_arg)
    ld.add_action(node_name_arg)
    ld.add_action(slave_inactive_state_handler)
    ld.add_action(slave_node)
    ld.add_action(slave_configure)
    return ld