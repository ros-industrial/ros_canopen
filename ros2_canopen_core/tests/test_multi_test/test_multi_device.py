import os
import sys
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros

import lifecycle_msgs.msg
import ros2_canopen_interfaces.srv

import pytest

import rclpy

import std_msgs.msg
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    print(os.path.join(path_to_test, "master.dcf"))
    master_node = launch_ros.actions.LifecycleNode(
        name="canopen_master",
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="canopen_master_node",
        parameters=[{
                    "yaml_path": os.path.join(path_to_test, "multi_test.yml"),
                    "dcf_path": os.path.join(path_to_test, "multi_test.dcf"),
                }
        ],
    )


    slave_node_2 = launch_ros.actions.LifecycleNode(
        name="slave_node_2", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "technosoft.eds"),
                "slave_id": 2
            }
            ],
    )

    slave_node_3 = launch_ros.actions.LifecycleNode(
        name="slave_node_3", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 3
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

    slave_3_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_3, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_3' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_3),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_3_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_3),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_2_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_2),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    master_node_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(master_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=3.0,
        actions=[
            launch_testing.actions.ReadyToTest()
        ],
    )

    ld = launch.LaunchDescription()

    # Bring up node and slave 
    ld.add_action(slave_2_inactive_state_handler)
    ld.add_action(slave_node_2)
    ld.add_action(slave_2_configure)
    ld.add_action(slave_3_inactive_state_handler)
    ld.add_action(slave_node_3)
    ld.add_action(slave_3_configure)
    ld.add_action(master_node_inactive_state_handler)
    ld.add_action(master_node)
    ld.add_action(master_node_configure)
    ld.add_action(ready_to_test)

    return (ld, {})

