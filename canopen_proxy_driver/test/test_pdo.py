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
import canopen_interfaces.srv
import canopen_interfaces.msg

import pytest

import rclpy

import std_msgs.msg
import std_srvs.srv


import rclpy
from rclpy.node import Node


@pytest.mark.rostest
def generate_test_description():
    test_dir_path = os.path.dirname(__file__)
    master_node = launch_ros.actions.Node(
        name="device_manager_node",
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="device_manager_node",
        parameters= [{
            "bus_config": os.path.join(test_dir_path, ".." ,  "config/pdo_test" , "pdo.yml"),
            "master_config": os.path.join(test_dir_path, ".." , "config/pdo_test" , "master.dcf"),
            "can_interface_name": "vcan0"}
        ],
    )


    slave_node = launch_ros.actions.LifecycleNode(
        name="slave_node", 
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="slave_node",
        parameters=[{
                "eds": os.path.join(test_dir_path, ".." , "config/pdo_test" , "pdo.eds"),
                "slave_id": 2,
                "test": "pdo_counter"}
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

    ready_to_test = launch.actions.TimerAction(
        period=10.0,
        actions=[
            launch_testing.actions.ReadyToTest()
        ],
    )

    ld = launch.LaunchDescription()

    # Bring up node and slave 
    ld.add_action(slave_inactive_state_handler)
    ld.add_action(slave_node)
    ld.add_action(slave_configure)
    ld.add_action(master_node)
    ld.add_action(ready_to_test)

    return (ld, {})


class TestBasicDevice(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):

        self.node = rclpy.create_node('test_sdo_service_client')

    def tearDown(self):
        self.node.destroy_node()


    def test_sdo(self, launch_service, proc_output):
        msgs_rx = []
        subscription = self.node.create_subscription(
            canopen_interfaces.msg.COData,
            "/motioncontroller_1/rpdo",
            lambda msg: msgs_rx.append(msg),
            10
            )

        try:
            end_time = time.time() + 2
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2)

            for msg in msgs_rx:
                self.assertGreater(msg.data, 1)

        finally:
            self.node.destroy_subscription(subscription)