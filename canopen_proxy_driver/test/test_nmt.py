import os
import sys
from threading import Thread
import time
from tokenize import String
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros

import lifecycle_msgs.msg
import canopen_interfaces.srv

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
            "bus_config": os.path.join(test_dir_path, ".." ,  "config/nmt_test" , "simple.yml"),
            "master_config": os.path.join(test_dir_path, ".." , "config/nmt_test" , "master.dcf"),
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
                "eds": os.path.join(test_dir_path, ".." , "config" , "slave.eds"),
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

    ready_to_test = launch.actions.TimerAction(
        period=2.0,
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

class MakeTestNode(Node):
    def __init__(self, name='test_node'):
        super().__init__(name)

    def check_service(self, name: String, timeout=3.0) -> bool:
        self.client = self.create_client(std_srvs.srv.Trigger, name)
        return self.client.wait_for_service(timeout_sec=3.0)

    def call_service(self, name: String, timeout=3.0):
        req = std_srvs.srv.Trigger.Request()
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=timeout)
        if self.future.done():
            try:
                result = self.future.result()
            except Exception as e:
                return False
            else:
                return result.success
    
    def clear_service(self):
        self.destroy_client(self.client)


class TestBasicDevice(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):

        self.node = MakeTestNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_nmt(self, launch_service, proc_output):
        
        assert self.node.check_service(name="/motioncontroller_1/nmt_reset_node", timeout=3.0), 'Could not find service'
        assert self.node.call_service(name="/motioncontroller_1/nmt_reset_node", timeout=3.0), 'Could not call service'
        self.node.clear_service()
        
