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

import pytest

import rclpy

import std_msgs.msg
import std_srvs.srv


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

    def check_read_service(self, name: String, timeout=3.0) -> bool:
        self.read_client = self.create_client(canopen_interfaces.srv.CORead, name)
        return self.read_client.wait_for_service(timeout_sec=3.0)

    def call_read_service(self, name: String, data: int, index: int, subindex: int, type: int, timeout=3.0):
        req = canopen_interfaces.srv.CORead.Request()
        req.index = index
        req.subindex = subindex
        req.type = type
        future = self.read_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        try:
            result = future.result()
        except Exception as e:
            return False
        else:
            if result.success and result.data == data:
                return True
            else:
                return False
    
    def clear_read_service(self):
        self.destroy_client(self.read_client)

    def check_write_service(self, name: String, timeout=3.0) -> bool:
        self.write_client = self.create_client(canopen_interfaces.srv.COWrite, name)
        return self.write_client.wait_for_service(timeout_sec=3.0)

    def call_write_service(self, name: String, data: int, index: int, subindex: int, type: int, timeout=3.0):
        req = canopen_interfaces.srv.COWrite.Request()
        req.data = data
        req.index = index
        req.subindex = subindex
        req.type = type
        future = self.write_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        try:
            result = future.result()
        except Exception as e:
            return False
        else:
            return result.success
    
    def clear_write_service(self):
        self.destroy_client(self.write_client)


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

    def test_sdo_read(self, launch_service, proc_output):
        
        assert self.node.check_read_service(name="/motioncontroller_1/sdo_read", timeout=3.0), 'Could not find service'
        assert self.node.call_read_service(name="/motioncontroller_1/sdo_read", data=0, index=0x1017, subindex=0, type=16, timeout=3.0), 'Could not call service'
        self.node.clear_read_service()

    def test_sdo_write(self, launch_service, proc_output):
        assert self.node.check_write_service(name="/motioncontroller_1/sdo_write", timeout=3.0), 'Could not find service'
        assert self.node.call_write_service(name="/motioncontroller_1/sdo_write", data=100, index=0x1017, subindex=0, type=16, timeout=3.0), 'Could not call service'
        self.node.clear_write_service()

    def test_sdo_read_write(self, launch_service, proc_output):
        assert self.node.check_read_service(name="/motioncontroller_1/sdo_read", timeout=3.0), 'Could not find service'
        assert self.node.check_write_service(name="/motioncontroller_1/sdo_write", timeout=3.0), 'Could not find service'

        for i in range(0, 100):
            assert self.node.call_write_service(name="/motioncontroller_1/sdo_write", data=i, index=0x1017, subindex=0, type=16, timeout=3.0), 'Could not call service'
            assert self.node.call_read_service(name="/motioncontroller_1/sdo_read", data=i, index=0x1017, subindex=0, type=16, timeout=3.0), 'Could not call service'
        
        self.node.clear_write_service()
        self.node.clear_read_service()