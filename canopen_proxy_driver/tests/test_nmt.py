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


@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    print(os.path.join(path_to_test, "master.dcf"))
    master_node = launch_ros.actions.LifecycleNode(
        name="canopen_master",
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="canopen_master_node",
        parameters=[{
                    "yaml_path": os.path.join(path_to_test, "simple.yml"),
                    "dcf_path": os.path.join(path_to_test, "simple.dcf"),
                }
        ],
    )


    slave_node_2 = launch_ros.actions.LifecycleNode(
        name="slave_node_2", 
        namespace="", 
        package="canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 2
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
        period=2.0,
        actions=[
            launch_testing.actions.ReadyToTest()
        ],
    )

    ld = launch.LaunchDescription()

    # Bring up node and slave 
    ld.add_action(slave_2_inactive_state_handler)
    ld.add_action(slave_node_2)
    ld.add_action(slave_2_configure)
    ld.add_action(master_node_inactive_state_handler)
    ld.add_action(master_node)
    ld.add_action(master_node_configure)
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
        
        client = self.node.create_client(canopen_interfaces.srv.COWrite,
            "/basic_device_2/sdo_write")
        
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))

        
        req = canopen_interfaces.srv.COWrite.Request()
        req.data = 100
        req.index = 0x1017
        req.subindex = 0
        req.type = 16
        future = client.call_async(req)
        
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    result = future.result()
                except Exception as e:
                    self.fail('Service call failed %r' % (e,))
                else:
                    self.assertTrue(result.success)
                break
            
        self.node.destroy_client(client)
        
        client = self.node.create_client(canopen_interfaces.srv.CORead,
            "/basic_device_2/sdo_read")
        
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))

        
        req = canopen_interfaces.srv.CORead.Request()
        req.index = 0x1017
        req.subindex = 0
        req.type = 16
        future = client.call_async(req)
        
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    result = future.result()
                except Exception as e:
                    self.fail('Service call failed %r' % (e,))
                else:
                    self.assertTrue(result.success)
                    self.assertEqual(result.data, 100)
                break
            
        self.node.destroy_client(client)

    def test_nmt(self, launch_service, proc_output):
        
        client = self.node.create_client(std_srvs.srv.Trigger,
            "/basic_device_2/nmt_reset_node")
        
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))

        
        req = std_srvs.srv.Trigger.Request()
        future = client.call_async(req)
        
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    result = future.result()
                except Exception as e:
                    self.fail('Service call failed %r' % (e,))
                else:
                    self.assertTrue(result.success)
                break
            
        self.node.destroy_client(client)
