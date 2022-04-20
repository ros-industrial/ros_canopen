from asyncio import Future
from concurrent.futures import thread
import os
import sys
from threading import Thread, Lock
import time
import unittest
import uuid
from rclpy.executors import MultiThreadedExecutor
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros

import lifecycle_msgs.msg
import canopen_interfaces.srv

import pytest

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import std_msgs.msg
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    ld = launch.LaunchDescription()

    master_node = launch_ros.actions.Node(
        name="device_manager_node",
        namespace="",
        package="canopen_core",
        output="screen",
        executable="device_manager_node",
        parameters=[{
            "bus_config": os.path.join(path_to_test, "../config/concurrency_test","concurrency.yml"),
            "master_config": os.path.join(path_to_test, "../config/concurrency_test" ,"master.dcf"),
            "can_interface_name": "vcan0"}
        ],
    )
    node_actions = []
    for i in range(2, 8):
        slave_node = launch_ros.actions.LifecycleNode(
            name="slave_node_{}".format(i),
            namespace="",
            package="canopen_core",
            output="screen",
            executable="slave_node",
            parameters=[{
                    "eds": os.path.join(path_to_test, "../config/concurrency_test", "simple.eds"),
                    "slave_id": i}
            ],
        )
        node_actions.append(slave_node)

        slave_inactive_state_handler = launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=slave_node, goal_state='inactive',
                entities=[
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(slave_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )
                    ),
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
        if i == 2:
            ld.add_action(slave_inactive_state_handler)
            ld.add_action(slave_node)
            ld.add_action(slave_configure)
        else:
            slave_activate = launch.actions.RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=node_actions[i-3], 
                    goal_state='active',
                    entities=[
                        slave_inactive_state_handler,
                        slave_node,
                        slave_configure
                    ],
                )
            )
            ld.add_action(slave_activate)
    ready_to_test = launch.actions.TimerAction(
        period=2.0,
        actions=[
            launch_testing.actions.ReadyToTest()
        ],
    )
    master_activate = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node_actions[7-2], 
            goal_state='active',
            entities=[
                master_node,
                ready_to_test
            ],
        )
    )


    ld.add_action(master_activate)
    return (ld, {})


class MakeTestNode(Node):
    def __init__(self, name='test_node'):
        super().__init__(name)
        self.get_logger().info("Created.")
        self.cbg = MutuallyExclusiveCallbackGroup()

    def check_read_service(self, name: str, timeout=3.0) -> bool:
        self.read_client = self.create_client(canopen_interfaces.srv.CORead, name, callback_group=self.cbg)
        return self.read_client.wait_for_service(timeout_sec=3.0)

    def call_read_service(self, name: str, data: int, index: int, subindex: int, type: int, timeout=3.0):
        req = canopen_interfaces.srv.CORead.Request()
        req.index = index
        req.subindex = subindex
        req.type = type
        result = self.read_client.call(req)

        if result.success and result.data == data:
            return True
        else:
            return False
    
    def clear_read_service(self):
        self.destroy_client(self.read_client)

    def check_write_service(self, name: str, timeout=3.0) -> bool:
        self.write_client = self.create_client(canopen_interfaces.srv.COWrite, name, callback_group=self.cbg)
        return self.write_client.wait_for_service(timeout_sec=3.0)

    def call_write_service(self, name: str, data: int, index: int, subindex: int, type: int, timeout=3.0):
        req = canopen_interfaces.srv.COWrite.Request()
        req.data = data
        req.index = index
        req.subindex = subindex
        req.type = type
        result = self.write_client.call(request=req)
        return result.success
    
    def clear_write_service(self):
        self.destroy_client(self.write_client)

    def read_write_run(self, data_start: int, data_end:int, step: int, index: int, subindex: int, type: int, timeout=3.0):
        self.write_result = True
        self.read_result = True
        self.write_failures = 0
        self.read_failures = 0
        write_req = canopen_interfaces.srv.COWrite.Request()
        write_req.index = index
        write_req.subindex = subindex
        write_req.type = type

        read_req = canopen_interfaces.srv.CORead.Request()
        read_req.index = index
        read_req.subindex = subindex
        read_req.type = type

        for i in range(data_start,  data_end, step):
            write_req.data = i
            write_result = self.write_client.call(request=write_req)
            if not write_result.success:
                self.write_failures += 1
            read_result = self.read_client.call(request=read_req)
            if not read_result.success:
                self.read_failures += 1
            if read_result.data != i:
                self.read_failures += 1
            time.sleep(0.1)

        if self.read_failures > 2:
            self.read_result = False
        
        if self.write_failures > 2:
            self.write_result = False

class TestBasicDevice(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.nodes = {}
        for i in range(2, 7):
            self.nodes["{}".format(i)] = MakeTestNode('tester_{}'.format(i))

    def tearDown(self):
        for key in self.nodes.keys():
            self.nodes[key].destroy_node()

    def test_concurrent_sdo(self, launch_service, proc_output):
        exec = MultiThreadedExecutor()
        threads = []
        for key in self.nodes.keys():
            exec.add_node(self.nodes[key])

        for key in self.nodes.keys():
            assert self.nodes[key].check_read_service(name='motioncontroller_{}/sdo_read'.format(key), timeout=3.0), "Cannot check read service"
            assert self.nodes[key].check_write_service(name='motioncontroller_{}/sdo_write'.format(key), timeout=3.0), "Cannot check write service"


        spinner = Thread(target=exec.spin)
        print("Start Executor")
        spinner.start()
        threads = {}

        for key in self.nodes.keys():
            threads[key] = Thread(target=self.nodes[key].read_write_run, args=(0, 1000, 20, 0x4000, 0x0, 32, 3.0))

        for key in threads.keys():
            print("Start Thread {}".format(key))
            threads[key].start()
            time.sleep(0.01)

        for key in threads.keys():
            print("Waiting for Thread {}".format(key))
            threads[key].join()

        exec.shutdown()
        spinner.join()

        for key in self.nodes.keys():
            assert self.nodes[key].read_result, "Read Failure in node {}".format(key)
            assert self.nodes[key].write_result, "Write Failure in node {}".format(key)