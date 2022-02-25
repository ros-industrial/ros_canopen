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
import ros2_canopen_interfaces.srv

import pytest

import rclpy

import std_msgs.msg
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    print(os.path.join(path_to_test, "master.dcf"))

    ld = launch.LaunchDescription()

    master_node = launch_ros.actions.Node(
        name="device_manager_node",
        namespace="",
        package="ros2_canopen_core",
        output="screen",
        executable="device_manager_node",
        parameters=[{
            "bus_config": os.path.join(path_to_test, "concurrency.yml"),
            "master_config": os.path.join(path_to_test, "concurrency.dcf"),
            "can_interface_name": "vcan0"}
        ],
    )

    for i in range(2, 7):
        print(os.path.join(path_to_test, "simle.eds"))
        slave_node = launch_ros.actions.LifecycleNode(
            name="slave_node_{}".format(i),
            namespace="",
            package="ros2_canopen_core",
            output="screen",
            executable="slave_node",
            parameters=[{
                    "eds": os.path.join(path_to_test, "simple.eds"),
                    "slave_id": i}
            ],
        )

        slave_inactive_state_handler = launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=slave_node, goal_state='inactive',
                entities=[
                    launch.actions.LogInfo(
                        msg="node 'slave_node_{}' reached the 'inactive' state, 'activating'.".format(i)),
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
        ld.add_action(slave_inactive_state_handler)
        ld.add_action(slave_node)
        ld.add_action(slave_configure)

    ready_to_test = launch.actions.TimerAction(
        period=10.0,
        actions=[
            launch_testing.actions.ReadyToTest()
        ],
    )
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
        self.nodes = []
        for i in range(2, 7):
            self.nodes.append(rclpy.create_node('tester_{}'.format(i)))

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()

    def test_sdo(self, launch_service, proc_output):
        success_vector = []
        duration_vector = []
        mutex = Lock()
        running = True

        def sdo_write(node, id):
            client = node.create_client(ros2_canopen_interfaces.srv.COWrite,
                                        "/motioncontroller_{}/sdo_write".format(id))
            success = True
            duration = 0.0
            for i in range(1, 10):
                print("Call nodeid {} #{}".format(id, i))
                req = ros2_canopen_interfaces.srv.COWrite.Request()
                req.data = 1000
                req.index = 0x1017
                req.subindex = 0
                req.type = 16
                result = client.call(req)
                if not result.success:
                    print("Call nodeid {} #{} failed with {}".format(id, i))
                    success = False
                    break
                time.sleep(0.01)

            success_vector[id-2] = success

        def spin_executor(exec):
            mutex.acquire()
            while(running):
                mutex.release()
                exec.spin_once(1.0)
                mutex.acquire()

        exec = MultiThreadedExecutor()
        threads = []
        for node in self.nodes:
            exec.add_node(node)
            success_vector.append(False)

        for i in range(2, 7):
            print("Add thread")
            t = Thread(target=sdo_write, args=(self.nodes[i-2], i, ))
            threads.append(t)

        spinner = Thread(target=spin_executor, args=(exec, ))
        print("Start Executor")
        spinner.start()

        time.sleep(2.0)

        for t in threads:
            print("Start Thread")
            t.start()

        for t in threads:
            print("Finished Thread")
            t.join()
            
        mutex.acquire()
        running = False
        mutex.release()

        print("Finished Executor")
        spinner.join()

        for success in success_vector:
            print(success)
            self.assertTrue(success)
