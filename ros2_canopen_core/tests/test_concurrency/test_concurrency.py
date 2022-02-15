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
    
    master_node = launch_ros.actions.LifecycleNode(
        name="canopen_master",
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="canopen_master_node",
        parameters=[{
                    "yaml_path": os.path.join(path_to_test, "concurrency.yml"),
                    "dcf_path": os.path.join(path_to_test, "concurrency.dcf"),
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
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
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

    slave_node_4 = launch_ros.actions.LifecycleNode(
        name="slave_node_4", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 4
            }
            ],
    )

    slave_4_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_4, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_4' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_4),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_4_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_4),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_node_5 = launch_ros.actions.LifecycleNode(
        name="slave_node_5", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 5
            }
            ],
    )

    slave_5_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_5, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_5' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_5),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_5_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_5),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_node_6 = launch_ros.actions.LifecycleNode(
        name="slave_node_6", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 6
            }
            ],
    )

    slave_6_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_6, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_6' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_6),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_6_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_6),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_node_7 = launch_ros.actions.LifecycleNode(
        name="slave_node_7", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 7
            }
            ],
    )

    slave_7_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_7, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_7' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_7),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_7_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_7),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slave_node_8 = launch_ros.actions.LifecycleNode(
        name="slave_node_8", 
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="test_slave",
        parameters=[
            {
                "eds": os.path.join(path_to_test, "simple.eds"),
                "slave_id": 8
            }
            ],
    )

    slave_8_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node_8, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'slave_node_8' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slave_node_8),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    slave_8_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node_8),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
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

    ld.add_action(slave_4_inactive_state_handler)
    ld.add_action(slave_node_4)
    ld.add_action(slave_4_configure)

    ld.add_action(slave_5_inactive_state_handler)
    ld.add_action(slave_node_5)
    ld.add_action(slave_5_configure)

    ld.add_action(slave_6_inactive_state_handler)
    ld.add_action(slave_node_6)
    ld.add_action(slave_6_configure)

    ld.add_action(slave_7_inactive_state_handler)
    ld.add_action(slave_node_7)
    ld.add_action(slave_7_configure)

    ld.add_action(slave_8_inactive_state_handler)
    ld.add_action(slave_node_8)
    ld.add_action(slave_8_configure)

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
        self.node2 = rclpy.create_node('tester2')
        self.node3 = rclpy.create_node('tester3')
        self.node4 = rclpy.create_node('tester4')
        self.node5 = rclpy.create_node('tester5')
        self.node6 = rclpy.create_node('tester6')
        self.node7 = rclpy.create_node('tester7')
        self.node8 = rclpy.create_node('tester8')

    def tearDown(self):
        self.node2.destroy_node()
        self.node3.destroy_node()
        self.node4.destroy_node()
        self.node5.destroy_node()
        self.node6.destroy_node()
        self.node7.destroy_node()
        self.node8.destroy_node()

    def test_sdo(self, launch_service, proc_output):
        success_vector = [True, True, False, False, False, False, False, False, False]
        duration_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mutex = Lock()
        running = True
        def sdo_write(node, id) : 
            client = node.create_client(ros2_canopen_interfaces.srv.COWrite,
            "/basic_device_{}/sdo_write".format(id))
            success_vector[id] = True
            duration = 0.0
            for i in range(1,10):
                req = ros2_canopen_interfaces.srv.COWrite.Request()
                req.data = 0
                req.index = 0x1017
                req.subindex = 0
                req.type = 16
                start = time.time()
                result = client.call(req)
                end = time.time()
                if not result.success:
                    success_vector[id] = False
                duration += (end-start)/10
                
            duration_vector[id] = duration

        def spin_executor(exec):
            mutex.acquire()
            while(running):
                mutex.release()
                exec.spin_once(1.0)
                mutex.acquire()

        exec = MultiThreadedExecutor()
        exec.add_node(self.node2)
        exec.add_node(self.node3)
        exec.add_node(self.node4)
        exec.add_node(self.node5)
        exec.add_node(self.node6)
        exec.add_node(self.node7)
        exec.add_node(self.node8)
        thread2 = Thread(target=sdo_write, args=(self.node2, 2, ))
        thread3 = Thread(target=sdo_write, args=(self.node3, 3, ))
        thread4 = Thread(target=sdo_write, args=(self.node4, 4, ))
        thread5 = Thread(target=sdo_write, args=(self.node5, 5, ))
        thread6 = Thread(target=sdo_write, args=(self.node6, 6, ))
        thread7 = Thread(target=sdo_write, args=(self.node7, 7, ))
        thread8 = Thread(target=sdo_write, args=(self.node8, 8, ))
        thread2.start()
        thread3.start()
        thread4.start()
        thread5.start()
        thread6.start()
        thread7.start()
        thread8.start()

        thread9 = Thread(target=spin_executor, args=(exec, ))
        thread9.start()
        thread2.join()
        print("thread2.join()")
        thread3.join()
        print("thread3.join()")
        thread4.join()
        print("thread4.join()")
        thread5.join()
        print("thread5.join()")
        thread6.join()
        print("thread6.join()")
        thread7.join()
        print("thread7.join()")
        thread8.join()
        print("thread8.join()")
        mutex.acquire()
        running=False
        mutex.release()
        thread9.join()

        for success in success_vector:
            self.assertTrue(success)
        
        for duration in duration_vector:
            self.assertLess(duration, 0.2)



