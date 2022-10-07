import os
from time import sleep
import pytest
from sympy import true
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from canopen_utils.test_node import TestNode
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger
import unittest


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/proxy_lifecycle_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("SetupClass")
        

    @classmethod
    def tearDownClass(cls):
        print("TearDownClass")

    @classmethod
    def setUp(self):
        print("Setup")
        rclpy.init()
        self.node = TestNode()
        self.x = threading.Thread(target=rclpy.spin, args=[self.node])
        self.x.start()

    def tearDown(self):
        print("TearDown")
        rclpy.shutdown()
        self.node.destroy_node()
        self.x.join()


    def test_01_to_active(self):
        assert self.node.checkTransition("lifecycle_manager", State.PRIMARY_STATE_UNCONFIGURED, Transition.TRANSITION_CONFIGURE), "Could not configure device manager"
        assert self.node.checkTransition("lifecycle_manager", State.PRIMARY_STATE_INACTIVE, Transition.TRANSITION_ACTIVATE), "Could not configure device manager"
    
    def test_02_nmt(self):
        assert self.node.checkTrigger("proxy_device_1", "nmt_reset_node")
        assert self.node.checkTrigger("proxy_device_2", "nmt_reset_node")
        sleep(1.0)
        
    def test_03_sdo_read(self):
        assert self.node.checkSDORead("proxy_device_1", index=0x1000, subindex=0, type=32, data=0)
        assert self.node.checkSDORead("proxy_device_2", index=0x1000, subindex=0, type=32, data=0)

    def test_04_sdo_write(self):
        assert self.node.checkSDOWrite("proxy_device_1", index=0x4000, subindex=0, type=32, data=100)
        assert self.node.checkSDOWrite("proxy_device_2", index=0x4000, subindex=0, type=32, data=100)
        assert self.node.checkSDORead("proxy_device_1", index=0x4000, subindex=0, type=32, data=100)
        assert self.node.checkSDORead("proxy_device_2", index=0x4000, subindex=0, type=32, data=100)

    def test_05_sdo_read_id(self):
        assert self.node.checkSDOReadID(node_id=2, index=0x4000, subindex=0, type=32, data=100)
        assert self.node.checkSDOReadID(node_id=3, index=0x4000, subindex=0, type=32, data=100)

    def test_06_sdo_write_id(self):
        assert self.node.checkSDOWriteID(node_id=2, index=0x4000, subindex=0, type=32, data=999)
        assert self.node.checkSDOWriteID(node_id=3, index=0x4000, subindex=0, type=32, data=999)        
        assert self.node.checkSDOReadID(node_id=2, index=0x4000, subindex=0, type=32, data=999)
        assert self.node.checkSDOReadID(node_id=3, index=0x4000, subindex=0, type=32, data=999)

    def test_07_to_unconfigured(self):
        assert self.node.checkTransition("lifecycle_manager", State.PRIMARY_STATE_ACTIVE, Transition.TRANSITION_DEACTIVATE), "Could not configure device manager"
        assert self.node.checkTransition("lifecycle_manager", State.PRIMARY_STATE_INACTIVE, Transition.TRANSITION_CLEANUP), "Could not configure device manager"
    