import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slave_eds_path = os.path.join(
                    get_package_share_directory("canopen_tests"), "config", "cia402", "cia402_slave.eds"
                )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_mock_slave"), "launch"
                ),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2", 
            "node_name": "cia402_node_1",
            "slave_config": slave_eds_path,
            }.items(),
    )
    master_bin_path = os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_lifecycle",
                "master.bin",
            )     
    if not os.path.exists(master_bin_path):
        master_bin_path = ""            
    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    return LaunchDescription(
            [slave_node_1, device_container]
        )
