import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slave_eds_path = os.path.join(
                    get_package_share_directory("canopen_tests"), "config", "simple_lifecycle", "simple.eds"
                )
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_mock_slave"), "launch"
                ),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2", 
            "node_name": "slave_node_1",
            "slave_config": slave_eds_path,
            }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_mock_slave"), "launch"
                ),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3", 
            "node_name": "slave_node_2",
            "slave_config": slave_eds_path,
            }.items(),
    )

    print(os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "proxy_write_sdo",
                "master.dcf",
            ))
                
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
                "simple_lifecycle",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "simple_lifecycle",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    return LaunchDescription(
            [slave_node_1, slave_node_2, device_container]
        )
