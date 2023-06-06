#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("canopen_tests"),
                    "urdf/robot_controller",
                    "robot_controller.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "config/robot_control", "ros2_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "rviz", "robot_controller.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "--controller-manager", "/controller_manager"],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    slave_config = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "config/robot_control", "cia402_slave.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_2",
            "slave_config": slave_config,
        }.items(),
    )

    nodes_to_start = [
        # slave_node_2,
        # slave_node_1,
        control_node,
        joint_state_broadcaster_spawner,
        # robot_controller_spawner,
        forward_position_controller_spawner,
        robot_state_publisher_node,
        slave_node_1,
        slave_node_2,
        # rviz_node
    ]

    return LaunchDescription(nodes_to_start)
