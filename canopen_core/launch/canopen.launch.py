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

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "launch"))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg


def generate_launch_description():

    bus_conf_arg = DeclareLaunchArgument(
        "bus_config",
        default_value=TextSubstitution(text=""),
        description="Path to the bus configuration to use.",
    )

    master_conf_arg = DeclareLaunchArgument(
        "master_config",
        default_value=TextSubstitution(text=""),
        description="Path to the master configuration to use (dcf).",
    )

    master_bin_arg = DeclareLaunchArgument(
        "master_bin",
        default_value=TextSubstitution(text=""),
        description="Path to the master configuration to use (bin).",
    )

    can_interface_name_arg = DeclareLaunchArgument(
        "can_interface_name",
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface used by master and drivers.",
    )

    ld = launch.LaunchDescription()
    logging = launch.actions.GroupAction(
        actions=[
            launch.actions.LogInfo(msg=LaunchConfiguration("bus_config")),
            launch.actions.LogInfo(msg=LaunchConfiguration("master_config")),
            launch.actions.LogInfo(msg=LaunchConfiguration("master_bin")),
            launch.actions.LogInfo(msg=LaunchConfiguration("can_interface_name")),
        ]
    )
    lifecycle_device_container_node = launch_ros.actions.LifecycleNode(
        name="device_container_node",
        namespace="",
        package="canopen_core",
        output="screen",
        executable="device_container_node",
        parameters=[
            {"bus_config": LaunchConfiguration("bus_config")},
            {"master_config": LaunchConfiguration("master_config")},
            {"master_bin": LaunchConfiguration("master_bin")},
            {"can_interface_name": LaunchConfiguration("can_interface_name")},
        ],
    )

    ld.add_action(bus_conf_arg)
    ld.add_action(master_conf_arg)
    ld.add_action(master_bin_arg)
    ld.add_action(can_interface_name_arg)
    ld.add_action(logging)
    ld.add_action(lifecycle_device_container_node)

    return ld
