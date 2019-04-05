# License: Apache License 2.0
# https://github.com/DynoRobotics/dyno-licence/raw/master/LICENCE_APACHE_SAM
# Author: Samuel Lindgren

import sys

import launch.actions
import launch.substitutions
from launch import LaunchDescription

import launch_ros.actions

# TODO(sam): add automatic launch tests


def generate_launch_description():
    params_file = launch.substitutions.LaunchConfiguration(
        'params',
        default=[launch.substitutions.ThisLaunchFileDir(),
                 '/chain_node_params.yaml'])

    return LaunchDescription([
        launch_ros.actions.Node(
            package='canopen_chain_node',
            node_executable='canopen_chain_node',
            output='screen',
            node_namespace='canopen_driver',
            parameters=[params_file]),
    ])
