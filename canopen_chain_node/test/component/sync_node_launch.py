# License: Apache License 2.0
# https://github.com/DynoRobotics/dyno-licence/raw/master/LICENCE_APACHE_SAM
# Author: Samuel Lindgren

import sys

import launch.actions
import launch.substitutions
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    params_file = launch.substitutions.LaunchConfiguration(
        'params',
        default=[launch.substitutions.ThisLaunchFileDir(), '/sync_node_params.yaml'])

    return LaunchDescription([
        launch_ros.actions.Node(
            package='canopen_chain_node',
            node_executable='canopen_sync_node',
            output='screen',
            parameters=[params_file]),
    ])
