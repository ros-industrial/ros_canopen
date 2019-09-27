# Copyright (c) 2016-2019, Mathias Lüdtke, Samuel Lindgren
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
