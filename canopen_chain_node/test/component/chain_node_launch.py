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
# along with this program.  If not, see <https:#www.gnu.org/licenses/>.

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
