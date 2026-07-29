# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 pang-yann
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('livox_mid360s_gz')
    )
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))
    world = package_share / 'demos' / 'worlds' / 'mid360s_demo.sdf'
    xacro_file = package_share / 'demos' / 'urdf' / 'mid360s_demo.xacro.urdf'
    bridge_config = package_share / 'demos' / 'config' / 'mid360s_demo.yaml'
    rviz_config = package_share / 'demos' / 'config' / 'demo.rviz'
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(package_share.parent),
            ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz with the demo configuration.',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': str(bridge_config)}],
        output='screen',
    )

    robot_description = Command(['xacro', ' ', str(xacro_file)])
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'mid360s', '-topic', 'robot_description'],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_config)],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        gazebo_resource_path,
        rviz,
        gazebo,
        state_publisher,
        spawn_robot,
        bridge,
        rviz_node,
    ])
