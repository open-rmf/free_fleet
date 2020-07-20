#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import \
    get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = 'burger'


def generate_launch_description():
    lcxt = LaunchContext()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('free_fleet_examples'),
            'empty_map.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_path = \
        os.path.join(
            get_package_share_directory('free_fleet_examples'),
            'maps',
            'empty_map',
            'empty_map.world')
    model_path = \
        os.path.join(
            get_package_share_directory('free_fleet_examples'),
            'maps',
            'empty_map',
            'models') + ':' + \
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models') + ':' + \
        '/usr/share/gazebo-9/models'
    resource_path = '/usr/share/gazebo-9'
    plugin_path = \
        os.path.join(get_package_prefix('building_gazebo_plugins'), 'lib')

    SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path).visit(lcxt)
    SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path).visit(lcxt)
    SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path).visit(lcxt)
    SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '').visit(lcxt)

    urdf_path = \
        os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so', world_path],
            output='both'),

        ExecuteProcess(
            cmd=['gzclient', '--verbose', world_path],
            output='both'),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[urdf_path],
            parameters=[{'use_sim_time': True}],
            output='both'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory('turtlebot3_navigation2'), 
                    '/launch/navigation2.launch.py'
                ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_path,
            }.items(),
        ),

    ])
