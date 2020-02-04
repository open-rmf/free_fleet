#! /usr/bin/env python3

# Software License Agreement (Apache License)
#
# Copyright 2019 Open Source Robotics Foundation
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    autostart = LaunchConfiguration('autostart', default='true')

    map_yaml_file_path = LaunchConfiguration(
        'map_param_file',
        default=os.path.join(
            get_package_share_directory('free_fleet_panel_ros2'),
            'launch',
            'test_maps',
            'house.yaml'))
    print('getting map yaml from: {}'.format(map_yaml_file_path))

    map_yaml_file_param = {
        'yaml_filename': map_yaml_file_path}
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    rviz_config_path = LaunchConfiguration(
        'rviz_config_path',
        default=os.path.join(
            get_package_share_directory('free_fleet_panel_ros2'),
            'launch',
            'test_maps',
            'free_fleet_panel_ros2.rviz'))
    print('getting rviz config from: {}'.format(rviz_config_path))

    server_yaml = LaunchConfiguration(
        'server_yaml',
        default=os.path.join(
            get_package_share_directory('free_fleet_server_ros2'),
            'launch',
            'turtlebot3_world_ff.yaml'))
    print('getting server parameters from: {}'.format(server_yaml))

    return LaunchDescription([

        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[map_yaml_file_param],
            remappings=remappings),

        Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'),

        Node(
            package='free_fleet_server_ros2',
            node_executable='free_fleet_server_ros2',
            parameters=[server_yaml],
            output='screen'),

        Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['map_server']}])

    ])
