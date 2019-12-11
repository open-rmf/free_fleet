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

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # .yaml file for configuring the parameters
    yaml = os.path.join(
        get_package_share_directory('free_fleet_server'), 
            'launch', 'test_server.yaml')
    print('getting parameter file: {}'.format(yaml))

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='free_fleet_server', node_executable='free_fleet_server', 
            output='screen', parameters=[yaml])
        ])
