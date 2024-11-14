#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import time

from free_fleet_adapter.nav2_robot_adapter import Nav2RobotAdapter
from tf2_ros import Buffer
import rclpy
from rclpy.node import Node

import zenoh


def test_robot_does_not_exist():
    rclpy.init()
    node = Node('missing_turtlebot3_1_nav2_robot_adapter_node')
    zenoh.try_init_log_from_env()
    with zenoh.open(zenoh.Config()) as session:
        tf_buffer = Buffer()

        robot_adapter = Nav2RobotAdapter(
            name='missing_turtlebot3_1',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=node,
            zenoh_session=session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        robot_exists = False
        for i in range(10):
            transform = robot_adapter.pose()
            if transform is not None:
                robot_exists = True
                break
            time.sleep(1)

        assert not robot_exists


def test_robot_exists():
    rclpy.init()
    node = Node('turtlebot3_1_nav2_robot_adapter_node')
    zenoh.try_init_log_from_env()
    with zenoh.open(zenoh.Config()) as session:
        tf_buffer = Buffer()

        robot_adapter = Nav2RobotAdapter(
            name='turtlebot3_1',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=node,
            zenoh_session=session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        robot_exists = False
        for i in range(10):
            transform = robot_adapter.pose()
            if transform is not None:
                robot_exists = True
                break
            time.sleep(1)

        # To check that it is running
        assert not robot_exists


# def test_robot_navigate():
