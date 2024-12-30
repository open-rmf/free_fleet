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

import math
import time
import unittest

from free_fleet_adapter.nav1_robot_adapter import Nav1RobotAdapter
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
from tf2_ros import Buffer

import zenoh


class TestNav2RobotAdapter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_nav2_robot_adapter')
        cls.zenoh_session = zenoh.open(zenoh.Config())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.zenoh_session.close()
        rclpy.shutdown()

    def test_non_existent_robot_pose(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='missing_turtlebot3_1',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        robot_exists = False
        for i in range(10):
            transform = robot_adapter.get_pose()
            if transform is not None:
                robot_exists = True
                break
            time.sleep(1)

        assert not robot_exists

    def test_robot_pose(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        robot_exists = False
        for i in range(10):
            transform = robot_adapter.get_pose()
            if transform is not None:
                robot_exists = True
                break
            time.sleep(1)

        assert robot_exists

    def test_robot_battery_soc(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        battery_soc = robot_adapter.get_battery_soc()
        assert math.isclose(battery_soc, 1.0)

    def test_robot_unable_to_update(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        able_to_update = False
        try:
            robot_adapter.update(
                rmf_easy.RobotState(
                    'L1',
                    [0.0, 0.0, 0.0],
                    1.0
                )
            )
            able_to_update = True
        except RuntimeError:
            able_to_update = False
        assert not able_to_update

    def test_idle_robot_navigate_is_done(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )
        assert robot_adapter._is_navigation_done()

    def test_robot_stop_without_command(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )
        assert robot_adapter.execution is None
        robot_adapter.stop(None)
        assert robot_adapter.execution is None
        assert robot_adapter._is_navigation_done()

    def test_robot_handle_navigate_to_invalid_map(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        able_to_handle_navigate = False
        try:
            robot_adapter._handle_navigate_to_pose(
                'invalid_map',
                0.0,
                1.0,
                2.0,
                0.0
            )
            able_to_handle_navigate = True
        except RuntimeError:
            able_to_handle_navigate = False
        assert not able_to_handle_navigate

    # def test_robot_stop_navigate(self):
    #     tf_buffer = Buffer()

    #     robot_adapter = Nav1RobotAdapter(
    #         name='tb3_0',
    #         configuration=None,
    #         robot_config_yaml={
    #             'initial_map': 'L1',
    #         },
    #         node=self.node,
    #         zenoh_session=self.zenoh_session,
    #         fleet_handle=None,
    #         tf_buffer=tf_buffer
    #     )

    #     robot_adapter._handle_navigate_to_pose(
    #         'L1',
    #         -1.6,
    #         -0.5,
    #         0.0,
    #         0.0,
    #         5.0
    #     )
    #     assert robot_adapter.nav_goal_id is not None
    #     assert not robot_adapter._is_navigation_done()
    #     time.sleep(1)
    #     robot_adapter._handle_stop_navigation()
    #     time.sleep(1)
    #     assert robot_adapter._is_navigation_done()

    def test_robot_execute_unknown_action(self):
        tf_buffer = Buffer()

        robot_adapter = Nav1RobotAdapter(
            name='tb3_0',
            configuration=None,
            robot_config_yaml={
                'initial_map': 'L1',
            },
            node=self.node,
            zenoh_session=self.zenoh_session,
            fleet_handle=None,
            tf_buffer=tf_buffer
        )

        able_to_execute_action = False
        try:
            robot_adapter.execute_action(
                'unknown_category',
                {},
                None
            )
            able_to_execute_action = True
        except RuntimeError:
            able_to_execute_action = False
        assert not able_to_execute_action
