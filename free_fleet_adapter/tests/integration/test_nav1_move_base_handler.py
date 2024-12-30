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
import unittest

from free_fleet_adapter.nav1_robot_adapter import Nav1MoveBaseHandler
import rclpy

import zenoh


class TestNav1MoveBaseHandler(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_nav1_move_base_handler')
        cls.zenoh_session = zenoh.open(zenoh.Config())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.zenoh_session.close()
        rclpy.shutdown()

    def test_move_base_statuses_do_not_exist(self):
        move_base_handler = Nav1MoveBaseHandler(
            'missing_turtlebot3_1', self.zenoh_session, self.node
        )

        statuses_exists = False
        for i in range(10):
            goal_status_array = move_base_handler.get_goal_status_array()
            if goal_status_array is not None:
                statuses_exists = True
                break
            time.sleep(1)

        assert not statuses_exists

    def test_command_status_and_cancel(self):
        move_base_handler = Nav1MoveBaseHandler(
            'tb3_0', self.zenoh_session, self.node
        )

        status_array = None
        for i in range(10):
            status_array = move_base_handler.get_goal_status_array()
            if status_array is not None:
                break
            time.sleep(1)
        assert status_array is not None
        assert len(status_array.status_list) == 0

        active_goal_status = move_base_handler.get_active_goal_status()
        assert active_goal_status is None

        # Longer timeout during testing
        goal_id = move_base_handler.navigate_to_pose(
            x=-1.6,
            y=0.5,
            z=0.0,
            yaw=0.0,
            timeout_sec=5.0
        )
        assert goal_id is not None

        status_array = None
        for i in range(10):
            status_array = move_base_handler.get_goal_status_array()
            if len(status_array.status_list) == 1 \
                and status_array.status_list[0].status == \
                    status_array.status_list[0].ACTIVE:
                break
            time.sleep(1)
        assert len(status_array.status_list) == 1

        assert status_array.status_list[0].status == \
            status_array.status_list[0].ACTIVE

        active_goal_status = move_base_handler.get_active_goal_status()
        assert active_goal_status is not None
        assert status_array.status_list[0] == active_goal_status
        assert active_goal_status.goal_id.id == goal_id

        move_base_handler.stop_navigation()
        for i in range(10):
            active_goal_status = move_base_handler.get_active_goal_status()
            if active_goal_status is None:
                break
            time.sleep(1)
        assert active_goal_status is None
