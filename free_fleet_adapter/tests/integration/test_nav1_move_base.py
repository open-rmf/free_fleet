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
            statuses = move_base_handler.get_goal_statuses()
            if statuses is not None:
                statuses_exists = True
                break
            time.sleep(1)

        assert not statuses_exists

    def test_move_base_without_navigate(self):
        move_base_handler = Nav1MoveBaseHandler(
            'tb3_0', self.zenoh_session, self.node
        )
        active_goal_status = move_base_handler.get_active_goal_status()
        assert active_goal_status is None
        assert move_base_handler.is_navigation_done()

    def test_command_status_and_cancel(self):
        move_base_handler = Nav1MoveBaseHandler(
            'tb3_0', self.zenoh_session, self.node
        )

        statuses = None
        for i in range(10):
            statuses = move_base_handler.get_goal_statuses()
            if statuses is not None:
                break
            time.sleep(1)
        assert statuses is not None
        assert len(statuses.status_list) == 0

        active_goal_status = move_base_handler.get_active_goal_status()
        assert active_goal_status is None
        assert move_base_handler.is_navigation_done()

        move_base_handler.navigate_to_pose(
            x=-1.6,
            y=0.5,
            z=0.0,
            yaw=0.0
        )

        statuses = None
        for i in range(10):
            statuses = move_base_handler.get_goal_statuses()
            if len(statuses.status_list) == 1 \
                and statuses.status_list[0].status == \
                    statuses.status_list[0].ACTIVE:
                break
            time.sleep(1)
        assert len(statuses.status_list) == 1

        assert statuses.status_list[0].status == statuses.status_list[0].ACTIVE

        active_goal_status = move_base_handler.get_active_goal_status()
        assert active_goal_status is not None
        assert statuses.status_list[0] == active_goal_status
        assert not move_base_handler.is_navigation_done()

        move_base_handler.stop_navigation()
        for i in range(10):
            active_goal_status = move_base_handler.get_active_goal_status()
            if active_goal_status is None:
                break
            time.sleep(1)
        assert active_goal_status is None
        assert move_base_handler.is_navigation_done()
