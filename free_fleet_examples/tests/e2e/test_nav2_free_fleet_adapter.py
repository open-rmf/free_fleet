#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from managed_process import managed_process
import rclpy
from rmf_fleet_msgs.msg import FleetState
from ros_testcase import RosTestCase


class RobotExistsTest(RosTestCase):

    @RosTestCase.timeout(60)
    async def asyncSetUp(self):
        self.rmf_common_proc = managed_process(
            (
                'ros2',
                'launch',
                'free_fleet_examples',
                'turtlebot3_world_rmf_common.launch.xml'
            ),
        )
        self.rmf_common_proc.__enter__()

        self.free_fleet_adapter_proc = managed_process(
            (
                'ros2',
                'launch',
                'free_fleet_examples',
                'nav2_tb3_simulation_fleet_adapter.launch.xml'
            ),
        )
        self.free_fleet_adapter_proc.__enter__()

        self.task_proc = managed_process(
            (
                'ros2',
                'run',
                'rmf_demos_tasks',
                'dispatch_patrol',
                '-p',
                'north_west',
                '-st',
                '0',
            ),
        )

        self.custom_action_proc = managed_process(
            (
                'ros2',
                'run',
                'rmf_demos_tasks',
                'dispatch_action',
                '-a',
                'delayed_hello_world',
                '-st',
                '0',
            ),
        )

        # give some time for discovery to happen
        await self.ros_sleep(5)

    def tearDown(self):
        self.task_proc.__exit__(None, None, None)
        self.free_fleet_adapter_proc.__exit__(None, None, None)
        self.rmf_common_proc.__exit__(None, None, None)

    @RosTestCase.timeout(120)  # 2min
    async def test_patrol_task_and_action(self):
        robot_exists = rclpy.Future()

        def fleet_states_check_robot_exists_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            if len(fleet_state.robots) == 1 and \
                    fleet_state.robots[0].name == 'nav2_tb3':
                robot_exists.set_result('found nav2_tb3')

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_check_robot_exists_cb, 10
        )
        result = await robot_exists
        self.assertIsNotNone(result)

        self.task_proc.__enter__()

        robot_performing_task_exists = rclpy.Future()

        def fleet_states_check_start_task_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            for robot in fleet_state.robots:
                if len(robot.task_id) != 0:
                    robot_performing_task_exists.set_result(True)

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_check_start_task_cb, 10
        )
        result = await robot_performing_task_exists
        self.assertIsNotNone(result)

        # TODO(aaronchongth): Test with task_state_update ROS 2 topic when
        # released instead, as well as fleet_state_update for mode changes
        robot_done_with_task = rclpy.Future()
        end_x = 8.392851
        end_y = -7.75286
        end_level = 'L1'

        def fleet_states_check_task_done_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            for robot in fleet_state.robots:
                if len(robot.task_id) == 0 and \
                        math.sqrt((robot.location.x - end_x)**2 + (robot.location.y - end_y)**2) \
                        < 0.1 and \
                        robot.location.level_name == end_level:
                    robot_done_with_task.set_result(True)

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_check_task_done_cb, 10
        )
        result = await robot_done_with_task
        self.assertIsNotNone(result)

        self.custom_action_proc.__enter__()

        robot_performing_action_exists = rclpy.Future()

        def fleet_states_check_start_action_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            for robot in fleet_state.robots:
                if len(robot.task_id) != 0:
                    robot_performing_action_exists.set_result(True)

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_check_start_action_cb, 10
        )
        result = await robot_performing_action_exists
        self.assertIsNotNone(result)

        # TODO(aaronchongth): Test with task_state_update ROS 2 topic when
        # released instead, as well as fleet_state_update for mode changes
        robot_done_with_action = rclpy.Future()

        def fleet_states_check_action_done_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            for robot in fleet_state.robots:
                if len(robot.task_id) == 0:
                    robot_done_with_action.set_result(True)

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_check_action_done_cb, 10
        )
        result = await robot_done_with_action
        self.assertIsNotNone(result)
