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

        # give some time for discovery to happen
        await self.ros_sleep(5)

    def tearDown(self):
        self.task_proc.__exit__(None, None, None)
        self.free_fleet_adapter_proc.__exit__(None, None, None)
        self.rmf_common_proc.__exit__(None, None, None)

    @RosTestCase.timeout(120)  # 2min
    async def test_patrol_task(self):
        robot_exists = rclpy.Future()

        def fleet_states_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            if len(fleet_state.robots) == 1 and \
                    fleet_state.robots[0].name == 'nav2_tb3':
                robot_exists.set_result('found nav2_tb3')

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_cb, 10
        )
        result = await robot_exists
        self.assertIsNotNone(result)

        self.task_proc.__enter__()

        robot_performing_task_exists = rclpy.Future()

        def fleet_states_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            for robot in fleet_state.robots:
                if len(robot.task_id) != 0:
                    robot_performing_task_exists.set_result(True)

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_cb, 10
        )
        result = await robot_performing_task_exists
        self.assertIsNotNone(result)
