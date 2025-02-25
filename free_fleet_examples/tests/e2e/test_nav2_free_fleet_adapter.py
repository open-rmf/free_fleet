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

import json
import time
import unittest
import uuid

from managed_process import managed_process
import rclpy
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse


class TestNav2FreeFleetAdapter(unittest.IsolatedAsyncioTestCase):

    @classmethod
    def setUpClass(cls):
        cls.rmf_common_proc = managed_process(
            (
                'ros2',
                'launch',
                'free_fleet_examples',
                'turtlebot3_world_rmf_common.launch.xml'
            ),
        )
        cls.rmf_common_proc.__enter__()

        cls.free_fleet_adapter_proc = managed_process(
            (
                'ros2',
                'launch',
                'free_fleet_examples',
                'nav2_tb3_simulation_fleet_adapter.launch.xml'
            ),
        )
        cls.free_fleet_adapter_proc.__enter__()

        cls.task_proc = managed_process(
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

        rclpy.init()
        cls.node = rclpy.create_node('test_nav2_robot_adapter')

        # give some time for discovery to happen
        time.sleep(5)

    @classmethod
    def tearDownClass(cls):
        cls.task_proc.__exit__(None, None, None)
        cls.free_fleet_adapter_proc.__exit__(None, None, None)
        cls.rmf_common_proc.__exit__(None, None, None)

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

        # # Modified from rmf_demos_tasks, dispatch_patrol
        # transient_qos = QoSProfile(
        #     history=History.KEEP_LAST,
        #     depth=1,
        #     reliability=Reliability.RELIABLE,
        #     durability=Durability.TRANSIENT_LOCAL,
        # )
        # pub = self.node.create_publisher(
        #     ApiRequest, 'task_api_requests', transient_qos
        # )

        # # Set task request request time and start time
        # request = {}
        # request['unix_millis_request_time'] = 0
        # request['unix_millis_earliest_start_time'] = 0
        # request['requester'] = 'test_patrol_task'
        # request['category'] = 'patrol'
        # request['description'] = {
        #     'places': ['north_west', 'north_east'],
        #     'rounds': 1
        # }

        # msg = ApiRequest()
        # msg.request_id = 'patrol_' + str(uuid.uuid4())
        # payload = {}
        # payload['request'] = request
        # msg.json_msg = json.dumps(payload)

        # api_response = rclpy.Future()

        # def receive_response(response_msg: ApiResponse):
        #     if response_msg.request_id == msg.request_id:
        #         api_response.set_result(json.loads(response_msg.json_msg))

        # transient_qos.depth = 10
        # self.node.create_subscription(
        #     ApiResponse, 'task_api_responses', receive_response,
        #     transient_qos
        # )

        # print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')
        # pub.publish(msg)

        # result = await api_response
        # self.assertIsNotNone(result)

        # assert len(result.request_id) != 0

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
