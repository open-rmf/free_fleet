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

import rclpy
from rmf_fleet_msgs.msg import FleetState

from ..managed_process import managed_process
from ..ros_testcase import RosTestCase


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

        print('Fleet is ready')

        # give some time for discovery to happen
        await self.ros_sleep(5)

    def tearDown(self):
        self.free_fleet_adapter_proc.__exit__(None, None, None)
        self.rmf_common_proc.__exit__(None, None, None)

    @RosTestCase.timeout(600)  # 10min
    async def test_robot_exists(self):
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

#     # def test_patrol_task(self):
#     #     transient_qos = QoSProfile(
#     #         history=History.KEEP_LAST,
#     #         depth=1,
#     #         reliability=Reliability.RELIABLE,
#     #         durability=Durability.TRANSIENT_LOCAL,
#     #     )
#     #     pub = self.node.create_publisher(
#     #         ApiRequest, 'task_api_requests', transient_qos
#     #     )

#     #     # Set task request request time and start time
#     #     now = self.get_clock().now().to_msg()
#     #     now.sec = now.sec + self.args.start_time
#     #     start_time = now.sec * 1000 + round(now.nanosec / 10**6)
#     #     request['unix_millis_request_time'] = start_time
#     #     request['unix_millis_earliest_start_time'] = start_time
#     #     # todo(YV): Fill priority after schema is added

#     #     request['requester'] = self.args.requester

#     #     # Define task request category
#     #     request['category'] = 'patrol'

#     #     if self.args.fleet:
#     #         request['fleet_name'] = self.args.fleet

#     #     # Define task request description
#     #     description = \
#     #         {'places': self.args.places, 'rounds': self.args.rounds}
#     #     request['description'] = description
#     #     payload['request'] = request
#     #     msg.json_msg = json.dumps(payload)

#     #     def receive_response(response_msg: ApiResponse):
#     #         if response_msg.request_id == msg.request_id:
#     #             self.response.set_result(json.loads(response_msg.json_msg))

#     #     transient_qos.depth = 10
#     #     self.sub = self.create_subscription(
#     #         ApiResponse, 'task_api_responses', receive_response,
#     #         transient_qos
#     #     )

# # ros2 run rmf_demos_tasks dispatch_patrol \
# #   -p north_west north_east south_east south_west \
# #   -n 2 \
# #   -st 0

#     # def test_non_existent_robot_pose(self):
#     #     tf_buffer = Buffer()

#     #     robot_adapter = Nav2RobotAdapter(
#     #         name='missing_nav2_tb3',
#     #         configuration=None,
#     #         robot_config_yaml={
#     #             'initial_map': 'L1',
#     #         },
#     #         node=self.node,
#     #         zenoh_session=self.zenoh_session,
#     #         fleet_handle=None,
#     #         tf_buffer=tf_buffer
#     #     )

#     #     time.sleep(2)
#     #     transform = robot_adapter.get_pose()
#     #     assert transform is None
