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

import atexit
import os
import signal
import subprocess
from contextlib import contextmanager
from subprocess import Popen


import inspect
import time
import unittest
from collections.abc import Callable, Coroutine, Sequence
from typing import TypeVar, cast
from uuid import uuid4

import rclpy
import rclpy.executors
import rclpy.node
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rmf_fleet_msgs.msg import FleetState, RobotState

# import os
import sys
# from typing import cast

# from action_msgs.msg import GoalStatus
# from managed_process import managed_process
# from nexus_orchestrator_msgs.action import ExecuteWorkOrder
# from nexus_orchestrator_msgs.msg import TaskState
# from nexus_test_case import NexusTestCase
# from rclpy import Future
# from rclpy.action import ActionClient
# from rclpy.action.client import ClientGoalHandle, GoalStatus
# from ros_testcase import RosTestCase
# import subprocess


def kill_process(proc: Popen):
    if proc.poll() is None:
        os.kill(-proc.pid, signal.SIGINT)
        proc.wait()


@contextmanager
def managed_process(*args, **kwargs):
    """
    A context managed process group that kills the process group when the
    context or when the script is exited. This avoid zombie processes in
    `ros2 run`, `ros2 launch` and other process launchers that do not kill
    their subprocesses on exit.

    :param args: The arguments to pass to `subprocess.Popen`.
    :param kwargs: The keyword arguments to pass to `subprocess.Popen`, except
    `start_new_session` which will always be `True`.
    """
    with subprocess.Popen(*args, **kwargs, start_new_session=True) as proc:
        def exit_cb():
            kill_process(proc)

        atexit.register(exit_cb)
        try:
            yield proc
        finally:
            exit_cb()
            atexit.unregister(exit_cb)


T = TypeVar("T", bound="RosTestCase")


class RosTestCase(unittest.TestCase):
    """
    A subclass of `unittest.TestCase` that supports async tests ran with ROS.
    This is similar to `IsolatedAsyncioTestCase` but using ROS executors
    instead of `asyncio`.

    Example usage:
    ```python
    class MyRosTest(RosTestCase):
        async def test_echo(self):
            fut = rclpy.Future()
            sub = self.node.create_subscription(
                std_msgs.msg.String, "/talker", fut.set_result, 10
            )
            result = await fut
            self.assertIsNotNone(result)
    ```
    """

    class Context:
        def __init__(self):
            self.ros_ctx = rclpy.Context()
            self.ros_ctx.init()
            self.node = rclpy.node.Node(  # type: ignore
                self._random_node_name("test_node"), context=self.ros_ctx
            )
            self.executor = rclpy.executors.SingleThreadedExecutor(
                context=self.ros_ctx)
            self.executor.add_node(self.node)

        def __del__(self):
            self.executor.shutdown()
            self.node.destroy_node()
            self.ros_ctx.shutdown()

        @staticmethod
        def _random_node_name(prefix: str):
            suffix = str(uuid4()).replace("-", "_")
            return f"{prefix}_{suffix}"

    class TestConfig:
        def __init__(self, *, timeout: float = 10):
            self.timeout = timeout

    @staticmethod
    def get_test_config(method: Callable) -> TestConfig:
        if hasattr(method, "__func__"):
            target = method.__func__
        else:
            target = method
        if not hasattr(target, "_test_config") or target._test_config is None:
            target._test_config = RosTestCase.TestConfig()
        return target._test_config

    @staticmethod
    def timeout(timeout: float):
        def deco(func: Callable[[T], Coroutine]):
            test_config = RosTestCase.get_test_config(func)
            test_config.timeout = timeout
            return func

        return deco

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)
        self._ros_test_ctx: RosTestCase.Context
        self.node: rclpy.node.Node

    def wait_for_nodes(self, *nodes: Sequence[str]):
        undiscovered = set(nodes)
        undiscovered.difference_update(self.node.get_node_names())
        while undiscovered:
            print("waiting for", undiscovered)
            time.sleep(0.1)
            undiscovered.difference_update(self.node.get_node_names())

    async def wait_for_robot_state(self):
        fut = rclpy.Future()
        sub = self.node.create_subscription(
            RobotState, "/robot_state", fut.set_result, 10
        )
        result = await fut
        self.assertIsNotNone(result)

    async def ros_sleep(self, secs: float):
        """
        async sleep using ros timers
        """
        fut = rclpy.Future()

        def done():
            timer.destroy()
            fut.set_result(None)

        timer = self.node.create_timer(secs, done)
        await fut

    def wait_for(
            self, rclpy_fut: rclpy.Future, timeout: float
    ) -> rclpy.Future:
        """
        Wait for a future to complete.
        Returns a future that resolves to `True` if the inner future completes,
        or `False` if timeout occurs.
        """
        wrapper_fut = rclpy.Future()

        def on_timeout():
            timer.cancel()
            wrapper_fut.set_result(False)

        timer = self.node.create_timer(timeout, on_timeout)

        def on_success(_):
            wrapper_fut.set_result(True)
            timer.cancel()

        rclpy_fut.add_done_callback(on_success)

        return wrapper_fut

    async def wait_for_lifecycle_active(self, target_node: str):
        get_state_client: rclpy.node.Client = self.node.create_client(
            GetState, f"/{target_node}/get_state"
        )
        if not get_state_client.wait_for_service(5):
            raise TimeoutError("timed out waiting for get_state service")
        resp = cast(
            GetState.Response,
            await get_state_client.call_async(GetState.Request())
        )
        while resp.current_state.id != State.PRIMARY_STATE_ACTIVE:
            print(f"waiting for {target_node} to be active")
            await self.ros_sleep(0.1)
            resp = cast(
                GetState.Response,
                await get_state_client.call_async(GetState.Request())
            )

    async def asyncSetUp(self):
        pass

    async def asyncTearDown(self):
        pass

    def _setUpRos(self):
        self._ros_test_ctx = RosTestCase.Context()
        self.node = self._ros_test_ctx.node

    def _callSetUp(self):
        self._setUpRos()
        super().setUp()
        coro = self.asyncSetUp()
        test_config = RosTestCase.get_test_config(self.asyncSetUp)
        if coro:
            self._spin_ros(coro, test_config.timeout)

    def _tearDownRos(self):
        del self._ros_test_ctx

    def _callTearDown(self):
        coro = self.asyncTearDown()
        test_config = RosTestCase.get_test_config(self.asyncTearDown)
        if coro:
            self._spin_ros(coro, test_config.timeout)
        super().tearDown()
        self._tearDownRos()

    def _spin_ros(self, coro: Coroutine, timeout: float = 10):
        timeout_fut = rclpy.Future()

        def on_timeout():
            timer.cancel()
            timeout_fut.set_result(None)

        timer = self.node.create_timer(timeout, on_timeout)
        task = self._ros_test_ctx.executor.create_task(coro)

        # We can't use spin_until_future_complete because of
        # https://github.com/ros2/rclpy/issues/985
        while not task.done() and not timeout_fut.done():
            self._ros_test_ctx.executor.spin_once()
        if not task.done():
            raise TimeoutError("Test timed out")

    def _callTestMethod(self, method):
        assert self._ros_test_ctx is not None, \
            "RosTestCase context is not initialized"
        test_config = RosTestCase.get_test_config(method)
        ret = method()
        if inspect.iscoroutine(ret):
            return self._spin_ros(ret, test_config.timeout)
        else:
            return ret


class RobotExistsTest(RosTestCase):
    @RosTestCase.timeout(60)
    async def asyncSetUp(self):
        # todo(YV): Find a better fix to the problem below.
        # zenoh-bridge was bumped to 0.72 as part of the upgrade to
        # ROS 2 Iron. However with this upgrade, the bridge does not clearly
        # terminate when a SIGINT is received leaving behind zombie bridge
        # processes from previous test cases. As a result, workcell
        # registration fails for this testcase due to multiple bridges
        # remaining active. Hence we explicitly kill any zenoh processes before
        # launching the test.
        # subprocess.Popen('pkill -9 -f zenoh', shell=True)

        self.proc = managed_process(
                (
                     "ros2",
                     "launch",
                     "free_fleet_examples",
                     "new_test.launch.xml"
                 ),
        )
        self.proc.__enter__()
        # print("waiting for nodes to be ready...", file=sys.stderr)
        # self.wait_for_nodes("system_orchestrator")
        # await self.wait_for_lifecycle_active("system_orchestrator")

        # await self.wait_for_workcells(
        #   "workcell_1", "workcell_2", "rmf_nexus_transporter")
        # print("all workcells are ready")
        await self.wait_for_robot_state()
        print("AMRs are ready")

        # give some time for discovery to happen
        await self.ros_sleep(5)

        # create action client to send work order
        # self.action_client = ActionClient(
        #     self.node, ExecuteWorkOrder, "/system_orchestrator/execute_order"
        # )

    def tearDown(self):
        self.proc.__exit__(None, None, None)

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

        rclpy.spin_until_future_complete(
            self.node, robot_exists, timeout_sec=20.0
        )

        robot_found = False
        if robot_exists.done():
            robot_found = True
        self.assertTrue(robot_found)

        # self.action_client.wait_for_server()
        # goal_msg = ExecuteWorkOrder.Goal()
        # goal_msg.order.work_order_id = "1"
        # with open(
        #   f"{os.path.dirname(__file__)}/config/pick_and_place.json"
        # ) as f:
        #     goal_msg.order.work_order = f.read()
        # feedbacks: list[ExecuteWorkOrder.Feedback] = []
        # fb_fut = Future()

        # def on_fb(msg):
        #     feedbacks.append(msg.feedback)
        #     if len(feedbacks) >= 5:
        #         fb_fut.set_result(None)

        # goal_handle = cast(
        #     ClientGoalHandle,
        #     await self.action_client.send_goal_async(goal_msg, on_fb)
        # )
        # self.assertTrue(goal_handle.accepted)

        # results = await goal_handle.get_result_async()
        # self.assertEqual(results.status, GoalStatus.STATUS_SUCCEEDED)

        # # check that we receive the correct feedbacks
        # # FIXME(koonpeng): First few feedbacks are sometimes missed when the
        # system in under high load so we only check the last feedback as a
        # workaround.
        # self.assertGreater(len(feedbacks), 0)
        # for msg in feedbacks:
        #     # The first task is transportation
        #     self.assertEqual(len(msg.task_states), 3)
        #     state: TaskState = msg.task_states[1]  # type: ignore
        #     self.assertEqual(state.workcell_id, "workcell_1")
        #     self.assertEqual(state.task_id, "1/place_on_conveyor/0")
        #     state: TaskState = msg.task_states[2]  # type: ignore
        #     self.assertEqual(state.workcell_id, "workcell_2")
        #     self.assertEqual(state.task_id, "1/pick_from_conveyor/1")

        # state: TaskState = feedbacks[-1].task_states[0]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)
        # state: TaskState = feedbacks[-1].task_states[1]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)
        # state: TaskState = feedbacks[-1].task_states[2]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)


# import asyncio
# import unittest

# import rclpy
# from rmf_fleet_msgs.msg import FleetState

# class TestNav2FreeFleetAdapter(unittest.TestCase):

#     @classmethod
#     def setUpClass(cls):
#         rclpy.init()
#         cls.node = rclpy.create_node('test_nav2_free_fleet_adapter')

#     @classmethod
#     def tearDownClass(cls):
#         rclpy.shutdown()

#     def test_robot_exists(self):
#         robot_exists = asyncio.Future()

#         def fleet_states_cb(fleet_state: FleetState):
#             if fleet_state.name != 'turtlebot3':
#                 return
#             if len(fleet_state.robots) == 1 and \
#                     fleet_state.robots[0].name == 'nav2_tb3':
#                 robot_exists.set_result('found nav2_tb3')

#         self.node.create_subscription(
#             FleetState, 'fleet_states', fleet_states_cb, 10
#         )

#         rclpy.spin_until_future_complete(
#             self.node, robot_exists, timeout_sec=20.0
#         )

#         robot_found = False
#         if robot_exists.done():
#             robot_found = True
#         assert robot_found

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
