#!/usr/bin/env python3

# Copyright (C) 2022 Johnson & Johnson
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

# Vendored from https://github.com/osrf/nexus

from collections.abc import Callable, Coroutine, Sequence
import inspect
import time
from typing import cast, TypeVar
import unittest
from uuid import uuid4

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
import rclpy.executors
import rclpy.node
from rmf_fleet_msgs.msg import RobotState


T = TypeVar('T', bound='RosTestCase')


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
                std_msgs.msg.String, '/talker', fut.set_result, 10
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
                self._random_node_name('test_node'), context=self.ros_ctx
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
            suffix = str(uuid4()).replace('-', '_')
            return f'{prefix}_{suffix}'

    class TestConfig:

        def __init__(self, *, timeout: float = 10):
            self.timeout = timeout

    @staticmethod
    def get_test_config(method: Callable) -> TestConfig:
        if hasattr(method, '__func__'):
            target = method.__func__
        else:
            target = method
        if not hasattr(target, '_test_config') or target._test_config is None:
            target._test_config = RosTestCase.TestConfig()
        return target._test_config

    @staticmethod
    def timeout(timeout: float):
        def deco(func: Callable[[T], Coroutine]):
            test_config = RosTestCase.get_test_config(func)
            test_config.timeout = timeout
            return func

        return deco

    def __init__(self, methodName='runTest'):
        super().__init__(methodName)
        self._ros_test_ctx: RosTestCase.Context
        self.node: rclpy.node.Node

    def wait_for_nodes(self, *nodes: Sequence[str]):
        undiscovered = set(nodes)
        undiscovered.difference_update(self.node.get_node_names())
        while undiscovered:
            print('waiting for', undiscovered)
            time.sleep(0.1)
            undiscovered.difference_update(self.node.get_node_names())

    async def wait_for_robot_state(self):
        fut = rclpy.Future()
        self.node.create_subscription(
            RobotState, '/robot_state', fut.set_result, 10
        )
        result = await fut
        self.assertIsNotNone(result)

    async def ros_sleep(self, secs: float):
        """Async sleep using ros timers."""
        fut = rclpy.Future()

        def done():
            timer.destroy()
            fut.set_result(None)

        timer = self.node.create_timer(secs, done)
        await fut

    def wait_for(
            self, rclpy_fut: rclpy.Future, timeout: float) -> rclpy.Future:
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
            GetState, f'/{target_node}/get_state'
        )
        if not get_state_client.wait_for_service(5):
            raise TimeoutError('timed out waiting for get_state service')
        resp = cast(
            GetState.Response,
            await get_state_client.call_async(GetState.Request())
        )
        while resp.current_state.id != State.PRIMARY_STATE_ACTIVE:
            print(f'waiting for {target_node} to be active')
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
            raise TimeoutError('Test timed out')

    def _callTestMethod(self, method):
        assert self._ros_test_ctx is not None, \
            'RosTestCase context is not initialized'
        test_config = RosTestCase.get_test_config(method)
        ret = method()
        if inspect.iscoroutine(ret):
            return self._spin_ros(ret, test_config.timeout)
        else:
            return ret
