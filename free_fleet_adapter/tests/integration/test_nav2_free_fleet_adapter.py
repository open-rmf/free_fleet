# #!/usr/bin/env python3

# # Copyright 2025 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# import asyncio
# import threading
# import time
# import unittest

# from free_fleet_adapter.fleet_adapter import start_fleet_adapter
# import pytest
# import rclpy
# from rmf_fleet_msgs.msg import FleetState

# class TestNav2FreeFleetAdapter(unittest.TestCase):

#     @pytest.fixture(scope='class', autouse=True)
#     def setup_paths(self, tmp_path_factory):
#         fleet_config_yaml = """
# # FLEET CONFIG ================================================================
# # RMF Fleet parameters

# rmf_fleet:
#   name: "turtlebot3"
#   limits:
#     linear: [0.5, 0.75] # velocity, acceleration
#     angular: [0.6, 2.0] # velocity, acceleration
#   profile: # Robot profile is modelled as a circle
#     footprint: 0.3 # radius in m
#     vicinity: 0.5 # radius in m
#   reversible: True # whether robots in this fleet can reverse
#   battery_system:
#     voltage: 12.0 # V
#     capacity: 24.0 # Ahr
#     charging_current: 5.0 # A
#   mechanical_system:
#     mass: 20.0 # kg
#     moment_of_inertia: 10.0 #kgm^2
#     friction_coefficient: 0.22
#   ambient_system:
#     power: 20.0 # W
#   tool_system:
#     power: 0.0 # W
#   recharge_threshold: 0.10
#   recharge_soc: 1.0
#   publish_fleet_state: 10.0
#   account_for_battery_drain: True
#   task_capabilities:
#     loop: True
#     delivery: True
#   # actions: ["some_action_here"]
#   finishing_request: "nothing" # [park, charge, nothing]
#   responsive_wait: True
#   robots:
#     nav2_tb3:
#       charger: "tb3_charger"
#       responsive_wait: False
#       # For Nav2RobotAdapter
#       navigation_stack: 2
#       initial_map: "L1"
#       maps:
#         L1:
#           map_url: "/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml"
#       # initial_pose: [-1.6000019311904907, -0.5031192898750305, 0]

#   robot_state_update_frequency: 10.0 # Hz


# # TRANSFORM CONFIG ============================================================
# # For computing transforms between Robot and RMF coordinate systems

# # Optional
# reference_coordinates:
#   L1:
#     rmf: [[8.9508, -6.6006],
#           [7.1006, -9.1508],
#           [12.3511, -9.2008],
#           [11.0510, -11.8010]]
#     robot: [[-1.04555, 2.5456],
#           [-2.90519, 0.00186],
#           [2.39611, -0.061773],
#           [1.08783, -2.59750]]
# """
#         TestNav2FreeFleetAdapter.__fleet_config_path = \
#             tmp_path_factory.mktemp('configs') / 'fleet_config.yaml'
#         TestNav2FreeFleetAdapter.__fleet_config_path.write_text(
#             fleet_config_yaml,
#             encoding='utf-8'
#         )

#     # @pytest.fixture(scope='class', autouse=True)
#     # def nav_graph_path(self, tmp_path_factory):
#         nav_graph_yaml = """
# building_name: turtlebot3_world
# doors: {}
# levels:
#   L1:
#     lanes:
#     - - 0
#       - 1
#       - {}
#     - - 1
#       - 0
#       - {}
#     - - 1
#       - 2
#       - {}
#     - - 2
#       - 1
#       - {}
#     - - 2
#       - 3
#       - {}
#     - - 3
#       - 2
#       - {}
#     - - 3
#       - 4
#       - {}
#     - - 4
#       - 3
#       - {}
#     - - 4
#       - 5
#       - {}
#     - - 5
#       - 4
#       - {}
#     - - 5
#       - 6
#       - {}
#     - - 6
#       - 5
#       - {}
#     - - 6
#       - 7
#       - {}
#     - - 7
#       - 6
#       - {}
#     - - 7
#       - 8
#       - {}
#     - - 8
#       - 7
#       - {}
#     - - 8
#       - 9
#       - {}
#     - - 9
#       - 8
#       - {}
#     - - 9
#       - 10
#       - {}
#     - - 10
#       - 9
#       - {}
#     - - 10
#       - 11
#       - {}
#     - - 11
#       - 10
#       - {}
#     - - 11
#       - 0
#       - {}
#     - - 0
#       - 11
#       - {}
#     - - 11
#       - 12
#       - {}
#     - - 12
#       - 11
#       - {}
#     - - 12
#       - 13
#       - {}
#     - - 13
#       - 12
#       - {}
#     - - 14
#       - 15
#       - {}
#     - - 15
#       - 14
#       - {}
#     - - 15
#       - 10
#       - {}
#     - - 10
#       - 15
#       - {}
#     - - 8
#       - 15
#       - {}
#     - - 15
#       - 8
#       - {}
#     - - 12
#       - 1
#       - {}
#     - - 1
#       - 12
#       - {}
#     - - 7
#       - 14
#       - {}
#     - - 14
#       - 7
#       - {}
#     - - 13
#       - 2
#       - {}
#     - - 2
#       - 13
#       - {}
#     - - 13
#       - 4
#       - {}
#     - - 4
#       - 13
#       - {}
#     - - 14
#       - 5
#       - {}
#     - - 5
#       - 14
#       - {}
#     - - 15
#       - 16
#       - {}
#     - - 16
#       - 15
#       - {}
#     - - 16
#       - 12
#       - {}
#     - - 12
#       - 16
#       - {}
#     - - 14
#       - 17
#       - {}
#     - - 17
#       - 14
#       - {}
#     - - 17
#       - 13
#       - {}
#     - - 13
#       - 17
#       - {}
#     vertices:
#     - - 11.579375256202459
#       - -7.622474905986078
#       - {name: north_east}
#     - - 11.77314241265301
#       - -8.68821926867784
#       - {name: ''}
#     - - 11.789293842726178
#       - -9.75936410953648
#       - {name: ''}
#     - - 11.600877160015036
#       - -10.706747992404125
#       - {name: south_east}
#     - - 10.610489469522237
#       - -10.992023251126804
#       - {name: ''}
#     - - 9.50704176851726
#       - -10.965120869147347
#       - {name: ''}
#     - - 8.398193589345404
#       - -10.604438933798233
#       - {name: south_west}
#     - - 8.199075959155444
#       - -9.813218877922866
#       - {is_charger: true, name: turtlebot3_1_charger}
#     - - 8.215177384801141
#       - -8.747474515231104
#       - {name: ''}
#     - - 8.478950739711244
#       - -7.595572524006621
#       - {name: north_west}
#     - - 9.41553366624513
#       - -7.353351077336566
#       - {name: ''}
#     - - 10.631991373334815
#       - -7.364102029242854
#       - {name: ''}
#     - - 10.599688513188477
#       - -8.623663552812635
#       - {name: ''}
#     - - 10.605088991355357
#       - -9.76476458770336
#       - {name: ''}
#     - - 9.447836526391466
#       - -9.75401363579707
#       - {name: ''}
#     - - 9.447836526391466
#       - -8.68286879493843
#       - {name: ''}
#     - - 10.053690169631432
#       - -8.673117931581563
#       - {is_charger: true, name: robot1_charger}
#     - - 10.022537411316929
#       - -9.7629644283144
#       - {is_charger: true, name: robot2_charger}
# lifts: {}
# """
#         TestNav2FreeFleetAdapter.__nav_graph_path = \
#             tmp_path_factory.mktemp('configs') / 'nav_graph.yaml'
#         TestNav2FreeFleetAdapter.__nav_graph_path.write_text(
#             nav_graph_yaml,
#             encoding='utf-8'
#         )

#     @classmethod
#     def setUpClass(cls):
#         rclpy.init()
#         cls.node = rclpy.create_node('test_nav2_free_fleet_adapter')

#     #     def start_adapter():
#     #         start_fleet_adapter(
#     #             config_path=str(cls.__fleet_config_path),
#     #             nav_graph_path=str(cls.__nav_graph_path),
#     #             zenoh_config_path=None,
#     #             server_uri=None,
#     #             use_sim_time=True
#     #         )
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread = \
#     #         threading.Thread(target=start_adapter, args=())
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread.daemon = True
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread.start()

#     @classmethod
#     def tearDownClass(cls):
#         rclpy.shutdown()

#     # @pytest.fixture(scope='class', autouse=True)
#     # def start_fleet_adapter(self):
#     #     def start_adapter():
#     #         start_fleet_adapter(
#     #             config_path=str(self.__fleet_config_path),
#     #             nav_graph_path=str(self.__nav_graph_path),
#     #             zenoh_config_path=None,
#     #             server_uri=None,
#     #             use_sim_time=True
#     #         )
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread = \
#     #         threading.Thread(target=start_adapter, args=())
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread.daemon = True
#     #     TestNav2FreeFleetAdapter.__fleet_adapter_thread.start()
#     #     yield

#     # @pytest.fixture(scope='session')
#     # def start_adapter(self):
#     #     # def __start_adapter():
#     #     #     start_fleet_adapter(
#     #     #         config_path=str(self.__fleet_config_path),
#     #     #         nav_graph_path=str(self.__nav_graph_path),
#     #     #         zenoh_config_path=None,
#     #     #         server_uri=None,
#     #     #         use_sim_time=True
#     #     #     )
#     #     # # TestNav2FreeFleetAdapter.__fleet_adapter_thread = \
#     #     # #     threading.Thread(target=start_adapter, args=())
#     #     # # TestNav2FreeFleetAdapter.__fleet_adapter_thread.daemon = True
#     #     # # TestNav2FreeFleetAdapter.__fleet_adapter_thread.start()
#     #     # fleet_adapter_thread = \
#     #     #     threading.Thread(target=__start_adapter, args=())
#     #     # fleet_adapter_thread.daemon = True
#     #     # fleet_adapter_thread.start()
#     #     # # time.sleep(5)
#     #     # # # assert True

#     #     start_fleet_adapter(
#     #         config_path=str(self.__fleet_config_path),
#     #         nav_graph_path=str(self.__nav_graph_path),
#     #         zenoh_config_path=None,
#     #         server_uri=None,
#     #         use_sim_time=True
#     #     )

#     # @pytest.mark.launch(fixture=launch_description)
#     def test_robot_exists(self):
#         def start_adapter():
#             start_fleet_adapter(
#                 config_path=str(self.__fleet_config_path),
#                 nav_graph_path=str(self.__nav_graph_path),
#                 zenoh_config_path=None,
#                 server_uri=None,
#                 use_sim_time=True
#             )
#         fleet_adapter_thread = \
#             threading.Thread(target=start_adapter, args=())
#         fleet_adapter_thread.daemon = True
#         fleet_adapter_thread.start()

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
#     #     description = {'places': self.args.places, 'rounds': self.args.rounds}
#     #     request['description'] = description
#     #     payload['request'] = request
#     #     msg.json_msg = json.dumps(payload)

#     #     def receive_response(response_msg: ApiResponse):
#     #         if response_msg.request_id == msg.request_id:
#     #             self.response.set_result(json.loads(response_msg.json_msg))

#     #     transient_qos.depth = 10
#     #     self.sub = self.create_subscription(
#     #         ApiResponse, 'task_api_responses', receive_response, transient_qos
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


import asyncio
import atexit
import os
import pytest
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

# from nexus_orchestrator_msgs.action import ExecuteWorkOrder
# from nexus_test_case import NexusTestCase
# from managed_process import managed_process
# from rclpy.action import ActionClient
# from rclpy.action.client import ClientGoalHandle
# from ros_testcase import RosTestCase
import subprocess

T = TypeVar("T", bound="RosTestCase")


def kill_process(proc: Popen):
    if proc.poll() is None:
        os.kill(-proc.pid, signal.SIGINT)
        proc.wait()


@contextmanager
def managed_process(*args, **kwargs):
    """
    A context managed process group that kills the process group when the context or
    when the script is exited. This avoid zombie processes in `ros2 run`, `ros2 launch` and other
    process launchers that do not kill their subprocesses on exit.

    :param args: The arguments to pass to `subprocess.Popen`.
    :param kwargs: The keyword arguments to pass to `subprocess.Popen`, except `start_new_session`
        which will always be `True`.
    """
    with subprocess.Popen(*args, **kwargs, start_new_session=True) as proc:
        exit_cb = lambda: kill_process(proc)
        atexit.register(exit_cb)
        try:
            yield proc
        finally:
            exit_cb()
            atexit.unregister(exit_cb)


class RosTestCase(unittest.TestCase):
    """
    A subclass of `unittest.TestCase` that supports async tests ran with ROS. This is similar
    to `IsolatedAsyncioTestCase` but using ROS executors instead of `asyncio`.

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
            self.executor = rclpy.executors.SingleThreadedExecutor(context=self.ros_ctx)
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

    def wait_for(self, rclpy_fut: rclpy.Future, timeout: float) -> rclpy.Future:
        """
        Wait for a future to complete.
        Returns a future that resolves to `True` if the inner future completes, or `False` if timeout occurs.
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
            GetState.Response, await get_state_client.call_async(GetState.Request())
        )
        while resp.current_state.id != State.PRIMARY_STATE_ACTIVE:
            print(f"waiting for {target_node} to be active")
            await self.ros_sleep(0.1)
            resp = cast(
                GetState.Response, await get_state_client.call_async(GetState.Request())
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

        # We can't use spin_until_future_complete because of https://github.com/ros2/rclpy/issues/985
        while not task.done() and not timeout_fut.done():
            self._ros_test_ctx.executor.spin_once()
        if not task.done():
            raise TimeoutError("Test timed out")

    def _callTestMethod(self, method):
        assert self._ros_test_ctx is not None, "RosTestCase context is not initialized"
        test_config = RosTestCase.get_test_config(method)
        ret = method()
        if inspect.iscoroutine(ret):
            return self._spin_ros(ret, test_config.timeout)
        else:
            return ret


class TestNav2FreeFleetAdapter(RosTestCase):

    @pytest.fixture(scope='class', autouse=True)
    def setup_paths(self, tmp_path_factory):
        fleet_config_yaml = """
# FLEET CONFIG ================================================================
# RMF Fleet parameters

rmf_fleet:
  name: "turtlebot3"
  limits:
    linear: [0.5, 0.75] # velocity, acceleration
    angular: [0.6, 2.0] # velocity, acceleration
  profile: # Robot profile is modelled as a circle
    footprint: 0.3 # radius in m
    vicinity: 0.5 # radius in m
  reversible: True # whether robots in this fleet can reverse
  battery_system:
    voltage: 12.0 # V
    capacity: 24.0 # Ahr
    charging_current: 5.0 # A
  mechanical_system:
    mass: 20.0 # kg
    moment_of_inertia: 10.0 #kgm^2
    friction_coefficient: 0.22
  ambient_system:
    power: 20.0 # W
  tool_system:
    power: 0.0 # W
  recharge_threshold: 0.10
  recharge_soc: 1.0
  publish_fleet_state: 10.0
  account_for_battery_drain: True
  task_capabilities:
    loop: True
    delivery: True
  # actions: ["some_action_here"]
  finishing_request: "nothing" # [park, charge, nothing]
  responsive_wait: True
  robots:
    nav2_tb3:
      charger: "tb3_charger"
      responsive_wait: False
      # For Nav2RobotAdapter
      navigation_stack: 2
      initial_map: "L1"
      maps:
        L1:
          map_url: "/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml"
      # initial_pose: [-1.6000019311904907, -0.5031192898750305, 0]

  robot_state_update_frequency: 10.0 # Hz


# TRANSFORM CONFIG ============================================================
# For computing transforms between Robot and RMF coordinate systems

# Optional
reference_coordinates:
  L1:
    rmf: [[8.9508, -6.6006],
          [7.1006, -9.1508],
          [12.3511, -9.2008],
          [11.0510, -11.8010]]
    robot: [[-1.04555, 2.5456],
          [-2.90519, 0.00186],
          [2.39611, -0.061773],
          [1.08783, -2.59750]]
"""
        TestNav2FreeFleetAdapter.__fleet_config_path = \
            tmp_path_factory.mktemp('configs') / 'fleet_config.yaml'
        TestNav2FreeFleetAdapter.__fleet_config_path.write_text(
            fleet_config_yaml,
            encoding='utf-8'
        )

    # @pytest.fixture(scope='class', autouse=True)
    # def nav_graph_path(self, tmp_path_factory):
        nav_graph_yaml = """
building_name: turtlebot3_world
doors: {}
levels:
  L1:
    lanes:
    - - 0
      - 1
      - {}
    - - 1
      - 0
      - {}
    - - 1
      - 2
      - {}
    - - 2
      - 1
      - {}
    - - 2
      - 3
      - {}
    - - 3
      - 2
      - {}
    - - 3
      - 4
      - {}
    - - 4
      - 3
      - {}
    - - 4
      - 5
      - {}
    - - 5
      - 4
      - {}
    - - 5
      - 6
      - {}
    - - 6
      - 5
      - {}
    - - 6
      - 7
      - {}
    - - 7
      - 6
      - {}
    - - 7
      - 8
      - {}
    - - 8
      - 7
      - {}
    - - 8
      - 9
      - {}
    - - 9
      - 8
      - {}
    - - 9
      - 10
      - {}
    - - 10
      - 9
      - {}
    - - 10
      - 11
      - {}
    - - 11
      - 10
      - {}
    - - 11
      - 0
      - {}
    - - 0
      - 11
      - {}
    - - 11
      - 12
      - {}
    - - 12
      - 11
      - {}
    - - 12
      - 13
      - {}
    - - 13
      - 12
      - {}
    - - 14
      - 15
      - {}
    - - 15
      - 14
      - {}
    - - 15
      - 10
      - {}
    - - 10
      - 15
      - {}
    - - 8
      - 15
      - {}
    - - 15
      - 8
      - {}
    - - 12
      - 1
      - {}
    - - 1
      - 12
      - {}
    - - 7
      - 14
      - {}
    - - 14
      - 7
      - {}
    - - 13
      - 2
      - {}
    - - 2
      - 13
      - {}
    - - 13
      - 4
      - {}
    - - 4
      - 13
      - {}
    - - 14
      - 5
      - {}
    - - 5
      - 14
      - {}
    - - 15
      - 16
      - {}
    - - 16
      - 15
      - {}
    - - 16
      - 12
      - {}
    - - 12
      - 16
      - {}
    - - 14
      - 17
      - {}
    - - 17
      - 14
      - {}
    - - 17
      - 13
      - {}
    - - 13
      - 17
      - {}
    vertices:
    - - 11.579375256202459
      - -7.622474905986078
      - {name: north_east}
    - - 11.77314241265301
      - -8.68821926867784
      - {name: ''}
    - - 11.789293842726178
      - -9.75936410953648
      - {name: ''}
    - - 11.600877160015036
      - -10.706747992404125
      - {name: south_east}
    - - 10.610489469522237
      - -10.992023251126804
      - {name: ''}
    - - 9.50704176851726
      - -10.965120869147347
      - {name: ''}
    - - 8.398193589345404
      - -10.604438933798233
      - {name: south_west}
    - - 8.199075959155444
      - -9.813218877922866
      - {is_charger: true, name: turtlebot3_1_charger}
    - - 8.215177384801141
      - -8.747474515231104
      - {name: ''}
    - - 8.478950739711244
      - -7.595572524006621
      - {name: north_west}
    - - 9.41553366624513
      - -7.353351077336566
      - {name: ''}
    - - 10.631991373334815
      - -7.364102029242854
      - {name: ''}
    - - 10.599688513188477
      - -8.623663552812635
      - {name: ''}
    - - 10.605088991355357
      - -9.76476458770336
      - {name: ''}
    - - 9.447836526391466
      - -9.75401363579707
      - {name: ''}
    - - 9.447836526391466
      - -8.68286879493843
      - {name: ''}
    - - 10.053690169631432
      - -8.673117931581563
      - {is_charger: true, name: robot1_charger}
    - - 10.022537411316929
      - -9.7629644283144
      - {is_charger: true, name: robot2_charger}
lifts: {}
"""
        TestNav2FreeFleetAdapter.__nav_graph_path = \
            tmp_path_factory.mktemp('configs') / 'nav_graph.yaml'
        TestNav2FreeFleetAdapter.__nav_graph_path.write_text(
            nav_graph_yaml,
            encoding='utf-8'
        )

    @RosTestCase.timeout(60)
    async def asyncSetUp(self):
        # todo(YV): Find a better fix to the problem below.
        # zenoh-bridge was bumped to 0.72 as part of the upgrade to
        # ROS 2 Iron. However with this upgrade, the bridge does not clearly
        # terminate when a SIGINT is received leaving behind zombie bridge
        # processes from previous test cases. As a result, workcell registration
        # fails for this testcase due to multiple bridges remaining active.
        # Hence we explicitly kill any zenoh processes before launching the test.
        # subprocess.Popen('pkill -9 -f zenoh', shell=True)

        self.proc = managed_process(
            ("ros2", "launch", "free_fleet_adapter", "fleet_adapter.launch.xml", "use_sim_time:=false", f"config_file:={self.__fleet_config_path}", f"nav_graph_file:={self.__nav_graph_path}"),
        )
        self.proc.__enter__()
        print("waiting for nodes to be ready...", file=sys.stderr)
        self.wait_for_nodes("turtlebot3_fleet_adapter")
        # await self.wait_for_lifecycle_active("system_orchestrator")

        # await self.wait_for_workcells("workcell_1", "workcell_2")
        # print("all workcells are ready")
        # await self.wait_for_transporters("transporter_node")
        # print("all transporters are ready")

        # create action client to send work order
        # self.action_client = ActionClient(
        #     self.node, ExecuteWorkOrder, "/system_orchestrator/execute_order"
        # )
        # self.action_client.wait_for_server()

    def tearDown(self):
        self.proc.__exit__(None, None, None)

    # @RosTestCase.timeout(180)  # 3min
    # async def test_reject_jobs_over_max(self):
    #     """
    #     New jobs should be rejected when the max number of jobs is already executing.
    #     """
    #     goal_msg = ExecuteWorkOrder.Goal()
    #     goal_msg.order.id = "1"
    #     with open(f"{os.path.dirname(__file__)}/config/place_on_conveyor.json") as f:
    #         goal_msg.order.work_order = f.read()
    #     goal_handle = cast(
    #         ClientGoalHandle, await self.action_client.send_goal_async(goal_msg)
    #     )
    #     self.assertTrue(goal_handle.accepted)

    #     goal_msg_2 = ExecuteWorkOrder.Goal()
    #     goal_msg_2.order.id = "2"
    #     with open(f"{os.path.dirname(__file__)}/config/pick_from_conveyor.json") as f:
    #         goal_msg_2.order.work_order = f.read()
    #     goal_handle_2 = cast(
    #         ClientGoalHandle, await self.action_client.send_goal_async(goal_msg_2)
    #     )
    #     self.assertTrue(goal_handle_2.accepted)

    #     goal_msg_3 = goal_msg
    #     goal_msg_3.order.id = "3"
    #     goal_handle_3 = cast(
    #         ClientGoalHandle, await self.action_client.send_goal_async(goal_msg_3)
    #     )
    #     self.assertFalse(goal_handle_3.accepted)

    # @RosTestCase.timeout(180)  # 3min
    # async def test_robot_exists(self):
    def test_robot_exists(self):
        time.sleep(30)

        robot_exists = asyncio.Future()

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
        assert robot_found
