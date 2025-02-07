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

import threading
import time
import unittest

from free_fleet_adapter.fleet_adapter import start_fleet_adapter
import pytest
import rclpy
from rmf_fleet_msgs.msg import FleetState


class TestNav2FreeFleetAdapter(unittest.TestCase):

    @pytest.fixture(autouse=True)
    def initdir(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)

        # Fleet config
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
        tmp_path.joinpath('fleet_config.yaml').write_text(
            fleet_config_yaml,
            encoding='utf-8'
        )

        # Nav graph
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
        tmp_path.joinpath('nav_graph.yaml').write_text(
            nav_graph_yaml,
            encoding='utf-8'
        )

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_nav2_free_fleet_adapter')

        def start_adapter():
            start_fleet_adapter(
                config_path='fleet_config.yaml',
                nav_graph_path='nav_graph.yaml',
                zenoh_config_path=None,
                server_uri=None,
                use_sim_time=True
            )

        update_thread = threading.Thread(target=start_adapter, args=())
        update_thread.start()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_robot_exists(self):
        robot_exists = False

        def fleet_states_cb(fleet_state: FleetState):
            if fleet_state.name != 'turtlebot3':
                return
            if len(fleet_state.robots) == 1 and fleet_state.robots[0].name == 'nav2_tb3':
                robot_exists = True

        self.node.create_subscription(
            FleetState, 'fleet_states', fleet_states_cb, 10
        )

        for i in range(10):
            rclpy.spin_once(self.node)
            time.sleep(0.2)

            if robot_exists:
                break

        assert robot_exists

    # def test_patrol_task(self):

    # def test_non_existent_robot_pose(self):
    #     tf_buffer = Buffer()

    #     robot_adapter = Nav2RobotAdapter(
    #         name='missing_nav2_tb3',
    #         configuration=None,
    #         robot_config_yaml={
    #             'initial_map': 'L1',
    #         },
    #         node=self.node,
    #         zenoh_session=self.zenoh_session,
    #         fleet_handle=None,
    #         tf_buffer=tf_buffer
    #     )

    #     time.sleep(2)
    #     transform = robot_adapter.get_pose()
    #     assert transform is None
