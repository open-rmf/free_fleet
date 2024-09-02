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

import sys
import argparse
import yaml
import time
import threading
import asyncio
import nudged
from typing import Optional
import zenoh

from free_fleet.types import (
    GeometryMsgs_Point,
    GeometryMsgs_Pose,
    GeometryMsgs_PoseStamped,
    GeometryMsgs_Quaternion,
    Header,
    NavigateToPose_Feedback,
    NavigateToPose_GetResult_Request,
    NavigateToPose_GetResult_Response,
    NavigateToPose_SendGoal_Request,
    NavigateToPose_SendGoal_Response,
    TFMessage,
    Time,
)
from free_fleet.utils import namespace_topic

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.duration import Duration

import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter import Transformation

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------
def compute_transforms(level, coords, node=None):
    """Get transforms between RMF and robot coordinates."""
    rmf_coords = coords['rmf']
    robot_coords = coords['robot']
    tf = nudged.estimate(rmf_coords, robot_coords)
    if node:
        mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
        node.get_logger().info(
            f"Transformation error estimate for {level}: {mse}"
        )

    return Transformation(
        tf.get_rotation(),
        tf.get_scale(),
        tf.get_translation()
    )

# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                        help="URI of the api server to transmit state and task information.")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    parser.add_argument(
        '--zenoh-config',
        type=str,
        help='Path to custom zenoh configuration file to be used, if not '
        'provided the default config will be used'
    )
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    if args.server_uri == '':
        server_uri = None
    else:
        server_uri = args.server_uri

    fleet_config.server_uri = server_uri

    # Configure the transforms between robot and RMF frames
    for level, coords in config_yaml['reference_coordinates'].items():
        tf = compute_transforms(level, coords, node)
        fleet_config.add_robot_coordinates_transformation(level, tf)

    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Initialize zenoh
    zenoh_config = zenoh.Config.from_file(args.zenoh_config) \
        if args.zenoh_config is not None else zenoh.Config()
    zenoh_session = zenoh.open(zenoh_config)

    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, zenoh_session, fleet_handle
        )

    update_period = 1.0/config_yaml['rmf_fleet'].get(
        'robot_state_update_frequency', 10.0
    )

    def update_loop():
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            now = node.get_clock().now()

            # Update all the robots in parallel using a thread pool
            update_jobs = []
            for robot in robots.values():
                update_jobs.append(update_robot(robot))

            asyncio.get_event_loop().run_until_complete(
                asyncio.wait(update_jobs)
            )

            next_wakeup = now + Duration(nanoseconds=update_period*1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()
    zenoh_session.close()


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        zenoh_session,
        fleet_handle
    ):
        self.name = name
        self.execution = None
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.zenoh_session = zenoh_session
        self.fleet_handle = fleet_handle

        self.tf_sub = None
        self.navigate_to_pose_action

    def __del__(self):
        if

    def update(self, state):
        activity_identifier = None
        if self.execution:
            if self.api.is_command_completed():
                self.execution.finished()
                self.execution = None
            else:
                activity_identifier = self.execution.identifier

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            )
        )

    def navigate(self, destination, execution):
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        self.api.navigate(
            self.name,
            destination.position,
            destination.map,
            destination.speed_limit
        )

    def stop(self, activity):
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self.api.stop(self.name)

    def execute_action(self, category: str, description: dict, execution):
        ''' Trigger a custom action you would like your robot to perform.
        You may wish to use RobotAPI.start_activity to trigger different
        types of actions to your robot.'''
        self.execution = execution
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


@parallel
def update_robot(robot: RobotAdapter):
    data = robot.api.get_data(robot.name)
    if data is None:
        return

    state = rmf_easy.RobotState(
        data.map,
        data.position,
        data.battery_soc
    )

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name,
            state,
            robot.configuration,
            robot.make_callbacks()
        )
        return

    robot.update(state)


if __name__ == '__main__':
    main(sys.argv)
