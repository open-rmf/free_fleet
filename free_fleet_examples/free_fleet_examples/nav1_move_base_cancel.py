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


import argparse
import sys

from free_fleet_adapter.nav1_robot_adapter import Nav1MoveBaseHandler
import rclpy

import zenoh


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    node = rclpy.node.Node('nav1_move_base_cancel')

    parser = argparse.ArgumentParser(
        prog='nav1_move_base_cancel',
        description='Zenoh/ROS1 move_base cancel example')
    parser.add_argument('--zenoh-config', '-c', dest='config', metavar='FILE',
                        type=str, help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')
    parser.add_argument('--goal-id', '-g', type=str)

    args = parser.parse_args(args_without_ros[1:])

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()

    zenoh.try_init_log_from_env()

    # Open Zenoh Session
    with zenoh.open(conf) as session:
        info = session.info
        print(f'zid: {info.zid()}')
        print(f'routers: {info.routers_zid()}')
        print(f'peers: {info.peers_zid()}')

        move_base_handler = Nav1MoveBaseHandler(args.namespace, session, node)
        move_base_handler.cancel_navigation(
            args.goal_id
        )


if __name__ == '__main__':
    main(sys.argv)
