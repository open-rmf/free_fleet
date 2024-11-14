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
import time

from free_fleet_adapter.nav2_robot_adapter import Nav2TfHandler
from tf2_ros import Buffer

import zenoh


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='get_tf',
        description='Zenoh/ROS2 tf example')
    parser.add_argument('--zenoh-config', '-c', dest='config', metavar='FILE',
                        type=str, help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')
    parser.add_argument(
        '-b', '--base-footprint-frame', default='base_footprint'
    )
    parser.add_argument('-m', '--map-frame', default='map')

    args = parser.parse_args()

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()

    zenoh.try_init_log_from_env()

    tf_buffer = Buffer()

    # Open Zenoh Session
    with zenoh.open(conf) as session:
        info = session.info
        print(f'zid: {info.zid()}')
        print(f'routers: {info.routers_zid()}')
        print(f'peers: {info.peers_zid()}')

        tf_handler = Nav2TfHandler('turtlebot3_1', session, tf_buffer)

        try:
            while True:
                transform = tf_handler.get_transform()
                if transform is None:
                    print('Unable to get transform between base_footprint and'
                          ' map')
                else:
                    print(transform)
                time.sleep(1)
        except (KeyboardInterrupt):
            pass
        finally:
            session.close()


if __name__ == '__main__':
    main(sys.argv)
