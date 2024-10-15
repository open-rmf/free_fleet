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

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.types import TFMessage
from free_fleet.utils import namespacify
from rclpy.time import Time
from tf2_ros import Buffer

import zenoh


tf_buffer = Buffer()


def tf_callback(sample: zenoh.Sample):
    transform = TFMessage.deserialize(sample.payload.to_bytes())
    for zt in transform.transforms:
        t = transform_stamped_to_ros2_msg(zt)
        tf_buffer.set_transform(t, 'free_fleet_examples_test_tf')


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='tf_listener',
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

    # Open Zenoh Session
    with zenoh.open(conf) as session:
        info = session.info
        print(f"zid: {info.zid()}")
        print(f"routers: {info.routers_zid()}")
        print(f"peers: {info.peers_zid()}")

    # Subscribe to TF
    pub = session.declare_subscriber(
        namespacify('tf', args.namespace),
        tf_callback
    )

    try:
        while True:
            try:
                transform = tf_buffer.lookup_transform(
                    args.base_footprint_frame,
                    args.map_frame,
                    Time()
                )
                print(transform)
            except Exception as err:
                print(f'Unable to get transform between base_footprint and '
                      f'map: {type(err)}: {err}')

            time.sleep(1)
    except (KeyboardInterrupt):
        pass
    finally:
        pub.undeclare()
        session.close()


if __name__ == '__main__':
    main(sys.argv)
