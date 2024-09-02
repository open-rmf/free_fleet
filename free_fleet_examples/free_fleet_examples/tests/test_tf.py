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
import zenoh

from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer

from free_fleet.types import TFMessage
from free_fleet.utils import namespace_topic


tf_buffer = Buffer()


def tf_callback(sample: zenoh.Sample):
    transform = TFMessage.deserialize(sample.payload)
    for zt in transform.transforms:
        time = Time(seconds=zt.header.stamp.sec, nanoseconds=zt.header.stamp.nanosec)
        t = TransformStamped()
        t.header.stamp = time.to_msg()
        t.header.stamp
        t.header.frame_id = zt.header.frame_id
        t.child_frame_id = zt.child_frame_id
        t.transform.translation.x = zt.transform.translation.x
        t.transform.translation.y = zt.transform.translation.y
        t.transform.translation.z = zt.transform.translation.z
        t.transform.rotation.x = zt.transform.rotation.x
        t.transform.rotation.y = zt.transform.rotation.y
        t.transform.rotation.z = zt.transform.rotation.z
        t.transform.rotation.w = zt.transform.rotation.w
        tf_buffer.set_transform(t, 'free_fleet_examples_test_tf')


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='tf_listener',
        description='Zenoh/ROS2 tf example')
    parser.add_argument('--zenoh-config', '-c', dest='config',
        metavar='FILE',
        type=str,
        help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')

    args = parser.parse_args()

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()
    # Open Zenoh Session
    session = zenoh.open(conf)

    # Subscribe to TF
    pub = session.declare_subscriber(
        namespace_topic("tf", args.namespace),
        tf_callback
    )

    try:
        while True:
            try:
                transform = tf_buffer.lookup_transform(
                    'base_footprint',
                    'map',
                    Time()
                )
                print(transform)
            except Exception as err:
                print(
                    f'Unable to get transform between base_footprint and map: {type(err)}: {err}'
                )

            time.sleep(1)
    except (KeyboardInterrupt):
        pass
    finally:
        pub.undeclare()
        session.close()


if __name__ == "__main__":
    main(sys.argv)
