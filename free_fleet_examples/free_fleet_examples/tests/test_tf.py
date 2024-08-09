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

from free_fleet_examples.types import TFMessage


def tf_callback(sample: zenoh.Sample):
    transform = TFMessage.deserialize(sample.payload)
    print("got a tf!")


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

    # Declare a subscriber for feedbacks
    tf_topic = 'tf'
    if len(args.namespace) != 0:
        tf_topic = args.namespace + '/' + tf_topic
    pub = session.declare_subscriber(tf_topic, tf_callback)

    try:
        while True:
            time.sleep(1)
    except (KeyboardInterrupt):
        pass
    finally:
        pub.undeclare()
        session.close()


if __name__ == "__main__":
    main(sys.argv)
