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

import time

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.types import TFMessage
from free_fleet.utils import namespacify
from rclpy.time import Time
from tf2_ros import Buffer

import zenoh


def test_tf():
    # Open Zenoh Session
    session = zenoh.open(zenoh.Config())

    tf_buffer = Buffer()

    def tf_callback(sample: zenoh.Sample):
        transform = TFMessage.deserialize(sample.payload)
        for zt in transform.transforms:
            t = transform_stamped_to_ros2_msg(zt)
            tf_buffer.set_transform(t, 'free_fleet_examples_test_tf')

    # Subscribe to TF
    pub = session.declare_subscriber(
        namespacify('tf', 'turtlebot3_1'),
        tf_callback
    )

    transform_exists = False
    for i in range(10):
        try:
            tf_buffer.lookup_transform(
                'base_footprint',
                'map',
                Time()
            )
            transform_exists = True
        except Exception as err:
            print(f'Unable to get transform between base_footprint and '
                  f'map: {type(err)}: {err}')

        time.sleep(1)
    pub.undeclare()
    session.close()

    assert transform_exists
