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

from free_fleet_adapter.nav2_robot_adapter import TfListener
from rclpy.time import Time
from tf2_ros import Buffer

import zenoh


def test_tf():
    zenoh.try_init_log_from_env()
    with zenoh.open(zenoh.Config()) as session:
        tf_buffer = Buffer()

        listener = TfListener('turtlebot3_1', session, tf_buffer)
        listener

        transform_exists = False
        for i in range(10):
            try:
                tf_buffer.lookup_transform(
                    'turtlebot3_1/base_footprint',
                    'turtlebot3_1/map',
                    Time()
                )
                transform_exists = True
                break
            except Exception as err:
                print(f'Unable to get transform between base_footprint and '
                      f'map: {type(err)}: {err}')

            time.sleep(1)

        assert transform_exists
