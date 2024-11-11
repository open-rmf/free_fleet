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

from free_fleet_adapter.nav2_robot_adapter import TfHandler
from tf2_ros import Buffer

import zenoh


def test_tf_does_not_exist():
    zenoh.try_init_log_from_env()
    with zenoh.open(zenoh.Config()) as session:
        tf_buffer = Buffer()

        tf_handler = TfHandler('missing_turtlebot3_1', session, tf_buffer)

        transform_exists = False
        for i in range(10):
            transform = tf_handler.get_transform()
            if transform is not None:
                transform_exists = True
                break
            time.sleep(1)

        assert not transform_exists


def test_tf_exists():
    zenoh.try_init_log_from_env()
    with zenoh.open(zenoh.Config()) as session:
        tf_buffer = Buffer()

        tf_handler = TfHandler('turtlebot3_1', session, tf_buffer)

        transform_exists = False
        for i in range(10):
            transform = tf_handler.get_transform()
            if transform is not None:
                transform_exists = True
                break
            time.sleep(1)

        assert transform_exists
