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
import unittest

from free_fleet_adapter.nav1_robot_adapter import Nav1TfHandler
import rclpy
from tf2_ros import Buffer

import zenoh


class TestNav1TfHandler(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_nav1_tf_handler')
        cls.zenoh_session = zenoh.open(zenoh.Config())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.zenoh_session.close()
        rclpy.shutdown()

    def test_tf_does_not_exist(self):
        tf_buffer = Buffer()

        tf_handler = Nav1TfHandler(
            'missing_turtlebot3_1', self.zenoh_session, tf_buffer, self.node
        )

        transform_exists = False
        for i in range(10):
            transform = tf_handler.get_transform()
            if transform is not None:
                transform_exists = True
                break
            time.sleep(1)

        assert not transform_exists

    def test_tf_exists(self):
        tf_buffer = Buffer()

        tf_handler = Nav1TfHandler(
            'tb3_0', self.zenoh_session, tf_buffer, self.node
        )

        transform_exists = False
        for i in range(10):
            transform = tf_handler.get_transform()
            if transform is not None:
                transform_exists = True
                break
            time.sleep(1)

        assert transform_exists
