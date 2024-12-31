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

from free_fleet.ros1_types import ROS1_STORE, TFMessage
from free_fleet.utils import (
    get_zenoh_name_of_ros1_topic,
    make_nav2_cancel_all_goals_request,
    namespacify
)


def test_namespacify():
    assert namespacify('base_name', '') == 'base_name'
    assert namespacify('base_name', 'namespace') == 'namespace/base_name'
    assert namespacify('base_name', 'namespace', '!') == 'namespace!base_name'


def test_make_nav2_cancel_all_goals_request():
    request = make_nav2_cancel_all_goals_request()
    assert request.goal_info.goal_id.uuid == [0 for i in range(16)]
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_get_zenoh_name_of_ros1_topic():
    test_topic = '/test/topic'
    zenoh_topic = get_zenoh_name_of_ros1_topic(
        ROS1_STORE,
        test_topic,
        TFMessage.type_name
    )
    assert zenoh_topic.endswith(test_topic)

    topic_parts = zenoh_topic.split('/')
    assert len(topic_parts) == 4
