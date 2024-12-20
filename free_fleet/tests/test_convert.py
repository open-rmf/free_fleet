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

from free_fleet.convert import (
    transform_stamped_to_ros2_msg,
    transform_time_to_ros2_msg
)
from free_fleet.ros2_types import (
    GeometryMsgs_Quaternion,
    GeometryMsgs_Transform,
    GeometryMsgs_TransformStamped,
    GeometryMsgs_Vector3,
    Header,
    Time
)


def test_transform_time_to_ros2_msg():
    time = Time(sec=123, nanosec=456)
    converted = transform_time_to_ros2_msg(time)
    assert converted.sec == time.sec
    assert converted.nanosec == time.nanosec


def test_transform_stamped_to_ros2_msg():
    frame_id = 'test_header_frame_id'
    child_frame_id = 'test_child_frame_id'

    msg = GeometryMsgs_TransformStamped(
        header=Header(
            stamp=Time(sec=123, nanosec=456),
            frame_id=frame_id,
        ),
        child_frame_id=child_frame_id,
        transform=GeometryMsgs_Transform(
            translation=GeometryMsgs_Vector3(x=1, y=2, z=3),
            rotation=GeometryMsgs_Quaternion(),
        )
    )
    converted = transform_stamped_to_ros2_msg(msg)
    assert converted.header.stamp.sec == 123
    assert converted.header.stamp.nanosec == 456
    assert converted.header.frame_id == frame_id
    assert converted.child_frame_id == child_frame_id
    assert abs(converted.transform.translation.x - 1) <= 1e-6
    assert abs(converted.transform.translation.y - 2) <= 1e-6
    assert abs(converted.transform.translation.z - 3) <= 1e-6
    assert abs(converted.transform.rotation.x - 0) <= 1e-6
    assert abs(converted.transform.rotation.y - 0) <= 1e-6
    assert abs(converted.transform.rotation.z - 0) <= 1e-6
    assert abs(converted.transform.rotation.w - 1) <= 1e-6
