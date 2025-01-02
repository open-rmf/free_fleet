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
from free_fleet.ros1_types import (
    Header as Nav1Header,
    Quaternion as Nav1Quaternion,
    Time as Nav1Time,
    Transform as Nav1Transform,
    TransformStamped as Nav1TransformStamped,
    Vector3 as Nav1Vector3
)
from free_fleet.ros2_types import (
    GeometryMsgs_Quaternion as Nav2Quaternion,
    GeometryMsgs_Transform as Nav2Transform,
    GeometryMsgs_TransformStamped as Nav2TransformStamped,
    GeometryMsgs_Vector3 as Nav2Vector3,
    Header as Nav2Header,
    Time as Nav2Time
)


def test_nav2_transform_time_to_ros2_msg():
    time = Nav2Time(sec=123, nanosec=456)
    converted = transform_time_to_ros2_msg(time)
    assert converted.sec == time.sec
    assert converted.nanosec == time.nanosec


def test_nav2_transform_stamped_to_ros2_msg():
    frame_id = 'test_header_frame_id'
    child_frame_id = 'test_child_frame_id'

    msg = Nav2TransformStamped(
        header=Nav2Header(
            stamp=Nav2Time(sec=123, nanosec=456),
            frame_id=frame_id,
        ),
        child_frame_id=child_frame_id,
        transform=Nav2Transform(
            translation=Nav2Vector3(x=1, y=2, z=3),
            rotation=Nav2Quaternion(),
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


def test_nav1_transform_time_to_ros2_msg():
    time = Nav1Time.msg_type(sec=123, nanosec=456)
    converted = transform_time_to_ros2_msg(time)
    assert converted.sec == time.sec
    assert converted.nanosec == time.nanosec


def test_nav1_transform_stamped_to_ros2_msg():
    frame_id = 'test_header_frame_id'
    child_frame_id = 'test_child_frame_id'

    msg = Nav1TransformStamped.msg_type(
        header=Nav1Header.msg_type(
            seq=0,
            stamp=Nav1Time.msg_type(sec=123, nanosec=456),
            frame_id=frame_id,
        ),
        child_frame_id=child_frame_id,
        transform=Nav1Transform.msg_type(
            translation=Nav1Vector3.msg_type(x=1, y=2, z=3),
            rotation=Nav1Quaternion.msg_type(x=0, y=0, z=0, w=1),
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
