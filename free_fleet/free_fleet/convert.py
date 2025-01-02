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


from builtin_interfaces.msg import Time as RclTimeMsg
from free_fleet.ros1_types import (
    Time as Ros1Time,
    TransformStamped as Ros1TransformStamped
)
from free_fleet.ros2_types import (
    GeometryMsgs_TransformStamped as Ros2TransformStamped,
    Time as Ros2Time
)
from geometry_msgs.msg import TransformStamped
from rclpy import time as rclpyTime


def transform_time_to_ros2_msg(
    msg: Ros1Time.msg_type | Ros2Time
) -> RclTimeMsg:
    time = rclpyTime.Time(
        seconds=msg.sec,
        nanoseconds=msg.nanosec
    )
    return time.to_msg()


def transform_stamped_to_ros2_msg(
    msg: Ros1TransformStamped.msg_type | Ros2TransformStamped
) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = transform_time_to_ros2_msg(msg.header.stamp)
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = msg.child_frame_id
    t.transform.translation.x = msg.transform.translation.x
    t.transform.translation.y = msg.transform.translation.y
    t.transform.translation.z = msg.transform.translation.z
    t.transform.rotation.x = msg.transform.rotation.x
    t.transform.rotation.y = msg.transform.rotation.y
    t.transform.rotation.z = msg.transform.rotation.z
    t.transform.rotation.w = msg.transform.rotation.w
    return t
