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

from rosbags.typesys import get_types_from_msg, get_typestore, Stores


ROS1_STORE = get_typestore(Stores.ROS1_NOETIC)


# https://github.com/ros/geometry2/blob/noetic-devel/tf2_msgs/msg/TFMessage.msg
TFMessage_Definition = """
geometry_msgs/TransformStamped[] transforms
"""
ROS1_STORE.register(
    get_types_from_msg(TFMessage_Definition, 'tf2_msgs/msg/TFMessage')
)


class TFMessage:
    type_name = 'tf2_msgs/msg/TFMessage'
    msg_type = ROS1_STORE.types[type_name]


class Transform:
    type_name = 'geometry_msgs/msg/Transform'
    msg_type = ROS1_STORE.types[type_name]


class TransformStamped:
    type_name = 'geometry_msgs/msg/TransformStamped'
    msg_type = ROS1_STORE.types[type_name]


class Vector3:
    type_name = 'geometry_msgs/msg/Vector3'
    msg_type = ROS1_STORE.types[type_name]


class Time:
    type_name = 'builtin_interfaces/msg/Time'
    msg_type = ROS1_STORE.types[type_name]


class Header:
    type_name = 'std_msgs/msg/Header'
    msg_type = ROS1_STORE.types[type_name]


class Pose:
    type_name = 'geometry_msgs/msg/Pose'
    msg_type = ROS1_STORE.types[type_name]


class Point:
    type_name = 'geometry_msgs/msg/Point'
    msg_type = ROS1_STORE.types[type_name]


class Quaternion:
    type_name = 'geometry_msgs/msg/Quaternion'
    msg_type = ROS1_STORE.types[type_name]


class PoseStamped:
    type_name = 'geometry_msgs/msg/PoseStamped'
    msg_type = ROS1_STORE.types[type_name]


class GoalID:
    type_name = 'actionlib_msgs/msg/GoalID'
    msg_type = ROS1_STORE.types[type_name]


class GoalStatus:
    type_name = 'actionlib_msgs/msg/GoalStatus'
    msg_type = ROS1_STORE.types[type_name]


class GoalStatusArray:
    type_name = 'actionlib_msgs/msg/GoalStatusArray'
    msg_type = ROS1_STORE.types[type_name]


class BatteryState:
    type_name = 'sensor_msgs/msg/BatteryState'
    msg_type = ROS1_STORE.types[type_name]
