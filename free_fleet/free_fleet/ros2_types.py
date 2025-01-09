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

from dataclasses import dataclass
from enum import Enum

import pycdr2
from pycdr2 import IdlStruct


# https://github.com/ros2/rcl_interfaces/blob/rolling/builtin_interfaces/msg/Time.msg
@dataclass
class Time(IdlStruct):
    sec: pycdr2.types.int32
    nanosec: pycdr2.types.uint32


# https://github.com/ros2/rcl_interfaces/blob/rolling/builtin_interfaces/msg/Duration.msg
@dataclass
class Duration(IdlStruct):
    sec: pycdr2.types.int32
    nanosec: pycdr2.types.uint32


# https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg
@dataclass
class Header(IdlStruct):
    stamp: Time
    frame_id: str


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Point.msg
@dataclass
class GeometryMsgs_Point(IdlStruct):
    x: pycdr2.types.float64
    y: pycdr2.types.float64
    z: pycdr2.types.float64


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Quaternion.msg
@dataclass
class GeometryMsgs_Quaternion(IdlStruct):
    x: pycdr2.types.float64 = 0
    y: pycdr2.types.float64 = 0
    z: pycdr2.types.float64 = 0
    w: pycdr2.types.float64 = 1


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Pose.msg
@dataclass
class GeometryMsgs_Pose(IdlStruct):
    position: GeometryMsgs_Point
    orientation: GeometryMsgs_Quaternion


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
@dataclass
class GeometryMsgs_PoseStamped(IdlStruct):
    header: Header
    pose: GeometryMsgs_Pose


# https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
@dataclass
class NavigateToPose_SendGoal_Request(IdlStruct):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]
    pose: GeometryMsgs_PoseStamped
    behavior_tree: str


# https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
@dataclass
class NavigateToPose_SendGoal_Response(IdlStruct):
    accepted: bool
    stamp: Time


# https://design.ros2.org/articles/actions.html#get-result-service
@dataclass
class NavigateToPose_GetResult_Request(IdlStruct):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]


# https://github.com/ros2/rcl/blob/rolling/rcl_action/include/rcl_action/types.h
class GoalStatus(Enum):
    STATUS_UNKNOWN: pycdr2.types.int8 = 0
    STATUS_ACCEPTED: pycdr2.types.int8 = 1
    STATUS_EXECUTING: pycdr2.types.int8 = 2
    STATUS_CANCELING: pycdr2.types.int8 = 3
    STATUS_SUCCEEDED: pycdr2.types.int8 = 4
    STATUS_CANCELED: pycdr2.types.int8 = 5
    STATUS_ABORTED: pycdr2.types.int8 = 6


# https://design.ros2.org/articles/actions.html#get-result-service
# https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
@dataclass
class NavigateToPose_GetResult_Response(IdlStruct):
    status: pycdr2.types.int8


# https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
@dataclass
class NavigateToPose_Feedback(IdlStruct):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]
    current_pose: GeometryMsgs_PoseStamped
    navigation_time: Duration
    estimated_time_remaining: Duration
    number_of_recoveries: pycdr2.types.int16
    distance_remaining: pycdr2.types.float32


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Vector3.msg
@dataclass
class GeometryMsgs_Vector3(IdlStruct):
    x: pycdr2.types.float64
    y: pycdr2.types.float64
    z: pycdr2.types.float64


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Transform.msg
@dataclass
class GeometryMsgs_Transform(IdlStruct):
    translation: GeometryMsgs_Vector3
    rotation: GeometryMsgs_Quaternion


# https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
@dataclass
class GeometryMsgs_TransformStamped(IdlStruct):
    header: Header
    child_frame_id: str
    transform: GeometryMsgs_Transform


# https://github.com/ros2/geometry2/blob/rolling/tf2_msgs/msg/TFMessage.msg
@dataclass
class TFMessage(IdlStruct):
    transforms: pycdr2.types.sequence[GeometryMsgs_TransformStamped]


# https://github.com/ros2/unique_identifier_msgs/blob/rolling/msg/UUID.msg
@dataclass
class UUID(IdlStruct):
    uuid: pycdr2.types.array[pycdr2.types.uint8, 16]


# https://github.com/ros2/rcl_interfaces/blob/rolling/action_msgs/msg/GoalInfo.msg
@dataclass
class ActionMsgs_GoalInfo(IdlStruct):
    goal_id: UUID
    stamp: Time


# https://github.com/ros2/rcl_interfaces/blob/rolling/action_msgs/srv/CancelGoal.srv
@dataclass
class ActionMsgs_CancelGoal_Request(IdlStruct):
    goal_info: ActionMsgs_GoalInfo


# https://github.com/ros2/rcl_interfaces/blob/rolling/action_msgs/srv/CancelGoal.srv
@dataclass
class ActionMsgs_CancelGoal_Response(IdlStruct):
    return_code: pycdr2.types.int8
    goals_canceling: pycdr2.types.sequence[ActionMsgs_GoalInfo]


# https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg
@dataclass
class SensorMsgs_BatteryState(IdlStruct):
    header: Header
    voltage: pycdr2.types.float32
    temperature: pycdr2.types.float32
    current: pycdr2.types.float32
    charge: pycdr2.types.float32
    capacity: pycdr2.types.float32
    design_capacity: pycdr2.types.float32
    percentage: pycdr2.types.float32
    power_supply_status: pycdr2.types.uint8
    power_supply_health: pycdr2.types.uint8
    power_supply_technology: pycdr2.types.uint8
    present: bool
    cell_voltage: pycdr2.types.sequence[pycdr2.types.float32]
    cell_temperature: pycdr2.types.sequence[pycdr2.types.float32]
    location: str
    serial_number: str
