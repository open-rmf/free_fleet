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

from enum import Enum
import pycdr2
from pycdr2 import IdlStruct
from dataclasses import dataclass


@dataclass
class Time(IdlStruct, typename="Time"):
    sec: pycdr2.types.int32
    nanosec: pycdr2.types.uint32


@dataclass
class Duration(IdlStruct, typename="Duration"):
    sec: pycdr2.types.int32
    nanosec: pycdr2.types.uint32


@dataclass
class Header(IdlStruct, typename="Header"):
    stamp: Time
    frame_id: str


@dataclass
class GeometryMsgs_Point(IdlStruct, typename="GeometryMsgs_Point"):
    x: pycdr2.types.float64
    y: pycdr2.types.float64
    z: pycdr2.types.float64


@dataclass
class GeometryMsgs_Quaternion(IdlStruct, typename="GeometryMsgs_Quaternion"):
    x: pycdr2.types.float64 = 0
    y: pycdr2.types.float64 = 0
    z: pycdr2.types.float64 = 0
    w: pycdr2.types.float64 = 1


@dataclass
class GeometryMsgs_Pose(IdlStruct, typename="GeometryMsgs_Pose"):
    position: GeometryMsgs_Point
    orientation: GeometryMsgs_Quaternion


@dataclass
class GeometryMsgs_PoseStamped(IdlStruct, typename="GeometryMsgs_PoseStamped"):
    header: Header
    pose: GeometryMsgs_Pose


@dataclass
class NavigateToPose_SendGoal_Request(
    IdlStruct,
    typename="NavigateToPose_SendGoal_Request"
):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]
    pose: GeometryMsgs_PoseStamped
    behavior_tree: str


@dataclass
class NavigateToPose_SendGoal_Response(
    IdlStruct,
    typename="NavigateToPose_SendGoal_Response"
):
    accepted: bool
    stamp: Time


@dataclass
class NavigateToPose_GetResult_Request(
    IdlStruct,
    typename="NavigateToPose_GetResult_Request"
):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]


class GoalStatus(Enum):
    STATUS_UNKNOWN: pycdr2.types.int8 = 0
    STATUS_ACCEPTED: pycdr2.types.int8 = 1
    STATUS_EXECUTING: pycdr2.types.int8 = 2
    STATUS_CANCELING: pycdr2.types.int8 = 3
    STATUS_SUCCEEDED: pycdr2.types.int8 = 4
    STATUS_CANCELED: pycdr2.types.int8 = 5
    STATUS_ABORTED: pycdr2.types.int8 = 6


@dataclass
class NavigateToPose_GetResult_Response(
    IdlStruct,
    typename="NavigateToPose_GetResult_Response"
):
    status: pycdr2.types.int8


@dataclass
class NavigateToPose_Feedback(IdlStruct, typename="NavigateToPose_Feedback"):
    goal_id: pycdr2.types.array[pycdr2.types.uint8, 16]
    current_pose: GeometryMsgs_PoseStamped
    navigation_time: Duration
    estimated_time_remaining: Duration
    number_of_recoveries: pycdr2.types.int16
    distance_remaining: pycdr2.types.float32


@dataclass
class GeometryMsgs_Vector3(IdlStruct, typename="GeometryMsgs_Vector3"):
    x: pycdr2.types.float64
    y: pycdr2.types.float64
    z: pycdr2.types.float64


@dataclass
class GeometryMsgs_Transform(IdlStruct, typename="GeometryMsgs_Transform"):
    translation: GeometryMsgs_Vector3
    rotation: GeometryMsgs_Quaternion


@dataclass
class GeometryMsgs_TransformStamped(
    IdlStruct,
    typename="GeometryMsgs_TransformStamped"
):
    header: Header
    child_frame_id: str
    transform: GeometryMsgs_Transform


@dataclass
class TFMessage(IdlStruct, typename="TFMessage"):
    transforms: pycdr2.types.sequence[GeometryMsgs_TransformStamped]


@dataclass
class UUID(IdlStruct, typename="UUID"):
    uuid: pycdr2.types.array[pycdr2.types.uint8, 16]


@dataclass
class ActionMsgs_GoalInfo(IdlStruct, typename="ActionMsgs_GoalInfo"):
    goal_id: UUID
    stamp: Time


@dataclass
class ActionMsgs_CancelGoal_Request(
    IdlStruct,
    typename="ActionMsgs_CancelGoal_Request"
):
    goal_info: ActionMsgs_GoalInfo


@dataclass
class ActionMsgs_CancelGoal_Response(
    IdlStruct,
    typename="ActionMsgs_CancelGoal_Response"
):
    return_code: pycdr2.types.int8
    goals_canceling: pycdr2.types.sequence[ActionMsgs_GoalInfo]


@dataclass
class SensorMsgs_BatteryState(IdlStruct, typename="SensorMsgs_BatteryState"):
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
