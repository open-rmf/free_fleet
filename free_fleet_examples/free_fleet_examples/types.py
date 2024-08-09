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
class GeometryMsgs_TransformStamped(IdlStruct, typename="GeometryMsgs_TransformStamped"):
    header: Header
    frame_id: str
    transform: GeometryMsgs_Transform


@dataclass
class TFMessage(IdlStruct, typename="TFMessage"):
    transforms: pycdr2.types.sequence[GeometryMsgs_TransformStamped]
