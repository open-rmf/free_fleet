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

import argparse
import sys
import time
import zenoh

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


def feedback_callback(sample: zenoh.Sample):
    # Deserialize the message
    feedback = NavigateToPose_Feedback.deserialize(sample.payload)
    print(f"Distance remaining: {feedback.distance_remaining}")


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='navigate_to_pose_action_client',
        description='Zenoh/ROS2 navigate_to_pose_action_client example')
    parser.add_argument('--zenoh-config', '-c', dest='config',
        metavar='FILE',
        type=str,
        help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')
    parser.add_argument('--frame-id', '-f', type=str, default='odom')
    parser.add_argument('-x', type=float)
    parser.add_argument('-y', type=float)

    args = parser.parse_args()

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()
    # Open Zenoh Session
    session = zenoh.open(conf)

    # Declare a subscriber for feedbacks
    feedback_topic = 'navigate_to_pose/_action/feedback'
    if len(args.namespace) != 0:
        feedback_topic = args.namespace + '/' + feedback_topic
    pub = session.declare_subscriber(feedback_topic, feedback_callback)

    time = Time(sec=0, nanosec=0)
    header = Header(stamp=time, frame_id=args.frame_id)

    position = GeometryMsgs_Point(x=args.x, y=args.y, z=0)
    orientation = GeometryMsgs_Quaternion()
    pose = GeometryMsgs_Pose(position=position, orientation=orientation)

    pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

    goal_id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    req = NavigateToPose_SendGoal_Request(
        goal_id=goal_id,
        pose=pose_stamped,
        behavior_tree=""
    )

    # Send the query with the serialized request
    print('Sending goal')
    replies = session.get(
        'navigate_to_pose/_action/send_goal',
        zenoh.Queue(),
        value=req.serialize()
    )

    # Zenoh could get several replies for a request (e.g. from several
    # "Service Servers" using the same name)
    for reply in replies.receiver:
        if not reply.ok:
            print("Reply was not ok!")
            continue
        print('handling a reply!')
        # Deserialize the response
        rep = NavigateToPose_SendGoal_Response.deserialize(reply.ok.payload)
        if not rep.accepted:
            print('Goal rejected :(')
            return

    print('Goal accepted by server, waiting for result')

    req = NavigateToPose_GetResult_Request(goal_id)
    # Send the query with the serialized request
    replies = session.get(
        'navigate_to_pose/_action/get_result',
        zenoh.Queue(),
        value=req.serialize()
    )
    # Zenoh could get several replies for a request (e.g. from several "Service Servers" using the same name)
    for reply in replies.receiver:
        # Deserialize the response
        rep = NavigateToPose_GetResult_Response.deserialize(reply.ok.payload)
        # print('Result: {0}'.format(rep.sequence))
        print(f"Result: {rep.status}")

    session.close()


if __name__ == "__main__":
    main(sys.argv)
