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

from free_fleet.ros2_types import (
    GeometryMsgs_Point,
    GeometryMsgs_Pose,
    GeometryMsgs_PoseStamped,
    GeometryMsgs_Quaternion,
    Header,
    Time,

    GoalStatus,
    DockRobot_Feedback,
    DockRobot_GetResult_Request,
    DockRobot_GetResult_Response,
    DockRobot_SendGoal_Request,
    DockRobot_SendGoal_Response,
)
from free_fleet.utils import namespacify

import numpy as np
import zenoh


def feedback_callback(sample: zenoh.Sample):
    if not sample.payload:
        print('Received empty feedback')
        return

    feedback = DockRobot_Feedback.deserialize(sample.payload.to_bytes())
    print(f'Docking state: {feedback.state} / Time elapsed: {feedback.docking_time}')


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='dock_robot_action_client',
        description='Zenoh/ROS2 dock_robot_action_client example')
    parser.add_argument('--zenoh-config', '-c', dest='config', metavar='FILE',
                        type=str, help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')
    parser.add_argument('--frame-id', '-f', type=str, default='map')
    parser.add_argument('--dock', type=str, default='')
    parser.add_argument('--stage', type=bool, default=True)
    parser.add_argument('--staging_time', type=float, default=1000.0)

    args = parser.parse_args()

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()
    # Open Zenoh Session
    session = zenoh.open(conf)

    # Declare a subscriber for feedbacks
    feedback_sub = session.declare_subscriber(
        namespacify('dock_robot/_action/feedback', args.namespace),
        feedback_callback
    )

    # create fake pose
    stamp = Time(sec=0, nanosec=0)
    header = Header(stamp=stamp, frame_id=args.frame_id)

    position = GeometryMsgs_Point(x=0, y=0, z=0)
    orientation = GeometryMsgs_Quaternion()
    pose = GeometryMsgs_Pose(position=position, orientation=orientation)

    pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

    goal_id = np.random.randint(0, 255, size=(16)).astype('uint8').tolist()
    print(f'{goal_id}: sending goal to dock to {args.dock} [navigate to staging pose: {args.stage}]')
    req = DockRobot_SendGoal_Request(
        goal_id=goal_id,
        use_dock_id=True,
        dock_id=args.dock,
        dock_pose=pose_stamped,
        dock_type='',
        max_staging_time=args.staging_time,
        navigate_to_staging_pose=args.stage,
    )

    # Send the query with the serialized request
    print('Sending goal')
    replies = session.get(
        namespacify('dock_robot/_action/send_goal', args.namespace),
        payload=req.serialize(),
    )

    # Zenoh could get several replies for a request (e.g. from several
    # 'Service Servers' using the same name)
    for reply in replies:
        if not reply.ok:
            print('Reply was not ok!')
            continue
        print('handling a reply!')
        # Deserialize the response
        rep = DockRobot_SendGoal_Response.deserialize(
            reply.ok.payload.to_bytes()
        )
        if not rep.accepted:
            print('Goal rejected')
            return

    print('Goal accepted by server, waiting for result')

    req = DockRobot_GetResult_Request(goal_id)
    try:
        while True:
            # Send the query with the serialized request
            replies = session.get(
                namespacify(
                    'dock_robot/_action/get_result',
                    args.namespace),
                payload=req.serialize(),
                timeout=5.5
            )

            # Zenoh could get several replies for a request (e.g. from several
            # 'Service Servers' using the same name)
            for reply in replies:
                try:
                    # Deserialize the response
                    rep = DockRobot_GetResult_Response.deserialize(
                        reply.ok.payload.to_bytes() if reply.ok else reply.err.payload.to_bytes()
                    )
                    print(f'Result: {rep.status}')
                    if rep.status == GoalStatus.STATUS_ABORTED.value:
                        print(
                            'Received (ERROR: "Plan aborted by '
                            'planner_server")'
                        )
                        break
                    if rep.status == GoalStatus.STATUS_SUCCEEDED.value:
                        break
                except Exception as e:
                    print(e)
                    print("Received (ERROR: '{}')".format(
                        reply.err.payload.to_string()))
                    continue

            time.sleep(1)
    except (KeyboardInterrupt):
        pass
    finally:
        feedback_sub.undeclare()
        session.close()


if __name__ == '__main__':
    main(sys.argv)
