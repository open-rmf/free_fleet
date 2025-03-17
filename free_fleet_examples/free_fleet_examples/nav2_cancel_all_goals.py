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

from free_fleet.ros2_types import ActionMsgs_CancelGoal_Response
from free_fleet.utils import make_nav2_cancel_all_goals_request, namespacify

import zenoh


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog='cancel_all_goals',
        description='Zenoh/ROS2 cancel all goals example')
    parser.add_argument('--zenoh-config', '-c', dest='config', metavar='FILE',
                        type=str, help='A configuration file.')
    parser.add_argument('--namespace', '-n', type=str, default='')

    args = parser.parse_args()

    # Create Zenoh Config from file if provoded, or a default one otherwise
    conf = zenoh.Config.from_file(args.config) \
        if args.config is not None else zenoh.Config()
    # Open Zenoh Session
    session = zenoh.open(conf)

    req = make_nav2_cancel_all_goals_request()

    # Send the query with the serialized request
    replies = session.get(
        namespacify(
            'navigate_to_pose/_action/cancel_goal',
            args.namespace
        ),
        payload=req.serialize()
    )
    # Zenoh could get several replies for a request (e.g. from several
    # 'Service Servers' using the same name)
    for reply in replies:
        # Deserialize the response
        rep = ActionMsgs_CancelGoal_Response.deserialize(
            reply.ok.payload.to_bytes()
        )
        print('Return code: %d' % rep.return_code)

    session.close()


if __name__ == '__main__':
    main(sys.argv)
