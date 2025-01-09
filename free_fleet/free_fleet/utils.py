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

from free_fleet.ros2_types import (
    ActionMsgs_CancelGoal_Request,
    ActionMsgs_GoalInfo,
    Time,
    UUID
)


def namespacify(base_name: str, namespace: str, delimiter: str = '/') -> str:
    """
    Namespaces a base_name with namespace and delimiter.

    If no namespace is provided, returns the base_name. Otherwise, naively
    prefixes the base_name with namespace and delimiter, returning the result.
    """
    return base_name if not namespace else f'{namespace}{delimiter}{base_name}'


def make_nav2_cancel_all_goals_request() -> ActionMsgs_CancelGoal_Request:
    """
    Return a Nav2 CancelGoal request targeting all ongoing goals.

    According to action_msgs/srv/CancelGoal.srv, if goal ID is zero and
    timestamp is zero, cancel all goals.
    """
    return ActionMsgs_CancelGoal_Request(
        goal_info=ActionMsgs_GoalInfo(
            UUID(uuid=[0 for i in range(16)]),
            Time(sec=0, nanosec=0)
        )
    )


# https://github.com/eclipse-zenoh/zenoh-plugin-ros1/issues/131
def get_zenoh_name_of_ros1_topic(
    ros1_store,
    topic: str,
    msg_type: str
) -> str:
    # Get md5 and encode msg_type to construct zenoh topic
    msg_type_split = msg_type.split('/')
    msg_type_encoded = \
        '/'.join([msg_type_split[0], msg_type_split[2]]).encode('utf-8').hex()
    md5 = ros1_store.generate_msgdef(msg_type)[1]
    zenoh_topic = '/'.join([msg_type_encoded, md5, topic[1:]])

    return zenoh_topic
