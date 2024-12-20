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
    return f'{namespace}{delimiter}{base_name}' if len(namespace) != 0 \
        else base_name


def make_cancel_all_goals_request() -> ActionMsgs_CancelGoal_Request:
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
