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

from free_fleet.types import (
    ActionMsgs_CancelGoal_Request,
    ActionMsgs_GoalInfo,
    Time,
    UUID
)


def namespace_frame(frame: str, namespace: str) -> str:
    return f"{namespace}/{frame}" if len(namespace) != 0 else frame


def namespace_topic(topic: str , namespace: str) -> str:
    return f"{namespace}/{topic}" if len(namespace) != 0 else topic


def make_cancel_all_goals_request() -> ActionMsgs_CancelGoal_Request:
    return ActionMsgs_CancelGoal_Request(
        goal_info=ActionMsgs_GoalInfo(
            UUID(uuid=[0 for i in range(16)]),
            Time(sec=0, nanosec=0)
        )
    )
