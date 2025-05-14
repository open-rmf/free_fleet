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

from abc import ABC, abstractmethod
from threading import Lock
from typing import Annotated

import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import ActivityIdentifier


class ExecutionHandle:

    def __init__(self, execution: rmf_easy.CommandExecution | None):
        self.execution = execution
        self.goal_id = None
        self.action = None
        self.mutex = Lock()
        self.mutex.acquire(blocking=True)

    def set_goal_id(self, goal_id):
        self.goal_id = goal_id
        self.mutex.release()

    def set_action(self, action):
        self.action = action
        self.mutex.release()

    @property
    def activity(self) -> ActivityIdentifier | None:
        # Move the execution reference into a separate variable just in case
        # another thread modifies self.execution while we're still using it.
        execution = self.execution
        if execution is not None:
            return execution.identifier
        return None


class RobotAdapter(ABC):
    """Abstract Robot Adapter to be used by the free fleet adapter."""

    def __init__(
        self,
        name: str,
        node,
        fleet_handle
    ):
        self.name = name
        self.node = node
        self.fleet_handle = fleet_handle
        self.update_handle = None

    """
    This method returns the battery state of charge as a float, with value
    between 0 and 1.0.
    """
    @abstractmethod
    def get_battery_soc(self) -> float:
        ...

    """
    This method returns the name of the current map that the robot is
    localized on.
    """
    @abstractmethod
    def get_map_name(self) -> str:
        ...

    """
    This method returns the last known 2D position in meters and orientation
    (yaw) of the robot in radians as a list of 3 floats, in the form of
    [x, y, yaw]. If the last known position of the robot is not available,
    returns None.
    """
    @abstractmethod
    def get_pose(self) -> Annotated[list[float], 3] | None:
        ...

    """
    This method is called to update RMF with the latest robot state.
    """
    @abstractmethod
    def update(self, state: rmf_easy.RobotState):
        ...

    """
    This method is called to send a navigation command to the robot.
    """
    @abstractmethod
    def navigate(
        self,
        destination: rmf_easy.Destination,
        execution: rmf_easy.CommandExecution
    ):
        ...

    """
    This method is called to stop the execution/continuation of the provided
    activity.
    """
    @abstractmethod
    def stop(self, activity: ActivityIdentifier):
        ...

    """
    This method is called to send a custom action command to the robot.
    """
    @abstractmethod
    def execute_action(
        self,
        category: str,
        description: dict,
        execution: ActivityIdentifier
    ):
        ...
