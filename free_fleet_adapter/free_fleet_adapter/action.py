#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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
from enum import StrEnum
from typing import Annotated, Callable

import rclpy.node as Node
from rmf_adapter import RobotUpdateHandle
import rmf_adapter.easy_full_control as rmf_easy


class RobotActionState(StrEnum):
    IN_PROGRESS = 'in progress'
    CANCELING = 'canceling'
    CANCELED = 'canceled'
    COMPLETED = 'completed'
    FAILED = 'failed'


class RobotActionContext:
    def __init__(
        self,
        node: Node,
        robot_name: str,
        update_handle: RobotUpdateHandle,
        fleet_config: rmf_easy.FleetConfiguration,
        action_config: dict,
        get_battery_soc: Callable[[], float],
        get_map_name: Callable[[], str],
        get_pose: Callable[[], Annotated[list[float], 3] | None]
    ):
        self.node = node
        self.robot_name = robot_name
        self.update_handle = update_handle
        self.fleet_config = fleet_config
        self.action_config = action_config
        self.get_battery_soc = get_battery_soc
        self.get_map_name = get_map_name
        self.get_pose = get_pose


class RobotAction(ABC):

    def __init__(self, context: RobotActionContext, execution):
        self.context = context
        self.execution = execution
        self.action_task_id = \
            self.context.update_handle.more().current_task_id()

    """
    This method is called on every update by the robot adapter to monitor the
    progress and completion of the action.
    Returns the state of the action.
    """
    @abstractmethod
    def update_action(self) -> RobotActionState:
        # To be populated in the plugins
        ...

    """
    This method may be used by the action's implementation to cancel the current
    ongoing task.
    """
    def cancel_task_of_action(
        self,
        cancel_success: Callable[[], None],
        cancel_fail: Callable[[], None],
        label: str = ''
    ):
        self.context.node.get_logger().info(
            f'[{self.context.robot_name}] Cancel task requested for '
            f'[{self.action_task_id}]')

        def _on_cancel(result: bool):
            if result:
                self.context.node.get_logger().info(
                    f'[{self.context.robot_name}] Found task '
                    f'[{self.action_task_id}], cancelling...')
                cancel_success()
            else:
                self.context.node.get_logger().info(
                    f'[{self.context.robot_name}] Failed to cancel task '
                    f'[{self.action_task_id}]')
                cancel_fail()
        self.context.update_handle.more().cancel_task(
            self.action_task_id, [label], lambda result: _on_cancel(result))


class RobotActionFactory(ABC):

    def __init__(self, context: RobotActionContext):
        self.context = context

        if 'actions' not in context.action_config:
            raise KeyError(
                'List of supported actions is not provided in the action '
                'config! Unable to instantiate an ActionFactory.')
        self.actions = context.action_config['actions']

    """
    This method can be used to verify whether this ActionFactory supports
    the configured action.
    """
    @abstractmethod
    def supports_action(self, category: str) -> bool:
        # To be populated in the plugins
        ...

    """
    This method creates a Action object for the robot adapter to begin and
    interact with an action.
    """
    @abstractmethod
    def perform_action(
        self,
        category: str,
        description: dict,
        execution,
    ) -> RobotAction:
        # To be populated in the plugins
        ...
