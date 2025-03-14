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

from free_fleet_adapter.action import (
    RobotAction,
    RobotActionContext,
    RobotActionFactory,
    RobotActionState,
)


class ActionFactory(RobotActionFactory):
    def __init__(self, context: RobotActionContext):
        RobotActionFactory.__init__(self, context)

    def supports_action(self, category: str) -> bool:
        if category == 'hello_world':
            return True

        return False

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> RobotAction:
        execution.set_automatic_cancel(True)

        assert category == 'hello_world'
        return HelloWorld(description, execution, self.context)


class HelloWorld(RobotAction):
    def __init__(
        self,
        description: dict,
        execution,
        context: RobotActionContext
    ):
        RobotAction.__init__(self, context, execution)

        self.context.node.get_logger().info(
            f'New HelloWorld requested for robot [{self.context.name}]'
        )
        self.description = description

        # Enum used for tracking whether this action has been completed
        self.state = RobotActionState.IN_PROGRESS

    def update_action(self) -> RobotActionState:
        # Perform the action if it has not been handled yet, update the state
        if self.state is not RobotActionState.COMPLETED:
            self.context.node.get_logger().info('Hello, world!')

            # Use information in perform_action description for custom logic
            user = self.description.get('user')
            if user is not None:
                self.context.node.get_logger().info(f'Hello, {user}!')

            self.state = RobotActionState.COMPLETED
        return self.state
