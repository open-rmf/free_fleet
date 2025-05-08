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

import time

from free_fleet_adapter.action import (
    RobotAction,
    RobotActionContext,
    RobotActionFactory,
    RobotActionState,
)
from std_msgs.msg import Empty


class ActionFactory(RobotActionFactory):

    def __init__(self, context: RobotActionContext):
        RobotActionFactory.__init__(self, context)
        self.supported_actions = [
            'hello_world',
            'delayed_hello_world'
        ]

    def supports_action(self, category: str) -> bool:
        return category in self.supported_actions

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> RobotAction:
        # TODO(ac): Re-instate this for ROS 2 Kilted onwards, as this feature
        # is only added after Jazzy release.
        # See https://github.com/open-rmf/rmf_ros2/pull/392 for more info.
        # execution.set_automatic_cancel(False)
        match category:
            case 'hello_world':
                return HelloWorld(description, execution, self.context)
            case 'delayed_hello_world':
                return DelayedHelloWorld(description, execution, self.context)


# The hello_world custom action parses the description for a user name,
# performs logging, and completes the action immediately.
class HelloWorld(RobotAction):

    def __init__(
        self,
        description: dict,
        execution,
        context: RobotActionContext
    ):
        RobotAction.__init__(self, context, execution)

        self.context.node.get_logger().info(
            f'New HelloWorld requested for robot [{self.context.robot_name}]'
        )
        self.description = description
        self.user = None

        # Enum used for tracking whether this action has been completed
        self.state = RobotActionState.IN_PROGRESS

    def update_action(self) -> RobotActionState:
        if self.state == RobotActionState.COMPLETED:
            return self.state

        # Perform the action if it has not been handled yet, update the state
        self.context.node.get_logger().info('Hello, world!')
        current_pose = self.context.get_pose()
        if current_pose is None:
            self.context.node.get_logger().error('Unable to get pose!')
        else:
            self.context.node.get_logger().info(f'Current pose: {current_pose}')

        # Use information in perform_action description for custom logic
        user = self.description.get('user')
        if user is not None:
            self.user = str(user)
            self.context.node.get_logger().info(f'Hello, {self.user} too!')

        self.state = RobotActionState.COMPLETED
        return self.state


# The delayed_hello_world custom action parses the description for a user
# name and duration for waiting, performs logging only after the specified
# duration has elapsed, and allows users to cancel the action via a custom
# Empty topic with topic name `cancel_delayed_hello_world`.
# Cancel the action with the following command
# ros2 topic pub --once  /cancel_delayed_hello_world std_msgs/msg/Empty "{}"
class DelayedHelloWorld(RobotAction):

    def __init__(
        self,
        description: dict,
        execution,
        context: RobotActionContext
    ):
        RobotAction.__init__(self, context, execution)

        self.context.node.get_logger().info(
            'New DelayedHelloWorld requested for robot '
            f'[{self.context.robot_name}]'
        )
        self.description = description
        self.user = None
        self.wait_duration_sec = 5
        self.start_millis = None

        # Enum used for tracking whether this action has been completed
        self.state = RobotActionState.IN_PROGRESS

        # Custom cancellation interaction
        self.cancel_topic = 'cancel_delayed_hello_world'
        self.cancel_sub = None

    def _cancel_action(self, msg: Empty):
        self.state = RobotActionState.CANCELING
        self.context.node.get_logger().info(
            'Received custom cancel command delayed_hello_world action for '
            f'robot [{self.context.robot_name}]'
        )

        def _cancel_success():
            self.state = RobotActionState.CANCELED

        def _cancel_fail():
            self.state = RobotActionState.FAILED

        # Canceling the entire task due to canceling this action
        self.cancel_task_of_action(
            _cancel_success,
            _cancel_fail,
            'Custom cancel behavior of delayed_hello_world action'
        )

    def update_action(self) -> RobotActionState:
        if self.state == RobotActionState.COMPLETED or \
                self.state == RobotActionState.CANCELING or \
                self.state == RobotActionState.CANCELED:
            return self.state

        # Parse the description, set up the custom cancellation topic and start
        # time for this action
        if self.start_millis is None:
            wait_duration_sec = self.description.get('wait_duration_sec')
            if wait_duration_sec is not None:
                self.wait_duration_sec = int(wait_duration_sec)
            user = self.description.get('user')
            if user is not None:
                self.user = str(user)

            # Start the subscription to listen for cancellation of this action
            self.cancel_sub = self.context.node.create_subscription(
                Empty,
                self.cancel_topic,
                self._cancel_action,
                10
            )

            self.start_millis = round(time.time() * 1000)

        # Complete the action if wait_duration_sec has elapsed
        current_millis = round(time.time() * 1000)
        if current_millis - self.start_millis > self.wait_duration_sec * 1000:
            # Perform the action if it has not been handled yet, update the state
            self.context.node.get_logger().info('Hello, world!')
            current_pose = self.context.get_pose()
            if current_pose is None:
                self.context.node.get_logger().error('Unable to get pose!')
            else:
                self.context.node.get_logger().info(
                    f'Current pose: {current_pose}')

            if self.user is not None:
                self.context.node.get_logger().info(f'Hello, {self.user} too!')

            self.state = RobotActionState.COMPLETED
            return self.state

        # Update that the action is still in progress
        self.state = RobotActionState.IN_PROGRESS
        return self.state
