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

import importlib
from typing import Annotated

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros2_types import (
    SensorMsgs_BatteryState,
    TFMessage,
)
from free_fleet.utils import (
    namespacify,
)
from free_fleet_adapter.action import (
    RobotActionContext,
    RobotActionState,
)
from free_fleet_adapter.nav2_execution_item import (
    DockExecutionItem,
    NavigationExecutionItem,
    NavigateToPoseExecutionItem,
    RequestAborted,
    RequestRejected,
)
from free_fleet_adapter.robot_adapter import ExecutionHandle, RobotAdapter

from geometry_msgs.msg import TransformStamped
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import ActivityIdentifier, Tier
from tf_transformations import euler_from_quaternion
import zenoh


class Nav2TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node,
                 robot_frame='base_footprint', map_frame='map'):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.map_frame = map_frame

        def _tf_callback(sample: zenoh.Sample):
            try:
                transform = TFMessage.deserialize(sample.payload.to_bytes())
            except Exception as e:
                self.node.get_logger().debug(
                    f'Failed to deserialize TF payload: {type(e)}: {e}'
                )
                return None
            for zt in transform.transforms:
                t = transform_stamped_to_ros2_msg(zt)
                t.header.frame_id = namespacify(zt.header.frame_id,
                                                self.robot_name)
                t.child_frame_id = namespacify(zt.child_frame_id,
                                               self.robot_name)
                self.tf_buffer.set_transform(
                    t, f'{self.robot_name}_TfListener')

        self.tf_sub = self.zenoh_session.declare_subscriber(
            namespacify('tf', self.robot_name),
            _tf_callback
        )

    def get_transform(self) -> TransformStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                namespacify(self.map_frame, self.robot_name),
                namespacify(self.robot_frame, self.robot_name),
                rclpy.time.Time()
                )
            return transform
        except Exception as err:
            self.node.get_logger().info(
                f'Unable to get transform between {self.robot_frame} '
                f'and {self.map_frame}: {type(err)}: {err}'
            )
        return None


class Nav2RobotAdapter(RobotAdapter):
    MAX_REPLAN_COUNTS = 5

    def __init__(
        self,
        name: str,
        configuration,
        robot_config_yaml,
        plugin_config: dict | None,
        node,
        zenoh_session,
        fleet_handle,
        fleet_config: rmf_easy.FleetConfiguration | None,
        tf_buffer
    ):
        RobotAdapter.__init__(self, name, node, fleet_handle)

        self.configuration = configuration
        self.robot_config_yaml = robot_config_yaml
        self.zenoh_session = zenoh_session
        self.fleet_config = fleet_config
        self.tf_buffer = tf_buffer

        # tracks active action execution requests
        self.exec_handle: ExecutionHandle | None = None

        # tracks active navigation execution requests
        self.nav_handle: NavigationExecutionItem | None = None

        self.map_name = self.robot_config_yaml['initial_map']
        default_map_frame = 'map'
        default_robot_frame = 'base_footprint'
        self.map_frame = self.robot_config_yaml.get('map_frame', default_map_frame)
        self.robot_frame = self.robot_config_yaml.get('robot_frame', default_robot_frame)

        # TODO(ac): Only use full battery if sim is indicated
        self.battery_soc = 1.0

        self.replan_counts = 0
        self.nav_issue_ticket = None

        # Maps action name to plugin name
        self.action_to_plugin_name = {}
        # Maps plugin name to action factory
        self.action_factories = {}

        self.tf_handler = Nav2TfHandler(
            self.name,
            self.zenoh_session,
            self.tf_buffer,
            self.node,
            robot_frame=self.robot_frame,
            map_frame=self.map_frame
        )

        def _battery_state_callback(sample: zenoh.Sample):
            battery_state = SensorMsgs_BatteryState.deserialize(
                sample.payload.to_bytes()
            )
            self.battery_soc = battery_state.percentage

        self.battery_state_sub = self.zenoh_session.declare_subscriber(
            namespacify('battery_state', name),
            _battery_state_callback
        )

        # Initialize robot
        init_timeout_sec = self.robot_config_yaml.get('init_timeout_sec', 10)
        self.node.get_logger().info(f'Initializing robot [{self.name}]...')
        init_robot_pose = rclpy.Future()

        def _get_init_pose():
            robot_pose = self.get_pose()
            if robot_pose is not None:
                init_robot_pose.set_result(robot_pose)
                init_robot_pose.done()

        init_pose_timer = self.node.create_timer(1, _get_init_pose)
        rclpy.spin_until_future_complete(
            self.node, init_robot_pose, timeout_sec=init_timeout_sec
        )

        if init_robot_pose.result() is None:
            error_message = \
                f'Timeout trying to initialize robot [{self.name}]'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)

        self.node.destroy_timer(init_pose_timer)
        state = rmf_easy.RobotState(
            self.get_map_name(),
            init_robot_pose.result(),
            self.get_battery_soc()
        )

        if self.fleet_handle is None:
            self.node.get_logger().warn(
                f'Fleet unavailable, skipping adding robot [{self.name}] '
                'to fleet.'
            )
            return

        self.update_handle = self.fleet_handle.add_robot(
            self.name,
            state,
            self.configuration,
            rmf_easy.RobotCallbacks(
                lambda destination, execution: self.navigate(
                    destination, execution
                ),
                lambda activity: self.stop(activity),
                lambda category, description, execution: self.execute_action(
                    category, description, execution
                )
            )
        )
        if not self.update_handle:
            error_message = \
                f'Failed to add robot [{self.name}] to fleet ' \
                f'[{self.fleet_handle.more().fleet_name()}], this is most ' \
                'likely due to a configuration error.'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)

        if self.fleet_config is None:
            self.node.get_logger().info(
                'No fleet configuration provided for RobotAdapter of '
                f'[{self.name}]. No plugin actions will be loaded.'
            )
            return

        # Import and store plugin actions and action factories
        if plugin_config is not None:
            for plugin_name, action_config in plugin_config.items():
                try:
                    module = action_config['module']
                    plugin = importlib.import_module(module)
                    action_context = RobotActionContext(
                        self.node,
                        self.name,
                        self.update_handle,
                        self.fleet_config,
                        action_config,
                        self.get_battery_soc,
                        self.get_map_name,
                        self.get_pose
                    )
                    action_factory = plugin.ActionFactory(action_context)
                    for action in action_factory.actions:
                        # Verify that this action is not duplicated across plugins
                        target_plugin = self.action_to_plugin_name.get(action)
                        if (target_plugin is not None and
                                target_plugin != plugin_name):
                            raise Exception(
                                f'Action [{action}] is indicated to be supported '
                                f'by multiple plugins: {target_plugin} and '
                                f'{plugin_name}. The fleet adapter is unable to '
                                f'select the intended plugin to be paired for '
                                f'this action. Please ensure that action names '
                                f'are not duplicated across supported plugins. '
                                f'Unable to create ActionFactory for '
                                f'{plugin_name}. Robot [{self.name}] will not be '
                                f'able to perform actions associated with this '
                                f'plugin.'
                            )
                        # Verify that this ActionFactory supports this action
                        if not action_factory.supports_action(action):
                            raise ValueError(
                                f'The plugin config provided [{action}] as a '
                                f'performable action, but it is not a supported '
                                f'action in the {plugin_name} ActionFactory!'
                            )
                        self.action_to_plugin_name[action] = plugin_name
                    self.action_factories[plugin_name] = action_factory
                except KeyError:
                    self.node.get_logger().info(
                        f'Unable to create ActionFactory for {plugin_name}! '
                        f'Configured plugin config is invalid. '
                        f'Robot [{self.name}] will not be able to perform '
                        f'actions associated with this plugin.'
                    )
                except ImportError:
                    self.node.get_logger().info(
                        f'Unable to import module for {plugin_name}! '
                        f'Robot [{self.name}] will not be able to perform '
                        f'actions associated with this plugin.'
                    )

    def get_battery_soc(self) -> float:
        return self.battery_soc

    def get_map_name(self) -> str:
        return self.map_name

    def get_pose(self) -> Annotated[list[float], 3] | None:
        transform = self.tf_handler.get_transform()
        if transform is None:
            error_message = \
                f'Failed to update robot [{self.name}]: Unable to get ' \
                f'transform between {self.robot_frame} and {self.map_frame}'
            self.node.get_logger().info(error_message)
            return None

        orientation = euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        robot_pose = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            orientation[2]
        ]
        return robot_pose

    # TODO(ac): issue ticket can be more generic for execute actions too
    def create_nav_issue_ticket(self, category, msg, nav_goal_id=None):
        tier = Tier.Error
        detail = {
            'nav_goal_id': f'{nav_goal_id}',
            'message': msg
        }
        nav_issue_ticket = \
            self.update_handle.more().create_issue(tier, category, detail)
        self.node.get_logger().info(
            f'Created [{category}] issue ticket for robot [{self.name}] with '
            f'nav goal ID [{nav_goal_id}]')
        return nav_issue_ticket

    def update(self, state: rmf_easy.RobotState):
        if self.update_handle is None:
            error_message = \
                f'Failed to update robot {self.name}, robot adapter has not ' \
                'yet been initialized with a fleet update handle.'
            self.node.get_logger().error(error_message)
            return

        activity_identifier = None
        exec_handle = self.exec_handle
        nav_handle = self.nav_handle

        if nav_handle:
            # Handle navigation requests
            try:
                is_done, succeeded = nav_handle.update(state)
                if is_done:
                    self._replan_counts = 0
                    if succeeded:
                        if self.nav_issue_ticket is not None:
                            msg = {}
                            self.nav_issue_ticket.resolve(msg)
                            self.nav_issue_ticket = None
                            self.node.get_logger().info(
                                'Navigation issue ticket has been resolved'
                            )
            except RequestAborted as e:
                self.nav_issue_ticket = self.create_nav_issue_ticket(
                    'navigation',
                    str(e),
                    nav_handle.get_goal_id()
                )
                self._trigger_replan()

        elif exec_handle:
            # Handle custom actions
            if exec_handle.execution and exec_handle.action:
                current_action_state = exec_handle.action.update_action()
                match current_action_state:
                    case RobotActionState.CANCELED | \
                            RobotActionState.COMPLETED | \
                            RobotActionState.FAILED:
                        self.node.get_logger().info(
                            f'Robot [{self.name}] current action '
                            f'[{current_action_state}]'
                        )
                        exec_handle.execution.finished()
                        exec_handle.action = None
            # Commands are still being carried out
            activity_identifier = exec_handle.activity

        self.update_handle.update(state, activity_identifier)

    def navigate(
        self,
        destination: rmf_easy.Destination,
        execution: rmf_easy.CommandExecution
    ):
        self._request_nav_stop()
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        if destination.map != self.map_name:
            # TODO(ac): test this map related replanning behavior
            self.replan_counts += 1
            self.node.get_logger().error(
                f'Destination is on map [{destination.map_name}], while robot '
                f'[{self.name}] is on map [{self.map_name}]'
            )
            self._trigger_replan()
            return

        if destination.dock is not None:
            self.nav_handle = DockExecutionItem(
                self.name, self.node, self.update_handle, self.zenoh_session, ExecutionHandle(execution),
                destination.dock
            )
        else:
            self.nav_handle = NavigateToPoseExecutionItem(
                self.name, self.node, self.update_handle, self.zenoh_session, ExecutionHandle(execution),
                self.map_frame,
                destination.position[0],
                destination.position[1],
                0.0,
                destination.position[2]
            )

        try:
            self.nav_handle.execute()
        except RequestRejected:
            self._trigger_replan()

    def _request_nav_stop(self):
        nav_handle = self.nav_handle
        if nav_handle:
            nav_handle.stop()
            self.nav_handle = None

    def stop(self, activity: ActivityIdentifier):
        if self.exec_handle is not None:
            # TODO: stop the ongoing custom activity
            self.exec_handle = None

        elif self.nav_handle is not None:
            self._request_nav_stop()

    def execute_action(
        self,
        category: str,
        description: dict,
        execution: ActivityIdentifier
    ):
        current_exec_handle = self.exec_handle
        if current_exec_handle is not None and \
                current_exec_handle.action is not None:
            # This should never be reached
            self.node.get_logger().error(
                f'Robot [{self.name}] received a new action while it is busy '
                'with another action. Ending current action and accepting '
                f'incoming action [{category}]'
            )
            if current_exec_handle.execution is not None:
                current_exec_handle.execution.finished()
            current_exec_handle.action = None

        action_factory = None
        plugin_name = self.action_to_plugin_name.get(category)
        if plugin_name:
            action_factory = self.action_factories.get(plugin_name)
        else:
            for plugin, factory in self.action_factories.items():
                if factory.supports_action(category):
                    factory.actions.append(category)
                    action_factory = factory
                    break

        if action_factory:
            # Valid action-plugin pair exists, create RobotAction
            robot_action = action_factory.perform_action(
                category, description, execution
            )
            self.exec_handle = ExecutionHandle(execution)
            self.exec_handle.set_action(robot_action)
            return

        # TODO(ac): change map using map_server load_map, and set initial
        # position again with /initialpose
        # TODO(ac): docking
        # We should never reach this point after initialization.
        error_message = \
            f'RobotAction [{category}] was not configured for this fleet.'
        self.node.get_logger().error(error_message)
        raise NotImplementedError(error_message)

    def _trigger_replan(self):
        if self._replan_counts < self.MAX_REPLAN_COUNTS:
            self._replan_counts += 1
            self.node.get_logger().info(
                f'Replanning navigation for robot {self.name}, attempt {self._replan_counts}'
            )

            if self.update_handle is None:
                error_message = \
                    f'Failed to replan for robot {self.name}, robot ' \
                    'adapter has not yet been initialized with a fleet ' \
                    'update handle.'
                self.node.get_logger().error(error_message)
                return

            self.update_handle.more().replan()
        else:
            self.node.get_logger().error(
                f'Maximum replanning attempts reached for robot {self.name}'
            )
            self._replan_counts = 0
