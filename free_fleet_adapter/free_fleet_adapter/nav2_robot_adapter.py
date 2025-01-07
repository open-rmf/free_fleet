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

from typing import Annotated

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros2_types import (
    ActionMsgs_CancelGoal_Response,
    GeometryMsgs_Point,
    GeometryMsgs_Pose,
    GeometryMsgs_PoseStamped,
    GeometryMsgs_Quaternion,
    GoalStatus,
    Header,
    NavigateToPose_GetResult_Request,
    NavigateToPose_GetResult_Response,
    NavigateToPose_SendGoal_Request,
    NavigateToPose_SendGoal_Response,
    SensorMsgs_BatteryState,
    TFMessage,
    Time,
)
from free_fleet.utils import (
    make_nav2_cancel_all_goals_request,
    namespacify,
)
from free_fleet_adapter.robot_adapter import RobotAdapter

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import ActivityIdentifier, Tier
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import zenoh


class Nav2TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer

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
            # TODO(ac): parameterize the frames for lookup
            transform = self.tf_buffer.lookup_transform(
                namespacify('map', self.robot_name),
                namespacify('base_footprint', self.robot_name),
                rclpy.time.Time()
            )
            return transform
        except Exception as err:
            self.node.get_logger().info(
                'Unable to get transform between base_footprint and map: '
                f'{type(err)}: {err}'
            )
        return None


class Nav2RobotAdapter(RobotAdapter):

    def __init__(
        self,
        name: str,
        configuration,
        robot_config_yaml,
        node,
        zenoh_session,
        fleet_handle,
        tf_buffer
    ):
        RobotAdapter.__init__(self, name, node, fleet_handle)

        self.execution = None
        self.configuration = configuration
        self.robot_config_yaml = robot_config_yaml
        self.zenoh_session = zenoh_session
        self.tf_buffer = tf_buffer

        self.nav_goal_id = None
        self.map_name = self.robot_config_yaml['initial_map']

        # TODO(ac): Only use full battery if sim is indicated
        self.battery_soc = 1.0

        self.replan_counts = 0
        self.nav_issue_ticket = None

        self.tf_handler = Nav2TfHandler(
            self.name,
            self.zenoh_session,
            self.tf_buffer,
            self.node
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

    def get_battery_soc(self) -> float:
        return self.battery_soc

    def get_map_name(self) -> str:
        return self.map_name

    def get_pose(self) -> Annotated[list[float], 3] | None:
        transform = self.tf_handler.get_transform()
        if transform is None:
            error_message = \
                f'Failed to update robot [{self.name}]: Unable to get ' \
                f'transform between base_footprint and map'
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

    def _is_navigation_done(self) -> bool:
        if self.nav_goal_id is None:
            return True

        req = NavigateToPose_GetResult_Request(goal_id=self.nav_goal_id)
        # TODO(ac): parameterize the service call timeout
        replies = self.zenoh_session.get(
            namespacify('navigate_to_pose/_action/get_result', self.name),
            payload=req.serialize(),
            # timeout=0.5
        )
        for reply in replies:
            try:
                # Deserialize the response
                rep = NavigateToPose_GetResult_Response.deserialize(
                    reply.ok.payload.to_bytes()
                )
                self.node.get_logger().debug(f'Result: {rep.status}')
                match rep.status:
                    case GoalStatus.STATUS_EXECUTING.value | \
                         GoalStatus.STATUS_ACCEPTED.value | \
                         GoalStatus.STATUS_CANCELING.value:
                        return False
                    case GoalStatus.STATUS_SUCCEEDED.value:
                        self.node.get_logger().info(
                            f'Navigation goal {self.nav_goal_id} reached'
                        )
                        if self.nav_issue_ticket is not None:
                            msg = {}
                            self.nav_issue_ticket.resolve(msg)
                            self.nav_issue_ticket = None
                            self.node.get_logger().info(
                                'Navigation issue ticket has been resolved'
                            )
                        return True
                    case GoalStatus.STATUS_CANCELED.value:
                        self.node.get_logger().info(
                            f'Navigation goal {self.nav_goal_id} was cancelled'
                        )
                        return True
                    case _:
                        self.nav_issue_ticket = self.create_nav_issue_ticket(
                            'navigation',
                            f'Navigate to pose result status [{rep.status}]',
                            self.nav_goal_id
                        )

                        # TODO(ac): test replanning behavior if goal status is
                        # neither executing or succeeded
                        self.replan_counts += 1
                        self.node.get_logger().error(
                            f'Navigation goal {self.nav_goal_id} status '
                            f'{rep.status}, replan count '
                            f'[{self.replan_counts}]'
                        )
                        self.update_handle.more().replan()
                        return False
            except Exception as e:
                self.node.get_logger().debug(
                    f'Received (ERROR: "{reply.err.payload.to_string()}"): '
                    f'{type(e)}: {e}')
                continue

    # TODO(ac): issue ticket can be more generic for execute actions too
    def create_nav_issue_ticket(self, category, msg, nav_goal_id=None):
        if self.update_handle is None:
            error_message = \
                'Failed to create navigation issue ticket for robot ' \
                f'{self.name}, robot adapter has not yet been initialized ' \
                'with a fleet update handle.'
            self.node.get_logger().error(error_message)
            return None

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
        if self.execution:
            # TODO(ac): use an enum to record what type of execution it is,
            # whether navigation or custom executions
            if self.nav_goal_id is not None and self._is_navigation_done():
                # TODO(ac): Refactor this check as as self._is_navigation_done
                # takes a while and the execution may have become None due to
                # task cancellation.
                if self.execution is not None:
                    self.execution.finished()
                    self.execution = None
                self.nav_goal_id = None
                self.replan_counts = 0
            else:
                activity_identifier = self.execution.identifier

        self.update_handle.update(state, activity_identifier)

    def _handle_navigate_to_pose(
        self,
        map_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float
    ):
        if map_name != self.map_name:
            # TODO(ac): test this map related replanning behavior
            self.replan_counts += 1
            self.node.get_logger().error(
                f'Destination is on map [{map_name}], while robot '
                f'[{self.name}] is on map [{self.map_name}], replan count '
                f'[{self.replan_counts}]'
            )

            if self.update_handle is None:
                error_message = \
                    f'Failed to replan for robot {self.name}, robot adapter ' \
                    'has not yet been initialized with a fleet update handle.'
                self.node.get_logger().error(error_message)
                return
            self.update_handle.more().replan()
            return

        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time(sec=time_now[0], nanosec=time_now[1])
        header = Header(stamp=stamp, frame_id='map')
        position = GeometryMsgs_Point(x=x, y=y, z=z)
        quat = quaternion_from_euler(0, 0, yaw)
        orientation = GeometryMsgs_Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )
        pose = GeometryMsgs_Pose(position=position, orientation=orientation)
        pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

        nav_goal_id = \
            np.random.randint(0, 255, size=(16)).astype('uint8').tolist()
        req = NavigateToPose_SendGoal_Request(
            goal_id=nav_goal_id,
            pose=pose_stamped,
            behavior_tree=''
        )

        replies = self.zenoh_session.get(
            namespacify('navigate_to_pose/_action/send_goal', self.name),
            payload=req.serialize(),
            # timeout=0.5
        )

        for reply in replies:
            try:
                rep = NavigateToPose_SendGoal_Response.deserialize(
                    reply.ok.payload.to_bytes())
                if rep.accepted:
                    self.node.get_logger().info(
                        f'Navigation goal {nav_goal_id} accepted'
                    )
                    self.nav_goal_id = nav_goal_id
                    return

                self.replan_counts += 1
                self.node.get_logger().error(
                    f'Navigation goal {nav_goal_id} was rejected, replan '
                    f'count [{self.replan_counts}]'
                )
                if self.update_handle is None:
                    error_message = \
                        f'Failed to replan for robot {self.name}, robot ' \
                        'adapter has not yet been initialized with a fleet ' \
                        'update handle.'
                    self.node.get_logger().error(error_message)
                    return
                self.update_handle.more().replan()
                self.nav_goal_id = None
                return
            except Exception as e:
                payload = reply.err.payload.to_string()
                self.node.get_logger().error(
                    f'Received (ERROR: {payload}: {type(e)}: {e})'
                )
                continue

    def navigate(
        self,
        destination: rmf_easy.Destination,
        execution: rmf_easy.CommandExecution
    ):
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )
        self._handle_navigate_to_pose(
            destination.map,
            destination.position[0],
            destination.position[1],
            0.0,
            destination.position[2]
        )

    def _handle_stop_navigation(self):
        req = make_nav2_cancel_all_goals_request()
        replies = self.zenoh_session.get(
            namespacify(
                'navigate_to_pose/_action/cancel_goal',
                self.name,
            ),
            payload=req.serialize(),
            # timeout=0.5
        )
        for reply in replies:
            rep = ActionMsgs_CancelGoal_Response.deserialize(
                reply.ok.payload.to_bytes()
            )
            self.node.get_logger().info(
                'Return code: %d' % rep.return_code
            )

    def stop(self, activity: ActivityIdentifier):
        if self.execution is None:
            return

        if self.execution.identifier.is_same(activity):
            self.execution = None
            # TODO(ac): check what is the type of execution when we start
            # supporting something other than navigation

            if self.nav_goal_id is not None:
                self._handle_stop_navigation()
                # TODO(ac): check return code before setting nav_goal_id to
                # None
                self.nav_goal_id = None

    def execute_action(
        self,
        category: str,
        description: dict,
        execution: ActivityIdentifier
    ):
        # TODO(ac): change map using map_server load_map, and set initial
        # position again with /initialpose
        # TODO(ac): docking
        # We should never reach this point after initialization.
        error_message = \
            f'Execute action [{category}] is unsupported, this might be a ' \
            'configuration error.'
        self.node.get_logger().error(error_message)
        raise RuntimeError(error_message)
