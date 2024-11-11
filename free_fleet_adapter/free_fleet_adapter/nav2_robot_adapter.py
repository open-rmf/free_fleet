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

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.types import (
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
    make_cancel_all_goals_request,
    namespacify,
)

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
from tf_transformations import quaternion_from_euler
import zenoh


class TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node=None):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer

        def _tf_callback(sample: zenoh.Sample):
            try:
                transform = TFMessage.deserialize(sample.payload.to_bytes())
            except Exception as e:
                error_message = \
                    f'Failed to deserialize TF payload: {type(e)}: {e}'
                if self.node is not None:
                    self.node.get_logger().debug(error_message)
                else:
                    print(error_message)
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
            error_message = \
                'Unable to get transform between base_footprint and map: ' \
                f'{type(err)}: {err}'
            if self.node is not None:
                self.node.get_logger().info(error_message)
            else:
                print(error_message)
        return None


class Nav2RobotAdapter:

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
        self.name = name
        self.execution = None
        self.update_handle = None
        self.configuration = configuration
        self.robot_config_yaml = robot_config_yaml
        self.node = node
        self.zenoh_session = zenoh_session
        self.fleet_handle = fleet_handle
        self.tf_buffer = tf_buffer

        self.nav_goal_id = None
        self.map = self.robot_config_yaml['initial_map']

        # TODO(ac): Only use full battery if sim is indicated
        self.battery_soc = 1.0

        self.replan_counts = 0

        self.tf_handler = TfHandler(
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

    def _make_random_goal_id(self):
        return np.random.randint(0, 255, size=(16)).astype('uint8').tolist()

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
                if rep.status == GoalStatus.STATUS_EXECUTING.value:
                    return False
                elif rep.status == GoalStatus.STATUS_SUCCEEDED.value:
                    self.node.get_logger().info(
                        f'Navigation goal {self.nav_goal_id} reached'
                    )
                    return True
                else:
                    # TODO(ac): test replanning behavior if goal status is
                    # neither executing or succeeded
                    self.replan_counts += 1
                    self.node.get_logger().error(
                        f'Navigation goal {self.nav_goal_id} status '
                        f'{rep.status}, replan count [{self.replan_counts}]')
                    self.update_handle.more().replan()
                    return False
            except Exception as e:
                self.node.get_logger().debug(
                    f'Received (ERROR: "{reply.err.payload.to_string()}"): '
                    f'{type(e)}: {e}')
                continue

    def update(self, state):
        activity_identifier = None
        if self.execution:
            # TODO(ac): use an enum to record what type of execution it is,
            # whether navigation or custom executions
            if self.nav_goal_id is not None and self._is_navigation_done():
                self.execution.finished()
                self.execution = None
                self.nav_goal_id = None
                self.replan_counts = 0
            else:
                activity_identifier = self.execution.identifier

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            )
        )

    def navigate(self, destination, execution):
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        if destination.map != self.map:
            # TODO(ac): test this map related replanning behavior
            self.replan_counts += 1
            self.node.get_logger().error(
                f'Destination is on map [{destination.map}], while robot '
                f'[{self.name}] is on map [{self.map}], replan count '
                f'[{self.replan_counts}]'
            )
            self.update_handle.more().replan()
            return

        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time(sec=time_now[0], nanosec=time_now[1])
        header = Header(stamp=stamp, frame_id='map')
        position = GeometryMsgs_Point(
            x=destination.position[0],
            y=destination.position[1],
            z=0
        )
        quat = quaternion_from_euler(0, 0, destination.position[2])
        orientation = GeometryMsgs_Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )
        pose = GeometryMsgs_Pose(position=position, orientation=orientation)
        pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

        nav_goal_id = self._make_random_goal_id()
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
                self.update_handle.more().replan()
                self.nav_goal_id = None
                return
            except Exception as e:
                payload = reply.err.payload.to_string()
                self.node.get_logger().error(
                    f'Received (ERROR: {payload}: {type(e)}: {e})'
                )
                continue

    def stop(self, activity):
        if self.execution is None:
            return

        if self.execution.identifier.is_same(activity):
            self.execution = None
            # TODO(ac): check what is the type of execution when we start
            # supporting something other than navigation

            if self.nav_goal_id is not None:
                req = make_cancel_all_goals_request()
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
                self.nav_goal_id = None

    def execute_action(self, category: str, description: dict, execution):
        # TODO(ac): change map using map_server load_map, and set initial
        # position again with /initialpose
        # TODO(ac): docking
        # We should never reach this point after initialization.
        error_message = \
            f'Execute action [{category}] is unsupported, this might be a ' \
            'configuration error.'
        self.node.get_logger().error(error_message)
        raise RuntimeError(error_message)
