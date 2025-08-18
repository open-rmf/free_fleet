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

import numpy as np

from abc import ABC, abstractmethod
from free_fleet.ros2_types import (
    ActionMsgs_CancelGoal_Response,
    DockRobot_GetResult_Request,
    DockRobot_GetResult_Response,
    DockRobot_SendGoal_Request,
    DockRobot_SendGoal_Response,
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
    Time,
)
from free_fleet.utils import (
    make_nav2_cancel_all_goals_request,
    namespacify,
)
from free_fleet_adapter.robot_adapter import ExecutionHandle
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class RequestAborted(Exception):
    pass


class RequestRejected(Exception):
    pass


class ExecutionItem(ABC):
    """Abstract Nav 2 execution item."""

    @abstractmethod
    def execute(self):
        ...

    @abstractmethod
    def update(self, state):
        ...

    @abstractmethod
    def is_done(self):
        ...

    @abstractmethod
    def stop(self):
        ...


class NavigationExecutionItem(ExecutionItem):
    """Base class of navigation request."""

    def __init__(self, robot_name, node, update_handle, zenoh_session, exec_handle):
        self.robot_name = robot_name
        self.node = node
        self.update_handle = update_handle
        self.zenoh_session = zenoh_session
        self.exec_handle: ExecutionHandle = exec_handle

    @abstractmethod
    def _get_ros2_action_string(self):
        ...

    @abstractmethod
    def _create_ros2_action_send_goal_request(self):
        ...

    @abstractmethod
    def _parse_ros2_action_send_goal_response(self, payload):
        ...

    @abstractmethod
    def _create_ros2_action_get_result_request(self, goal_id):
        ...

    @abstractmethod
    def _parse_ros2_action_get_result_response(self, payload):
        ...

    def _generate_goal_id(self):
        return np.random.randint(0, 255, size=(16)).astype('uint8').tolist()

    def get_goal_id(self):
        if self.exec_handle:
            return self.exec_handle.goal_id

        return None

    def execute(self):
        action_string = self._get_ros2_action_string()
        req = self._create_ros2_action_send_goal_request()

        nav_goal_id = req.goal_id

        replies = self.zenoh_session.get(
            namespacify(f'{action_string}/_action/send_goal', self.robot_name),
            payload=req.serialize(),
            # timeout=0.5
        )

        for reply in replies:
            try:
                # TODO: reply.ok may not always be set and should be handled appropriately
                rep = self._parse_ros2_action_send_goal_response(reply.ok.payload.to_bytes())
            except Exception as e:
                payload = reply.err.payload.to_string()
                self.node.get_logger().error(
                    f'Received (ERROR: {payload}: {type(e)}: {e})'
                )
                continue
            else:
                if rep.accepted:
                    self.node.get_logger().info(
                        f'{action_string} goal {nav_goal_id} accepted'
                    )
                    self.exec_handle.set_goal_id(nav_goal_id)
                    return

                self.node.get_logger().error(
                    f'{action_string} goal {nav_goal_id} was rejected'
                )
                raise RequestRejected

    def update(self, state):
        if self.exec_handle and self.exec_handle.execution and self.exec_handle.goal_id:
            is_done, succeeded = self.is_done()
            if is_done:
                self.exec_handle.execution.finished()
                self.exec_handle.execution = None
            return (is_done, succeeded)

        return (False, None)

    def is_done(self):
        if self.exec_handle is None or self.exec_handle.goal_id is None:
            return (True, None)

        action_string = self._get_ros2_action_string()
        req = self._create_ros2_action_get_result_request(goal_id=self.exec_handle.goal_id)
        # TODO(ac): parameterize the service call timeout
        replies = self.zenoh_session.get(
            namespacify(f'{action_string}/_action/get_result', self.robot_name),
            payload=req.serialize(),
            # timeout=0.5
        )
        for reply in replies:
            try:
                # Deserialize the response
                # TODO: reply.ok may not always be set and should be handled appropriately
                rep = self._parse_ros2_action_get_result_response(reply.ok.payload.to_bytes())
            except Exception as e:
                self.node.get_logger().debug(
                    f'Received (ERROR: "{reply.err.payload.to_string()}"): '
                    f'{type(e)}: {e}')
                continue
            else:
                self.node.get_logger().debug(f'Result: {rep.status}')
                match rep.status:
                    case GoalStatus.STATUS_EXECUTING.value | \
                         GoalStatus.STATUS_ACCEPTED.value | \
                         GoalStatus.STATUS_CANCELING.value:
                        return (False, None)
                    case GoalStatus.STATUS_SUCCEEDED.value:
                        self.node.get_logger().info(
                            f'{action_string} goal {self.exec_handle.goal_id} reached'
                        )
                        return (True, True)
                    case GoalStatus.STATUS_CANCELED.value:
                        self.node.get_logger().info(
                            f'{action_string} goal {self.exec_handle.goal_id} was cancelled'
                        )
                        return (True, False)
                    case _:
                        self.node.get_logger().error(
                            f'{action_string} goal {self.exec_handle.goal_id} status '
                            f'{rep.status}'
                        )
                        raise RequestAborted(f'{action_string} result status [{rep.status}]')
        return (False, None)

    def stop(self):
        if self.exec_handle is not None:
            with self.exec_handle.mutex:
                if self.exec_handle.execution is not None and self.exec_handle.goal_id is not None:
                    action_string = self._get_ros2_action_string()
                    req = make_nav2_cancel_all_goals_request()
                    replies = self.zenoh_session.get(
                        namespacify(
                            f'{action_string}/_action/cancel_goal',
                            self.robot_name,
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


class NavigateToPoseExecutionItem(NavigationExecutionItem):
    def __init__(
        self,
        robot_name,
        node,
        update_handle,
        zenoh_session,
        exec_handle: ExecutionHandle,
        map_frame: str,
        x: float,
        y: float,
        z: float,
        yaw: float
    ):
        super().__init__(robot_name, node, update_handle, zenoh_session, exec_handle)
        self.map_frame = map_frame
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def _get_ros2_action_string(self):
        return 'navigate_to_pose'

    def _create_ros2_action_send_goal_request(self):
        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time(sec=time_now[0], nanosec=time_now[1])
        header = Header(stamp=stamp, frame_id=self.map_frame)
        position = GeometryMsgs_Point(x=self.x, y=self.y, z=self.z)
        quat = quaternion_from_euler(0, 0, self.yaw)
        orientation = GeometryMsgs_Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )
        pose = GeometryMsgs_Pose(position=position, orientation=orientation)
        pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

        return NavigateToPose_SendGoal_Request(
            goal_id=self._generate_goal_id(),
            pose=pose_stamped,
            behavior_tree=''
        )

    def _parse_ros2_action_send_goal_response(self, payload):
        return NavigateToPose_SendGoal_Response.deserialize(payload)

    def _create_ros2_action_get_result_request(self, goal_id):
        return NavigateToPose_GetResult_Request(goal_id=goal_id)

    def _parse_ros2_action_get_result_response(self, payload):
        return NavigateToPose_GetResult_Response.deserialize(payload)


class DockExecutionItem(NavigationExecutionItem):
    DEFAULT_DOCK_ACTION_USE_DOCK_ID = True
    # since dock ID is used, a dock pose is not required but it needs to be provided to satisfy the API
    DEFAULT_DOCK_ACTION_POSE = PoseStamped()
    DEFAULT_DOCK_ACTION_DOCK_TYPE = ''

    # the following are the default values in the DockRobot ROS 2 action
    DEFAULT_DOCK_ACTION_MAX_STAGING_TIME = 1000.0
    DEFAULT_DOCK_ACTION_NAVIGATE_TO_STAGING_POSE = True

    def __init__(
        self,
        robot_name,
        node,
        update_handle,
        zenoh_session,
        exec_handle: ExecutionHandle,
        dock_id: str
    ):
        super().__init__(robot_name, node, update_handle, zenoh_session, exec_handle)
        self.dock_id = dock_id

    def _get_ros2_action_string(self):
        return 'dock_robot'

    def _create_ros2_action_send_goal_request(self):
        return DockRobot_SendGoal_Request(
            goal_id=self._generate_goal_id(),
            dock_id=self.dock_id,
            use_dock_id=DockExecutionItem.DEFAULT_DOCK_ACTION_USE_DOCK_ID,
            dock_pose=DockExecutionItem.DEFAULT_DOCK_ACTION_POSE,
            dock_type=DockExecutionItem.DEFAULT_DOCK_ACTION_DOCK_TYPE,
            max_staging_time=DockExecutionItem.DEFAULT_DOCK_ACTION_MAX_STAGING_TIME,
            navigate_to_staging_pose=DockExecutionItem.DEFAULT_DOCK_ACTION_NAVIGATE_TO_STAGING_POSE,
        )

    def _parse_ros2_action_send_goal_response(self, payload):
        return DockRobot_SendGoal_Response.deserialize(payload)

    def _create_ros2_action_get_result_request(self, goal_id):
        return DockRobot_GetResult_Request(goal_id=goal_id)

    def _parse_ros2_action_get_result_response(self, payload):
        return DockRobot_GetResult_Response.deserialize(payload)
