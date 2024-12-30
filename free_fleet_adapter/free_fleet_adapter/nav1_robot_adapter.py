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

import time
from typing import Annotated

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros1_types import (
    BatteryState,
    GoalID,
    GoalStatus,
    GoalStatusArray,
    Header,
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    ROS1_STORE,
    TFMessage,
    Time,
)
from free_fleet.utils import (
    get_zenoh_name_of_ros1_topic,
    namespacify,
)
from free_fleet_adapter.robot_adapter import RobotAdapter

from geometry_msgs.msg import TransformStamped
# import numpy as np
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import ActivityIdentifier
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import zenoh


class Nav1TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer

        def _tf_callback(sample: zenoh.Sample):
            try:
                transform = ROS1_STORE.deserialize_ros1(
                    sample.payload.to_bytes(),
                    TFMessage.type_name
                )
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

        zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f"/{namespacify('tf', self.robot_name)}",
            msg_type=TFMessage.type_name
        )
        self.tf_sub = self.zenoh_session.declare_subscriber(
            zenoh_topic,
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


class Nav1MoveBaseHandler:

    def __init__(self, robot_name, zenoh_session, node):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node

        self.goal_status_array = None
        self.active_goal_status = None
        self.previous_goal_status = None
        self._tmp_active_goal_id = None

        def _move_base_status_callback(sample: zenoh.Sample):
            try:
                goal_status_array = ROS1_STORE.deserialize_ros1(
                    sample.payload.to_bytes(),
                    GoalStatusArray.type_name
                )
                self.goal_status_array = goal_status_array
            except Exception as e:
                self.node.get_logger().debug(
                    f'Failed to deserialize GoalStatusArray payload: '
                    f'{type(e)}: {e}'
                )
                return

            active_goal_status = None
            for goal_status in goal_status_array.status_list:
                if goal_status.status == goal_status.ACTIVE:
                    active_goal_status = goal_status
                else:
                    self.previous_goal_status = goal_status
            self.active_goal_status = active_goal_status

        self.move_base_status_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f"/{namespacify('move_base/status', self.robot_name)}",
            msg_type=GoalStatusArray.type_name
        )
        self.move_base_status_sub = self.zenoh_session.declare_subscriber(
            self.move_base_status_zenoh_topic,
            _move_base_status_callback
        )

        self.move_base_simple_goal_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f"/{namespacify('move_base_simple/goal', self.robot_name)}",
            msg_type=PoseStamped.type_name
        )
        self.move_base_simple_goal_pub = self.zenoh_session.declare_publisher(
            self.move_base_simple_goal_zenoh_topic
        )

        self.move_base_cancel_goal_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f"/{namespacify('move_base/cancel', self.robot_name)}",
            msg_type=GoalID.type_name
        )
        self.move_base_cancel_goal_pub = self.zenoh_session.declare_publisher(
            self.move_base_cancel_goal_zenoh_topic
        )

    def get_goal_status_array(self) -> GoalStatusArray.msg_type | None:
        return self.goal_status_array

    def get_goal_status(self, goal_id: str) -> GoalStatus.msg_type | None:
        if self.active_goal_status is not None and \
                self.active_goal_status.goal_id.id == goal_id:
            return self.active_goal_status

        if self.previous_goal_status is not None and \
                self.previous_goal_status.goal_id.id == goal_id:
            return self.previous_goal_status

        return None

    def get_active_goal_status(self) -> GoalStatus.msg_type | None:
        return self.active_goal_status

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        timeout_sec: float = 3
    ) -> str | None:
        # Keep track of last status stamp to find the new goal status and ID
        if self.goal_status_array is None:
            self.node.get_logger().error(
                'move_base/status has not yet been received over zenoh topic '
                f'[{self.move_base_status_zenoh_topic}], unable to publish '
                'navigate_to_pose.'
            )
            return None
        last_status_stamp = self.goal_status_array.header.stamp

        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time.msg_type(sec=time_now[0], nanosec=time_now[1])
        header = Header.msg_type(seq=0, stamp=stamp, frame_id='map')
        position = Point.msg_type(x=x, y=y, z=z)
        quat = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion.msg_type(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )
        pose = Pose.msg_type(position=position, orientation=orientation)
        pose_stamped = PoseStamped.msg_type(header=header, pose=pose)

        serialized_msg = ROS1_STORE.serialize_ros1(
            pose_stamped,
            PoseStamped.msg_type.__msgtype__
        )
        self.move_base_simple_goal_pub.put(serialized_msg.tobytes())
        self.node.get_logger().info(
            'Sending move_base_simple/goal, over zenoh topic '
            f'[{self.move_base_simple_goal_zenoh_topic}], '
            f'msg: [{pose_stamped}]'
        )

        self._tmp_active_goal_id = None

        def _get_goal_id_callback(sample: zenoh.Sample):
            try:
                goal_status_array = ROS1_STORE.deserialize_ros1(
                    sample.payload.to_bytes(),
                    GoalStatusArray.type_name
                )
                self.goal_status_array = goal_status_array
            except Exception as e:
                self.node.get_logger().debug(
                    f'Failed to deserialize GoalStatusArray payload: '
                    f'{type(e)}: {e}'
                )
                return None

            for goal_status in goal_status_array.status_list:
                goal_status_stamp = goal_status.goal_id.stamp
                if goal_status.status == goal_status.ACTIVE and \
                        goal_status_stamp.sec >= last_status_stamp.sec and \
                        goal_status_stamp.nanosec >= last_status_stamp.nanosec:
                    self._tmp_active_goal_id = goal_status.goal_id.id

        move_base_status_sub = self.zenoh_session.declare_subscriber(
            self.move_base_status_zenoh_topic,
            _get_goal_id_callback
        )

        # Timeout to get move_base goal ID
        sleep_time = timeout_sec / 10
        for i in range(10):
            if self._tmp_active_goal_id is not None:
                break
            time.sleep(sleep_time)

        move_base_status_sub.undeclare()
        return self._tmp_active_goal_id

    def stop_current_navigation(self):
        if self.active_goal_status is None:
            return

        goal_id = self.active_goal_status.goal_id
        serialized_msg = ROS1_STORE.serialize_ros1(
            goal_id,
            GoalID.msg_type.__msgtype__
        )
        self.move_base_cancel_goal_pub.put(serialized_msg.tobytes())
        self.node.get_logger().info(
            'Sending move_base/cancel, over zenoh topic '
            f'[{self.move_base_cancel_goal_zenoh_topic}], '
            f'msg: [{goal_id}]'
        )

    def cancel_navigation(self, goal_id_str: str):
        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time.msg_type(sec=time_now[0], nanosec=time_now[1])
        goal_id = GoalID.msg_type(stamp=stamp, id=goal_id_str)
        serialized_msg = ROS1_STORE.serialize_ros1(
            goal_id,
            GoalID.msg_type.__msgtype__
        )
        self.move_base_cancel_goal_pub.put(serialized_msg.tobytes())
        self.node.get_logger().info(
            'Sending move_base/cancel, over zenoh topic '
            f'[{self.move_base_cancel_goal_zenoh_topic}], '
            f'msg: [{goal_id}]'
        )


class Nav1RobotAdapter(RobotAdapter):

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

        self.map_name = self.robot_config_yaml['initial_map']

        # TODO(ac): Only use full battery if sim is indicated
        self.battery_soc = 1.0

        self.replan_counts = 0

        self.tf_handler = Nav1TfHandler(
            self.name,
            self.zenoh_session,
            self.tf_buffer,
            self.node
        )
        self.move_base_handler = Nav1MoveBaseHandler(
            self.name,
            self.zenoh_session,
            self.node
        )
        self.nav_goal_id = None

        def _battery_state_callback(sample: zenoh.Sample):
            try:
                battery_state = ROS1_STORE.deserialize_ros1(
                    sample.payload.to_bytes(),
                    BatteryState.type_name
                )
            except Exception as e:
                self.node.get_logger().debug(
                    'Failed to deserialize BatteryState payload: '
                    f'{type(e)}: {e}'
                )
                return
            self.battery_soc = battery_state.percentage

        battery_state_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f"/{namespacify('battery_state', self.name)}",
            msg_type=BatteryState.type_name
        )
        self.battery_state_sub = self.zenoh_session.declare_subscriber(
            battery_state_zenoh_topic,
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

        nav_goal_id = self.nav_goal_id
        goal_status = \
            self.move_base_handler.get_goal_status(nav_goal_id)
        if goal_status is None:
            self.replan_counts += 1
            self.node.get_logger().error(
                f'Navigation goal {nav_goal_id} is not found, replan count '
                f'[{self.replan_counts}]'
            )
            self.update_handle.more().replan()
            return False

        match goal_status.status:
            case goal_status.PENDING | \
                 goal_status.ACTIVE | \
                 goal_status.PREEMPTING | \
                 goal_status.RECALLING:
                return False
            case goal_status.SUCCEEDED:
                self.node.get_logger().info(
                    f'Navigation goal {nav_goal_id} reached'
                )
                return True
            case goal_status.PREEMPTED:
                self.node.get_logger().info(
                    f'Navigation goal {nav_goal_id} was cancelled'
                )
                return True
            case _:
                # TODO(ac): test replanning behavior if goal status is
                # neither executing or succeeded
                self.replan_counts += 1
                self.node.get_logger().error(
                    f'Navigation goal {nav_goal_id} status '
                    f'{goal_status.status}, replan count '
                    f'[{self.replan_counts}]'
                )
                self.update_handle.more().replan()
                return False

    def _check_update_handle_initialization(self):
        if self.update_handle is None:
            error_message = \
                f'Failed to update robot {self.name}, robot adapter has ' \
                'not yet been initialized with a fleet update handle.'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)

    def update(self, state: rmf_easy.RobotState):
        self._check_update_handle_initialization()

        activity_identifier = None
        if self.execution:
            if self.nav_goal_id is not None and self._is_navigation_done():
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
        yaw: float,
        timeout_sec: float = 3.0
    ):
        if map_name != self.map_name:
            # TODO(ac): test this map related replanning behavior
            self.replan_counts += 1
            self.node.get_logger().error(
                f'Destination is on map [{map_name}], while robot '
                f'[{self.name}] is on map [{self.map_name}], replan count '
                f'[{self.replan_counts}]'
            )

            self._check_update_handle_initialization()
            self.update_handle.more().replan()
            return

        nav_goal_id = self.move_base_handler.navigate_to_pose(
            x,
            y,
            z,
            yaw,
            timeout_sec
        )

        if nav_goal_id is not None:
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
        self._check_update_handle_initialization()
        self.update_handle.more().replan()
        self.nav_goal_id = None

    def navigate(
        self,
        destination: rmf_easy.Destination,
        execution: rmf_easy.CommandExecution
    ):
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position}'
            f' on map [{destination.map}]'
        )
        self._handle_navigate_to_pose(
            destination.map,
            destination.position[0],
            destination.position[1],
            0.0,
            destination.position[2]
        )

    def _handle_stop_navigation(self):
        self.move_base_handler.stop_current_navigation()

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
