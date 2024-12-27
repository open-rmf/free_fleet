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

# from typing import Annotated

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros1_types import (
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

from geometry_msgs.msg import TransformStamped
# import numpy as np
import rclpy
# import rmf_adapter.easy_full_control as rmf_easy
# from rmf_adapter.robot_update_handle import ActivityIdentifier
# from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf_transformations import quaternion_from_euler

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
            topic=f'/{self.robot_name}/tf',
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
            self.active_goal_status = active_goal_status

        self.move_base_status_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f'/{self.robot_name}/move_base/status',
            msg_type=GoalStatusArray.type_name
        )
        self.move_base_status_sub = self.zenoh_session.declare_subscriber(
            self.move_base_status_zenoh_topic,
            _move_base_status_callback
        )

        self.move_base_simple_goal_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f'/{self.robot_name}/move_base_simple/goal',
            msg_type=PoseStamped.type_name
        )
        self.move_base_simple_goal_pub = self.zenoh_session.declare_publisher(
            self.move_base_simple_goal_zenoh_topic
        )

        self.move_base_cancel_goal_zenoh_topic = get_zenoh_name_of_ros1_topic(
            ROS1_STORE,
            topic=f'/{self.robot_name}/move_base/cancel',
            msg_type=GoalID.type_name
        )
        self.move_base_cancel_goal_pub = self.zenoh_session.declare_publisher(
            self.move_base_cancel_goal_zenoh_topic
        )

    def get_goal_statuses(self) -> GoalStatusArray.msg_type | None:
        return self.goal_status_array

    def get_active_goal_status(self) -> GoalStatus.msg_type | None:
        return self.active_goal_status

    def is_navigation_done(self) -> bool:
        if self.active_goal_status is None:
            return True
        return False

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float
    ):
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

    def stop_navigation(self):
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
