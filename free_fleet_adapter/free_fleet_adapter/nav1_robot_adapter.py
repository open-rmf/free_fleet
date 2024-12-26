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
# from free_fleet.ros2_types import (
#     ActionMsgs_CancelGoal_Response,
#     GeometryMsgs_Point,
#     GeometryMsgs_Pose,
#     GeometryMsgs_PoseStamped,
#     GeometryMsgs_Quaternion,
#     GoalStatus,
#     Header,
#     NavigateToPose_GetResult_Request,
#     NavigateToPose_GetResult_Response,
#     NavigateToPose_SendGoal_Request,
#     NavigateToPose_SendGoal_Response,
#     SensorMsgs_BatteryState,
#     TFMessage,
#     Time,
# )
from free_fleet.ros1_types import (
    ROS1_STORE,
    TFMessage
)
from free_fleet.utils import (
    get_zenoh_name_of_ros1_topic,
    # make_cancel_all_goals_request,
    namespacify,
)

from geometry_msgs.msg import TransformStamped
# import numpy as np
import rclpy
# import rmf_adapter.easy_full_control as rmf_easy
# from rmf_adapter.robot_update_handle import ActivityIdentifier
# from tf_transformations import euler_from_quaternion, quaternion_from_euler

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
