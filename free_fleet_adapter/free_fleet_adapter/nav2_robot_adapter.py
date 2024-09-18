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
import zenoh

from geometry_msgs.msg import TransformStamped

from free_fleet.types import (
    ActionMsgs_CancelGoal_Response,
    GeometryMsgs_Point,
    GeometryMsgs_Pose,
    GeometryMsgs_PoseStamped,
    GeometryMsgs_Quaternion,
    GoalStatus,
    Header,
    NavigateToPose_Feedback,
    NavigateToPose_GetResult_Request,
    NavigateToPose_GetResult_Response,
    NavigateToPose_SendGoal_Request,
    NavigateToPose_SendGoal_Response,
    TFMessage,
    Time,
)
from free_fleet.utils import (
    make_cancel_all_goals_request,
    namespace_frame,
    namespace_topic,
)

import rmf_adapter.easy_full_control as rmf_easy

from tf_transformations import quaternion_from_euler


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
        self.map = self.robot_config_yaml["initial_map"]
        self.battery_soc = None

        def _tf_callback(sample: zenoh.Sample):
            transform = TFMessage.deserialize(sample.payload)
            for zt in transform.transforms:
                time = Time(
                    seconds=zt.header.stamp.sec,
                    nanoseconds=zt.header.stamp.nanosec
                )
                t = TransformStamped()
                t.header.stamp = time.to_msg()
                t.header.stamp
                t.header.frame_id = namespace_frame(zt.header.frame_id, self.name)
                t.child_frame_id = namespace_frame(zt.child_frame_id, self.name)
                t.transform.translation.x = zt.transform.translation.x
                t.transform.translation.y = zt.transform.translation.y
                t.transform.translation.z = zt.transform.translation.z
                t.transform.rotation.x = zt.transform.rotation.x
                t.transform.rotation.y = zt.transform.rotation.y
                t.transform.rotation.z = zt.transform.rotation.z
                t.transform.rotation.w = zt.transform.rotation.w
                print("adding transform")
                self.tf_buffer.set_transform(t, f"{self.name}_RobotAdapter")

        self.tf_sub = self.zenoh_session.declare_subscriber(
            namespace_topic("tf", name),
            _tf_callback
        )

        # def _feedback_callback(sample: zenoh.Sample):
        #     feedback = NavigateToPose_Feedback.deserialize(sample.payload)
        #     print(f"Distance remaining: {feedback.distance_remaining}")

        # self.navigate_to_pose_action_feedback = self.zenoh_session.declare_subscriber(
        #     namespace_topic("navigate_to_pose/_action/feedback", name),
        #     _feedback_callback
        # )

        def _battery_state_callback(sample: zenoh.Sample):
            self.battery_soc = None

        self.battery_state_sub = self.zenoh_session.declare_subscriber(
            namespace_topic("battery_state", name),
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
            namespace_topic("navigate_to_pose/_action/get_result", self.name),
            zenoh.Queue(),
            value=req.serialize(),
            timeout=0.5
        )
        for reply in replies.receiver:
            try:
                # Deserialize the response
                rep = NavigateToPose_GetResult_Response.deserialize(
                    reply.ok.payload
                )
                print(f"Result: {rep.status}")
                if rep.status == GoalStatus.STATUS_EXECUTING:
                    return False
                elif rep.status == GoalStatus.STATUS_SUCCEEDED:
                    self.node.get_logger().info(
                        f"Navigation goal {self.nav_goal_id} reached"
                    )
                    return True
                else:
                    self.node.get_logger().error(
                        f"Navigation goal {self.nav_goal_id} status {rep.status}"
                    )
                    return True
            except:
                print("Received (ERROR: '{}')"
                    .format(reply.err.payload.decode("utf-8")))
                continue

    def update(self, state):
        activity_identifier = None
        if self.execution:
            # TODO(ac): use an enum to record what type of execution it is,
            # whether navigation or custom executions
            if self.nav_goal_id is not None and self._is_navigation_done(self):
                self.execution.finished()
                self.execution = None
                self.nav_goal_id = None
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
            self.node.get_logger().error(
                f"Destination is on map [{destination.map}], while robot [{self.name}] is on map [{self.map}]"
            )
            return

        time_now = self.node.get_clock().now()
        stamp = Time(sec=time_now.seconds, nanosec=time_now.nanoseconds)
        header = Header(stamp=stamp, frame_id="map")
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
            behavior_tree=""
        )

        replies = self.zenoh_session.get(
            namespace_topic("navigate_to_pose/_action/send_goal", self.name),
            zenoh.Queue(),
            value=req.serialize(),
            timeout=0.5
        )
        for reply in replies.receiver:
            try:
                rep = NavigateToPose_SendGoal_Response.deserialize(reply.ok.payload)
                if rep.accepted:
                    self.node.get_logger().info(
                        f"Navigation goal {nav_goal_id} accepted"
                    )
                    self.nav_goal_id = nav_goal_id
                    return

                self.node.get_logger().error(
                    f"Navigation goal {nav_goal_id} was rejected"
                )
                self.nav_goal_id = None
                return
            except:
                payload = reply.err.payload.decode("utf-8")
                self.node.get_logger().error(f"Received (ERROR: {payload})")
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
                    namespace_topic(
                        "navigate_to_pose/_action/cancel_goal",
                        self.name,
                        timeout=0.5
                    ),
                    zenoh.Queue(),
                    value=req.serialize()
                )
                for reply in replies.receiver:
                    rep = ActionMsgs_CancelGoal_Response.deserialize(
                        reply.ok.payload
                    )
                    self.node.get_logger().info(
                        "Return code: %d" % rep.return_code
                    )
                self.nav_goal_id = None

    def execute_action(self, category: str, description: dict, execution):
        self.execution = execution
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        # TODO(ac): change map using map_server load_map, and set initial
        # position again with /initialpose
        # TODO(ac): docking
        return
