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

# # https://github.com/eclipse-zenoh/zenoh-plugin-ros1/issues/131

# import zenoh, time
# from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# TOPIC = '/tf'
# # TYPE = 'std_msgs/msg/String'
# TYPE = 'tf2_msgs/msg/TFMessage'

# # Noetic TFMessage message definition, taken from:
# #
# https://github.com/ros/geometry2/blob/noetic-devel/tf2_msgs/msg/TFMessage.msg
# TFMESSAGE_DEFINITION = """
# geometry_msgs/TransformStamped[] transforms
# """

# def get_zenoh_name_of_ros1_topic(ros1_store, topic: str, msg_type: str
# ) -> str:
#     # Get md5 and encode msg_type to construct zenoh topic
#     msg_type_split = msg_type.split('/')
#     msg_type_encoded = \
#         '/'.join([msg_type_split[0],msg_type_split[2]]).encode('utf-8').hex()
#     md5 = ros1_store.generate_msgdef(msg_type)[1]
#     zenoh_topic = '/'.join([msg_type_encoded, md5, topic[1:]])

#     return zenoh_topic


# def listener(sample: zenoh.Sample):
#     msg = ros1_store.deserialize_ros1(sample.payload.to_bytes(), TYPE)
#     print(f'ROS1 msg: {msg}')


# if __name__ == "__main__":
#     session = zenoh.open(zenoh.Config())
#     ros1_store = get_typestore(Stores.ROS1_NOETIC)

#     ros1_store.register(
#         get_types_from_msg(TFMESSAGE_DEFINITION, TYPE),
#     )

#     zenoh_topic = get_zenoh_name_of_ros1_topic(
#         ros1_store,
#         topic=TOPIC,
#         msg_type=TYPE
#     )
#     print(f'ROS topic {TOPIC} is converted to Zenoh {zenoh_topic}')
#     sub = session.declare_subscriber(zenoh_topic, listener)
#     time.sleep(60)
