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

# https://github.com/eclipse-zenoh/zenoh-plugin-ros1/issues/131

# import zenoh, time
# from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# BRIDGE_NAMESPACE = '/tb3_0'
# TOPIC = '/move_base/cancel'
# TYPE = 'actionlib_msgs/msg/GoalID'

# def get_zenoh_name_of_ros1_topic(
#     ros1_store,
#     topic: str,
#     msg_type: str
# ) -> str:
#     # Get md5 and encode msg_type to construct zenoh topic
#     msg_type_split = msg_type.split('/')
#     msg_type_encoded = \
#         '/'.join([msg_type_split[0],msg_type_split[2]]).encode('utf-8').hex()
#     md5 = ros1_store.generate_msgdef(msg_type)[1]
#     zenoh_topic = '/'.join([msg_type_encoded, md5, topic[1:]])

#     return zenoh_topic


# if __name__ == "__main__":
#     session = zenoh.open(zenoh.Config())
#     ros1_store = get_typestore(Stores.ROS1_NOETIC)
#     GoalID = ros1_store.types[TYPE]
#     Stamp = ros1_store.types['builtin_interfaces/msg/Time']

#     zenoh_topic = get_zenoh_name_of_ros1_topic(
#         ros1_store,
#         topic=f'{BRIDGE_NAMESPACE}{TOPIC}',
#         msg_type=TYPE
#     )
#     print(f'ROS topic {TOPIC} is converted to Zenoh {zenoh_topic}')

#     # Declare a publisher (optional but allows Zenoh to perform some
#     # optimizations)
#     pub = session.declare_publisher(zenoh_topic)

#     stamp = Stamp(sec=0, nanosec=0)
#     msg = GoalID(
#         stamp=stamp,
#         id='/move_base-1-36.21000000'
#     )

#     print('Publishing: "{0}"'.format(msg))
#     # Publish the serialized message
#     serialized_msg = ros1_store.serialize_ros1(msg, GoalID.__msgtype__)
#     pub.put(serialized_msg.tobytes())
#     time.sleep(1)
#     pub.undeclare()
#     session.close()

# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped
# '{ header: { frame_id:  "map"}, pose: { position: { x: 1.808, y: 0.503 },
# orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
