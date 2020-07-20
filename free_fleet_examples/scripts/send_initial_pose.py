#!/usr/bin/env python3

# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import sys
import numpy as np
import argparse

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from geometry_msgs.msg import PoseWithCovarianceStamped

# https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def main(argv = sys.argv):
    default_topic = '/initialpose'
    default_frame = 'map'
    default_initial_x = 0.0
    default_initial_y = 0.0
    default_initial_z = 0.0
    default_initial_yaw = 0.0

    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic-name', default=default_topic)
    parser.add_argument('-f', '--frame', default=default_frame)
    parser.add_argument('-x', default=default_initial_x, type=float)
    parser.add_argument('-y', default=default_initial_y, type=float)
    parser.add_argument('-z', default=default_initial_z, type=float)
    parser.add_argument('--yaw', default=default_initial_yaw, type=float)
    args = parser.parse_args(remove_ros_args(argv[1:]))

    print('topic_name: {}'.format(args.topic_name))
    print('frame: {}'.format(args.frame))
    print('x: {}'.format(args.x))
    print('y: {}'.format(args.y))
    print('z: {}'.format(args.z))
    print('yaw: {}'.format(args.yaw))

    rclpy.init()
    node = rclpy.create_node('send_inital_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, args.topic_name, 10)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = args.frame
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.position.z = args.z
    quat = euler_to_quaternion(0.0, 0.0, args.yaw)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
  
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891945200942

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')


if __name__ == '__main__':
    main(sys.argv)
