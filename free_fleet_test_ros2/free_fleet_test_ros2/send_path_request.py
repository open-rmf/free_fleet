#!/usr/bin/env python3
 
# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import json
import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import PathRequest


def print_path(json_path_string):
    path_dict = json.loads(json_path_string)
    for waypoint in path_dict:
        print('01 x: {} y: {} yaw: {} level_name: {}'.format(
                waypoint['x'], waypoint['y'], waypoint['yaw'],
                waypoint['level_name']))


def get_path_message(json_path_string):
    path_dict = json.loads(json_path_string)
    path = []
    for waypoint in path_dict:
        new_wp = Location()
        # ignore time for now
        new_wp.x = float(waypoint['x'])
        new_wp.y = float(waypoint['y'])
        new_wp.yaw = float(waypoint['yaw'])
        new_wp.level_name = waypoint['level_name']
        path.append(new_wp)
    return path


def main(argv = sys.argv):
    '''
    Example path request:
    - fleet_name: magni
    - robot_name: magni123
    - task_id: tfuhbjndsujewsduisd
    - path: [
        {"x": 0.0, "y": 0.0, "yaw": 0.0, "level_name": "B1"},
        {"x": 1.0, "y": 1.0, "yaw": 1.0, "level_name": "B1"},
        {"x": 2.0, "y": 2.0, "yaw": 2.0, "level_name": "B1"}]
    '''

    default_fleet_name = 'fleet_name'
    default_robot_name = 'robot_name'
    default_task_id = 'tfuhbjndsujewsduisd'
    default_desired_path = '[]'
    default_topic_name = 'robot_path_requests'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-r', '--robot-name', default=default_robot_name)
    parser.add_argument('-i', '--task-id', default=default_task_id)
    parser.add_argument('-p', '--path', default=default_desired_path)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('robot_name: {}'.format(args.robot_name))
    print('task_id: {}'.format(args.task_id))
    print_path(args.path)
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_path_request_node')
    pub = node.create_publisher(PathRequest, args.topic_name, 10)

    msg = PathRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.task_id = args.task_id
    msg.path = get_path_message(args.path)

    rclpy.spin_once(node, timeout_sec=2.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')


if __name__ == '__main__':
    main(sys.argv)
