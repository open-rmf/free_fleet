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
import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import ModeRequest


def main(argv = sys.argv):
    '''
    Example mode request:
    - fleet_name: magni
    - robot_name: magni123
    - task_id: 576y13ewgyffeijuais
    - mode.mode: PAUSED
    '''

    default_fleet_name = 'fleet_name'
    default_robot_name = 'robot_name'
    default_task_id = '576y13ewgyffeijuais'
    default_mode = 'mode'
    default_topic_name = 'robot_mode_requests'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-r', '--robot-name', default=default_robot_name)
    parser.add_argument('-m', '--mode', default=default_mode)
    parser.add_argument('-i', '--task-id', default=default_task_id)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('robot_name: {}'.format(args.robot_name))
    print('mode: {}'.format(args.mode))
    print('task_id: {}'.format(args.task_id))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_mode_request_node')
    pub = node.create_publisher(ModeRequest, args.topic_name, 10)

    msg = ModeRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.task_id = args.task_id
    
    if args.mode == 'mode':
        print('Please insert desired mode, pause or resume')
        return
    elif args.mode == 'pause':
        msg.mode.mode = RobotMode.MODE_PAUSED
    elif args.mode == 'resume':
        msg.mode.mode = RobotMode.MODE_MOVING
    else:
        print('unrecognized mode requested, only use pause or resume please')
        return
  
    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
