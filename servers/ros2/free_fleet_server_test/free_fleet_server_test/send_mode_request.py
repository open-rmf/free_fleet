#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import ModeRequest


def send_mode_request(args):
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
    
    if args.mode == 'pause':
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


if __name__ == '__main__':
    fleet_name = 'fleet_name'
    robot_name = 'robot_name'
    mode = 'pause'
    task_id = 'yhuijnesdxunsd'
    topic_name = 'mode_request'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=fleet_name)
    parser.add_argument('-r', '--robot-name', default=robot_name)
    parser.add_argument('-m', '--mode', default=mode)
    parser.add_argument('-i', '--task-id', default=task_id)
    parser.add_argument('-t', '--topic-name', default=topic_name)
    args = parser.parse_args()

    send_mode_request(args)
