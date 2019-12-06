#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import DestinationRequest


def send_destination_request(args):
    print('fleet_name: {}'.format(args.fleet_name))
    print('robot_name: {}'.format(args.robot_name))
    print('x: {}'.format(args.x))
    print('y: {}'.format(args.y))
    print('yaw: {}'.format(args.yaw))
    print('task_id: {}'.format(args.task_id))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_destination_request_node')
    pub = node.create_publisher(DestinationRequest, args.topic_name, 10)

    msg = DestinationRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.task_id = args.task_id
    # ignore time for now
    msg.destination.x = args.x
    msg.destination.y = args.y
    msg.destination.yaw = args.yaw

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')


if __name__ == '__main__':
    fleet_name = 'fleet_name'
    robot_name = 'robot_name'
    desired_x = 0.0
    desired_y = 0.0
    desired_yaw = 0.0
    level_name = 'B1'
    task_id = 'yhuijnesdxunsd'
    topic_name = 'destination_request'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=fleet_name)
    parser.add_argument('-r', '--robot-name', default=robot_name)
    parser.add_argument('--x', default=desired_x)
    parser.add_argument('--y', default=desired_y)
    parser.add_argument('--yaw', default=desired_yaw)
    parser.add_argument('-i', '--task-id', default=task_id)
    parser.add_argument('-t', '--topic-name', default=topic_name)
    args = parser.parse_args()

    send_destination_request(args)
