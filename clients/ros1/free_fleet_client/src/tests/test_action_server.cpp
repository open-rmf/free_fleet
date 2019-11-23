/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>


using MoveBaseServer = 
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;

void execute(
    const move_base_msgs::MoveBaseGoalConstPtr& _goal, 
    MoveBaseServer* _server)
{
  std::cout << "got a action service call." << std::endl;
  // do lots of awesome groundbreaking robot stuff here
  _server->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_move_base_server");
  ros::NodeHandle n;
  MoveBaseServer server(
      n, "move_base", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
