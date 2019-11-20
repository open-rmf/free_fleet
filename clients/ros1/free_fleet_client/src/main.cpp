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

#include "Client.hpp"
#include "FreeFleet.h"

int main(int argc, char** argv)
{
  // Initialize all the ROS 1 items
  ros::init(argc, argv, "free_fleet_client");
  ros::NodeHandle ros_node_handle;
  ROS_INFO("greetings from free_fleet_client");

  // Will most likely only need to define the fleet_name
  std::string fleet_name = "fake_fleet";
  auto client = free_fleet::Client::make(fleet_name);

  // Checks if the DDS client was created and is ready to roll
  if (!client || !client->is_ready())
    return 1;

  // Create a starting state
  ros::Time t_start(ros::Time::now());
  FreeFleetData_RobotState msg;
  msg.name = "robot_name";
  msg.model = "robot_model";
  msg.mode.mode = FreeFleetData_RobotMode_Constants_MODE_IDLE;
  msg.battery_percent = 100.0;
  msg.location.sec = t_start.sec;
  msg.location.nanosec = t_start.nsec;
  msg.location.x = 1.0;
  msg.location.y = 2.0;
  msg.location.yaw = 3.0;
  msg.location.level_name = "L1";
  msg.path._maximum = 0;
  msg.path._length = 0;
  msg.path._buffer = NULL;
  msg.path._release = true;  // not sure what this means

  // Start running the client with the starting state
  if (!client->start(msg))
    return 1;

  // Periodically updating the state to test the client
  ros::Time t_prev_send(ros::Time::now());
  while (ros::ok())
  {
    ros::Time t(ros::Time::now());
    if ((t - t_prev_send).toSec() > 2.0)
    {
      t_prev_send = t;
      msg.location.sec = t.sec;
      msg.location.nanosec = t.nsec;

      client->update_robot_state(msg);
    }
  }
  
  return 0;
}
