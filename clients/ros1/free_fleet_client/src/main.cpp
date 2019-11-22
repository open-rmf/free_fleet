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

/// We should expect no loss in communications between the robot stack and this
/// node, as they would normally be running on the same computer.
/// Troubleshooting and error handling will be implemented later on after DP2, 
/// right now the client expects all the fields to be updated via the different
/// topics regularly, which can then be published through DDS to the server.
///
/// Notes: 
/// * the time of the state will be tied to the transform
/// * location comes from listening to transforms
/// * battery from listening to a std_msgs/Float32
/// * mode will be using free_fleet_msgs/RobotMode
/// * path will be using free_fleet_msgs/PathSequence
/// * level name will be a std_msgs/String
/// * calling robot commands using move_base_msgs/MoveBaseAction
/// 
/// TODO: 
/// * need to sync up the level topic with the transform of the robot, one
/// might be published after the other, the server might catch fire then
/// * handle losing connection with action server
///

int main(int argc, char** argv)
{
  // Initialize all the ROS 1 items
  ros::init(argc, argv, "free_fleet_client");
  ros::NodeHandle ros_node_handle;
  ROS_INFO("greetings from free_fleet_client");

  // Will most likely only need to define the fleet_name
  free_fleet::ClientConfig config;
  config.fleet_name = "fake_fleet";
  config.robot_name = "fake_robot";
  config.robot_model = "Magni";
  auto client = free_fleet::Client::make(config);

  // Checks if the DDS client was created and is ready to roll
  if (!client->is_ready())
  {
    ROS_ERROR("Client: unable to initialize.");
    return 1;
  }

  // Start running the client with the starting state
  client->start();
  return 0;
}
