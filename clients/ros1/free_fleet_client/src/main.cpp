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


int main(int argc, char** argv)
{
  // Initialize all the ROS 1 items
  ros::init(argc, argv, "free_fleet_client");
  ros::NodeHandle ros_node_handle;
  ROS_INFO("greetings from free_fleet_client");

  auto config = free_fleet::ClientConfig::make();
  
  auto client = free_fleet::Client::make(config);

  // Checks if the DDS client was created and is ready to roll
  if (!client)
  {
    ROS_ERROR("Client: unable to initialize.");
    return 1;
  }

  // Start running the client with the starting state
  client->start();
  return 0;
}
