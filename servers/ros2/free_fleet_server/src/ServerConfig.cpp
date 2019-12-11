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

#include "ServerConfig.hpp"

namespace free_fleet
{

void ServerConfig::print_config()
{
  std::cout << "SERVER CONFIGURATION :" << std::endl;
  std::cout << "Fleet Name                         : " << fleet_name 
      << std::endl;
  std::cout << "ROS 2  - Fleet State Topic         : " << fleet_state_topic 
      << std::endl;
  std::cout << "ROS 2  - Mode Request Topic        : " << mode_request_topic 
      << std::endl;
  std::cout << "ROS 2  - Path Reqeust Topic        : " << path_request_topic 
      << std::endl;
  std::cout << "ROS 2  - Destination Request Topic : " 
      << destination_request_topic << std::endl;
  std::cout << "DDS    - DDS Domain                : " << dds_domain 
      << std::endl;
  std::cout << "DDS    - Robot State Topic         : " << dds_robot_state_topic 
      << std::endl;
  std::cout << "DDS    - Mode Request Topic        : " << dds_mode_request_topic 
      << std::endl;
  std::cout << "DDS    - Path Request Topic        : " << dds_path_request_topic 
      << std::endl;
  std::cout << "DDS    - Destination Request Topic : " 
      << dds_destination_request_topic << std::endl;

  std::cout << "Server - Update State Frequency    : " << update_state_frequency 
      << std::endl;
  std::cout << "Server - Publish State Frequency   : " 
      << publish_state_frequency << std::endl;

  std::cout << "Server - Coordinate Transformation : ";
  for (size_t i = 0; i < 9; ++i)
    std::cout << transformation[i] << ", ";
  std::cout << std::endl;
}

} // namespace free_fleet
