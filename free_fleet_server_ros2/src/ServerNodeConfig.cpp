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

#include <cstdio>

#include <free_fleet/ServerConfig.hpp>

#include "ServerNodeConfig.hpp"

namespace free_fleet
{
namespace ros2
{

void ServerNodeConfig::print_config() const
{
  setbuf(stdout, NULL);
  printf("ROS 2 SERVER CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  update state frequency: %.1f\n", update_state_frequency);
  printf("  publish state frequency: %.1f\n", publish_state_frequency);
  printf("  TOPICS\n");
  printf("    fleet state: %s\n", fleet_state_topic.c_str());
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
  printf("SERVER-CLIENT DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", dds_robot_state_topic.c_str());
  printf("    mode request: %s\n", dds_mode_request_topic.c_str());
  printf("    path request: %s\n", dds_path_request_topic.c_str());
  printf("    destination request: %s\n",
      dds_destination_request_topic.c_str());
  printf("COORDINATE TRANSFORMATION\n");
  printf("  translation x (meters): %.3f\n", translation_x);
  printf("  translation y (meters): %.3f\n", translation_y);
  printf("  rotation (radians): %.3f\n", rotation);
  printf("  scale: %.3f\n", scale);
}

ServerConfig ServerNodeConfig::get_server_config() const
{
  ServerConfig server_config;
  server_config.dds_domain = dds_domain;
  server_config.dds_robot_state_topic = dds_robot_state_topic;
  server_config.dds_mode_request_topic = dds_mode_request_topic;
  server_config.dds_path_request_topic = dds_path_request_topic;
  server_config.dds_destination_request_topic = dds_destination_request_topic;
  return server_config;
}

ServerNodeConfig ServerNodeConfig::make()
{
  ServerNodeConfig config;
  return config;
}

} // namespace ros2
} // namespace free_fleet
