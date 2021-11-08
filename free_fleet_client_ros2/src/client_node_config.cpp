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

#include "free_fleet/ros2/client_node_config.hpp"

namespace free_fleet
{
namespace ros2
{

void ClientNodeConfig::print_config() const
{
  printf("ROS 2 CLIENT CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  robot name: %s\n", robot_name.c_str());
  printf("  robot model: %s\n", robot_model.c_str());
  printf("  level name: %s\n", level_name.c_str());
  printf("  wait timeout: %.1f\n", wait_timeout);
  printf("  update request frequency: %.1f\n", update_frequency);
  printf("  publish state frequency: %.1f\n", publish_frequency);
  printf("  maximum distance to first waypoint: %.1f\n", 
      max_dist_to_first_waypoint);
  printf("  TOPICS\n");
  printf("    battery state: %s\n", battery_state_topic.c_str());
  printf("    move base server: %s\n", move_base_server_name.c_str());
  printf("    docking trigger server: %s\n", docking_trigger_server_name.c_str());
  printf("  ROBOT FRAMES\n");
  printf("    map frame: %s\n", map_frame.c_str());
  printf("    robot frame: %s\n", robot_frame.c_str());
  printf("CLIENT-SERVER DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", dds_state_topic.c_str());
  printf("    mode request: %s\n", dds_mode_request_topic.c_str());
  printf("    path request: %s\n", dds_path_request_topic.c_str());
  printf("    destination request: %s\n", 
      dds_destination_request_topic.c_str());
  fflush(stdout);
}
  
ClientConfig ClientNodeConfig::get_client_config() const
{
  ClientConfig client_config;
  client_config.dds_domain = dds_domain;
  client_config.dds_state_topic = dds_state_topic;
  client_config.dds_mode_request_topic = dds_mode_request_topic;
  client_config.dds_path_request_topic = dds_path_request_topic;
  client_config.dds_destination_request_topic = dds_destination_request_topic;
  return client_config;
}

} // namespace ros2
} // namespace free_fleet