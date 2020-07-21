/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <chrono>

#include "ClientNode.hpp"

namespace free_fleet
{

//==============================================================================

void ClientNode::Config::print_config() const
{  
  setbuf(stdout, NULL);
  printf("ROS 2 CLIENT CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  robot name: %s\n", robot_name.c_str());
  printf("  robot model: %s\n", robot_model.c_str());
  printf("  wait timeout: %d\n", wait_timeout);
  printf("  publish state frequency: %d\n", publish_state_frequency);
  printf("  handle request frequency: %d\n", handle_request_frequency);
  printf("  maximum distance to first waypoint: %.1f\n", 
      max_dist_to_first_waypoint);
  printf("  TOPICS\n");
  printf("    battery state: %s\n", battery_state_topic.c_str());
  printf("    level name: %s\n", level_name_topic.c_str());
  printf("    action server: %s\n", action_server_name.c_str());
  printf("  ROBOT FRAMES\n");
  printf("    map frame: %s\n", map_frame.c_str());
  printf("    robot frame: %s\n", robot_frame.c_str());
  printf("CLIENT-SERVER DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", dds_robot_state_topic.c_str());
  printf("    mode request: %s\n", dds_mode_request_topic.c_str());
  printf("    path request: %s\n", dds_path_request_topic.c_str());
  printf("    destination request: %s\n", 
      dds_destination_request_topic.c_str());
}

//==============================================================================

Client::Config ClientNode::Config::get_client_config() const
{
  Client::Config client_config {
    dds_domain,
    dds_robot_state_topic,
    dds_mode_request_topic,
    dds_path_request_topic,
    dds_destination_request_topic
  };
  return client_config;
}

//==============================================================================

ClientNode::SharedPtr ClientNode::make(Config config)
{
  SharedPtr client_node(new ClientNode(config));

  if (client_node)
    client_node->_config.print_config();

  auto action_client = rclcpp_action::create_client<NavigateToPose>(
      client_node, config.action_server_name);
  if (!action_client)
    return nullptr;
  // Make sure the server is actually there before continuing
  RCLCPP_INFO(
      client_node->get_logger(), 
      "Waiting for \"%s\" action server", 
      config.action_server_name.c_str());
  if (!action_client->wait_for_action_server(
          std::chrono::seconds(config.wait_timeout)))
  {
    RCLCPP_ERROR(
        client_node->get_logger(),
        "timed out waiting for action server: %s",
        config.action_server_name.c_str());
    return nullptr;
  }

  client_node->_action_client = std::move(action_client);
  return client_node;
}

//==============================================================================

ClientNode::~ClientNode()
{}

//==============================================================================

void ClientNode::init_ros()
{
  _publish_state_timer = create_wall_timer(
      std::chrono::milliseconds(1000 / _config.publish_state_frequency), 
      std::bind(&ClientNode::publish_state, this));
  _handle_request_timer = create_wall_timer(
      std::chrono::milliseconds(1000 / _config.handle_request_frequency), 
      std::bind(&ClientNode::handle_request, this));
}

//==============================================================================

void ClientNode::publish_state()
{}

//==============================================================================

void ClientNode::handle_request()
{}

//==============================================================================

ClientNode::ClientNode(Config config)
: Node(config.robot_name + "_node"),
  _tf2_buffer(this->get_clock()),
  _tf2_listener(_tf2_buffer),
  _config(std::move(config))
{}

//==============================================================================

} // namespace free_fleet
