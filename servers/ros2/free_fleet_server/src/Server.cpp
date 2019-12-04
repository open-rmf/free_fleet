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

#include "Server.hpp"

namespace free_fleet
{

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server(new Server(_config));
  if (!server->is_ready())
    return nullptr;

  return server;
}

Server::~Server()
{}

bool Server::is_ready()
{
  return ready;
}

void Server::start()
{
  if (!is_ready())
  {
    RCLCPP_ERROR(get_logger(), "Server: is not ready, can't start");
    return;
  }

  using namespace std::chrono_literals;

  update_timer = create_wall_timer(
      100ms, std::bind(&Server::update_callback, this));
}

Server::Server(const ServerConfig& _config) :
  Node(_config.fleet_name + "_free_fleet_server"),
  server_config(_config)
{
  ready = false;

  participant = dds_create_participant(
    static_cast<dds_domainid_t>(server_config.dds_domain), NULL, NULL);

  robot_state_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotState>(
          participant, &FreeFleetData_RobotState_desc,
          server_config.dds_robot_state_topic));
  if (!robot_state_sub->is_ready())
    return;

  ready = true;
}

void Server::update_state_callback()
{
  auto new_robot_state = robot_state_sub->read();

  if (!new_robot_state)
    RCLCPP_INFO(get_logger(), "getting nothing yet.");
  else
    RCLCPP_INFO(get_logger(), "got a state through dds!");
}

} // namespace free_fleet
