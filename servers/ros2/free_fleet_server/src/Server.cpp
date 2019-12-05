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

Server::Server(const ServerConfig& _config) :
  Node(_config.fleet_name + "_free_fleet_server"),
  server_config(_config)
{
  ready = false;

  participant = dds_create_participant(
    static_cast<dds_domainid_t>(server_config.dds_domain), NULL, NULL);

  dds_robot_state_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>(
          participant, &FreeFleetData_RobotState_desc,
          server_config.dds_robot_state_topic));
  if (!dds_robot_state_sub->is_ready())
    return;

  ready = true;
}

void Server::start()
{
  if (!is_ready())
  {
    RCLCPP_ERROR(get_logger(), "Server: is not ready, can't start");
    return;
  }

  using namespace std::chrono_literals;
  
  update_state_timer = create_wall_timer(
      100ms, std::bind(&Server::update_state_callback, this));

  using std::placeholders::_1;
  
  mode_request_sub = create_subscription<ModeRequest>(
      server_config.mode_request_topic, 10, std::bind(&Server::mode_request_callback, this, _1));
  
  path_request_sub = create_subscription<PathRequest>(
      server_config.path_request_topic, 10, std::bind(&Server::path_request_callback, this, _1));

  destination_request_sub = create_subscription<DestinationRequest>(
      server_config.destination_request_topic, 10, 
      std::bind(&Server::destination_request_callback, this, _1));
}

void Server::update_state_callback()
{
  std::vector<std::shared_ptr<const FreeFleetData_RobotState>> incoming_states;
  dds_robot_state_sub->read(incoming_states);

  RCLCPP_INFO(get_logger(), "got %u states!", incoming_states.size());

  // for (auto new_robot_state : incoming_states)
  // {

  // }
}

void Server::mode_request_callback(ModeRequest::UniquePtr _msg)
{

}

void Server::path_request_callback(PathRequest::UniquePtr _msg)
{

}

void Server::destination_request_callback(DestinationRequest::UniquePtr _msg)
{

}

} // namespace free_fleet
