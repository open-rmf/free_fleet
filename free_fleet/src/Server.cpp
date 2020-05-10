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

#include <dds/dds.h>

#include <free_fleet/Server.hpp>

#include "ServerImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server = SharedPtr(new Server(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>::SharedPtr state_sub(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>(
          participant, &FreeFleetData_RobotState_desc,
          _config.dds_robot_state_topic));

  dds::DDSPublishHandler<FreeFleetData_ModeRequest>::SharedPtr 
      mode_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_ModeRequest>(
              participant, &FreeFleetData_ModeRequest_desc,
              _config.dds_mode_request_topic));

  dds::DDSPublishHandler<FreeFleetData_PathRequest>::SharedPtr 
      path_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_PathRequest>(
              participant, &FreeFleetData_PathRequest_desc,
              _config.dds_path_request_topic));

  dds::DDSPublishHandler<FreeFleetData_DestinationRequest>::SharedPtr 
      destination_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_DestinationRequest>(
              participant, &FreeFleetData_DestinationRequest_desc,
              _config.dds_destination_request_topic));

  if (!state_sub->is_ready() ||
      !mode_request_pub->is_ready() ||
      !path_request_pub->is_ready() ||
      !destination_request_pub->is_ready())
    return nullptr;

  server->impl->start(ServerImpl::Fields{
      std::move(participant),
      std::move(state_sub),
      std::move(mode_request_pub),
      std::move(path_request_pub),
      std::move(destination_request_pub)});
  return server;
}

Server::Server(const ServerConfig& _config)
{
  impl.reset(new ServerImpl(_config));
}

Server::~Server()
{}

bool Server::read_robot_states(
    std::vector<messages::RobotState>& _new_robot_states)
{
  return impl->read_robot_states(_new_robot_states);
}

bool Server::send_mode_request(const messages::ModeRequest& _mode_request)
{
  return impl->send_mode_request(_mode_request);
}

bool Server::send_path_request(const messages::PathRequest& _path_request)
{
  return impl->send_path_request(_path_request);
}

bool Server::send_destination_request(
    const messages::DestinationRequest& _destination_request)
{
  return impl->send_destination_request(_destination_request);
}

} // namespace free_fleet
