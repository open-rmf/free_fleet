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
#include <free_fleet/Participant.hpp>
#include <free_fleet/StateSubscriber.hpp>
#include <free_fleet/RequestPublisher.hpp>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

//==============================================================================

class Server::Implementation
{
public:
  Config _config;
  Participant::SharedPtr _participant;
  StateSubscriber::SharedPtr _state_subscriber;
  RequestPublisher::SharedPtr _request_publisher;

  Implementation(
      Config config,
      Participant::SharedPtr participant,
      StateSubscriber::SharedPtr state_subscriber,
      RequestPublisher::SharedPtr request_publisher)
  : _config(std::move(config)),
    _participant(std::move(participant)),
    _state_subscriber(std::move(state_subscriber)),
    _request_publisher(std::move(request_publisher))
  {}

  ~Implementation()
  {}
};

//==============================================================================

void Server::Config::print_config() const
{
  printf("SERVER-CLIENT DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", domain_id);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", robot_state_topic.c_str());
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
}

//==============================================================================

Server::SharedPtr Server::make(Config config)
{
  auto participant_ptr = Participant::make(config.domain_id);
  if (!participant_ptr)
    return nullptr;

  StateSubscriber::Config ss_config {
    config.domain_id,
    config.robot_state_topic
  };
  auto state_subscriber_ptr = StateSubscriber::make(
      std::move(ss_config), participant_ptr);
  if (!state_subscriber_ptr)
    return nullptr;

  RequestPublisher::Config rp_config {
    config.domain_id,
    config.mode_request_topic,
    config.path_request_topic,
    config.destination_request_topic
  };
  auto request_publisher_ptr = RequestPublisher::make(
      std::move(rp_config), participant_ptr);
  if (!request_publisher_ptr)
    return nullptr;

  SharedPtr server_ptr = SharedPtr(new Server);
  server_ptr->_pimpl.reset(
      new Implementation(
          std::move(config),
          std::move(participant_ptr),
          std::move(state_subscriber_ptr),
          std::move(request_publisher_ptr)));
  return server_ptr;
}

//==============================================================================

bool Server::read_robot_states(
    std::vector<messages::RobotState>& new_robot_states)
{
  return _pimpl->_state_subscriber->read_robot_states(new_robot_states);
}

//==============================================================================

bool Server::send_mode_request(const messages::ModeRequest& mode_request)
{
  return _pimpl->_request_publisher->send_mode_request(mode_request);
}

//==============================================================================

bool Server::send_path_request(const messages::PathRequest& path_request)
{
  return _pimpl->_request_publisher->send_path_request(path_request);
}

//==============================================================================

bool Server::send_destination_request(
    const messages::DestinationRequest& destination_request)
{
  return 
      _pimpl->_request_publisher->send_destination_request(destination_request);
}

//==============================================================================

Server::~Server()
{}

//==============================================================================

Server::Server()
{}

//==============================================================================

} // namespace free_fleet
