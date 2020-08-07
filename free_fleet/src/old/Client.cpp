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

#include <free_fleet/Client.hpp>
#include <free_fleet/Participant.hpp>

#include "messages/FleetMessages.h"
#include "messages/message_utils.hpp"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

//==============================================================================

class Client::Implementation
{
public:
  using RobotStatePub = dds::DDSPublishHandler<FreeFleetData_RobotState>;
  using ModeRequestSub = dds::DDSSubscribeHandler<FreeFleetData_ModeRequest>;
  using PathRequestSub = dds::DDSSubscribeHandler<FreeFleetData_PathRequest>;
  using DestinationRequestSub =
      dds::DDSSubscribeHandler<FreeFleetData_DestinationRequest>;

  Config _config;
  Participant::SharedPtr _participant;
  RobotStatePub::SharedPtr _robot_state_pub;
  ModeRequestSub::SharedPtr _mode_request_sub;
  PathRequestSub::SharedPtr _path_request_sub;
  DestinationRequestSub::SharedPtr _destination_request_sub;

  Implementation(
      Config config,
      Participant::SharedPtr participant,
      RobotStatePub::SharedPtr robot_state_pub,
      ModeRequestSub::SharedPtr mode_request_sub,
      PathRequestSub::SharedPtr path_request_sub,
      DestinationRequestSub::SharedPtr destination_request_sub)
  : _config(std::move(config)),
    _participant(std::move(participant)),
    _robot_state_pub(std::move(robot_state_pub)),
    _mode_request_sub(std::move(mode_request_sub)),
    _path_request_sub(std::move(path_request_sub)),
    _destination_request_sub(std::move(destination_request_sub))
  {}
  
  ~Implementation()
  {}
};

//==============================================================================

void Client::Config::print_config() const
{
  printf("CLIENT-SERVER DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", domain_id);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", robot_state_topic.c_str());
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
}

//==============================================================================

Client::SharedPtr Client::make(Config config)
{
  auto participant_ptr = Participant::make(config.domain_id);
  if (!participant_ptr)
    return nullptr;

  Implementation::RobotStatePub::SharedPtr robot_state_pub(
      new Implementation::RobotStatePub(
          participant_ptr->id(),
          &FreeFleetData_RobotState_desc,
          config.robot_state_topic));
  Implementation::ModeRequestSub::SharedPtr mode_request_sub(
      new Implementation::ModeRequestSub(
          participant_ptr->id(),
          &FreeFleetData_ModeRequest_desc,
          config.mode_request_topic));
  Implementation::PathRequestSub::SharedPtr path_request_sub(
      new Implementation::PathRequestSub(
          participant_ptr->id(),
          &FreeFleetData_PathRequest_desc,
          config.path_request_topic));
  Implementation::DestinationRequestSub::SharedPtr destination_request_sub(
      new Implementation::DestinationRequestSub(
          participant_ptr->id(),
          &FreeFleetData_DestinationRequest_desc,
          config.destination_request_topic));

  if (!robot_state_pub->is_ready() ||
      !mode_request_sub->is_ready() ||
      !path_request_sub->is_ready() ||
      !destination_request_sub->is_ready())
    return nullptr;

  SharedPtr client_ptr = SharedPtr(new Client);
  client_ptr->_pimpl.reset(
      new Implementation(
          std::move(config),
          std::move(participant_ptr),
          std::move(robot_state_pub),
          std::move(mode_request_sub),
          std::move(path_request_sub),
          std::move(destination_request_sub)));
  return client_ptr;
}

//==============================================================================

bool Client::send_robot_state(const messages::RobotState& new_robot_state)
{
  FreeFleetData_RobotState* new_rs = FreeFleetData_RobotState__alloc();
  convert(new_robot_state, *new_rs);
  bool sent = _pimpl->_robot_state_pub->write(new_rs);
  FreeFleetData_RobotState_free(new_rs, DDS_FREE_ALL);
  return sent;
}

//==============================================================================

bool Client::read_mode_request(messages::ModeRequest& mode_request)
{
  auto mode_requests = _pimpl->_mode_request_sub->read();
  if (!mode_requests.empty())
  {
    convert(*(mode_requests[0]), mode_request);
    return true;
  }
  return false;
}

//==============================================================================

bool Client::read_path_request(messages::PathRequest& path_request)
{
  auto path_requests = _pimpl->_path_request_sub->read();
  if (!path_requests.empty())
  {
    convert(*(path_requests[0]), path_request);
    return true;
  }
  return false;
}

//==============================================================================

bool Client::read_destination_request(
    messages::DestinationRequest& destination_request)
{
  auto destination_requests = _pimpl->_destination_request_sub->read();
  if (!destination_requests.empty())
  {
    convert(*(destination_requests[0]), destination_request);
    return true;
  }
  return false;
}

//==============================================================================

Client::~Client()
{}

//==============================================================================

Client::Client()
{}

//==============================================================================

} // namespace free_fleet
