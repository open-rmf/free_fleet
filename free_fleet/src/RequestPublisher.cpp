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

#include <free_fleet/Participant.hpp>
#include <free_fleet/RequestPublisher.hpp>

#include "messages/FleetMessages.h"
#include "messages/message_utils.hpp"

#include "dds_utils/DDSPublishHandler.hpp"

namespace free_fleet {

//==============================================================================

class RequestPublisher::Implementation
{
public:
  using ModeRequestPub = dds::DDSPublishHandler<FreeFleetData_ModeRequest>;
  using PathRequestPub = dds::DDSPublishHandler<FreeFleetData_PathRequest>;
  using DestinationRequestPub = 
      dds::DDSPublishHandler<FreeFleetData_DestinationRequest>;

  Config _config;
  Participant::SharedPtr _participant;
  ModeRequestPub::SharedPtr _mode_request_pub;
  PathRequestPub::SharedPtr _path_request_pub;
  DestinationRequestPub::SharedPtr _destination_request_pub;

  Implementation(
      Config config, Participant::SharedPtr participant,
      ModeRequestPub::SharedPtr mode_request_pub,
      PathRequestPub::SharedPtr path_request_pub,
      DestinationRequestPub::SharedPtr destination_request_pub)
  : _config(std::move(config)),
    _participant(std::move(participant)),
    _mode_request_pub(std::move(mode_request_pub)),
    _path_request_pub(std::move(path_request_pub)),
    _destination_request_pub(std::move(destination_request_pub))
  {}

  ~Implementation()
  {}
};

//==============================================================================

void RequestPublisher::Config::print_config() const
{
  printf("REQUEST PUBLISHER CONFIGURATION\n");
  printf("  domain ID: %d\n", domain_id);
  printf("  TOPICS\n");
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
}

//==============================================================================

RequestPublisher::SharedPtr RequestPublisher::make(Config config)
{
  auto participant_ptr = Participant::make(config.domain_id);
  if (!participant_ptr)
    return nullptr;

  dds::DDSPublishHandler<FreeFleetData_ModeRequest>::SharedPtr 
      mode_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_ModeRequest>(
              participant_ptr->id(),
              &FreeFleetData_ModeRequest_desc,
              config.mode_request_topic));

  dds::DDSPublishHandler<FreeFleetData_PathRequest>::SharedPtr 
      path_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_PathRequest>(
              participant_ptr->id(),
              &FreeFleetData_PathRequest_desc,
              config.path_request_topic));

  dds::DDSPublishHandler<FreeFleetData_DestinationRequest>::SharedPtr 
      destination_request_pub(
          new dds::DDSPublishHandler<FreeFleetData_DestinationRequest>(
              participant_ptr->id(),
              &FreeFleetData_DestinationRequest_desc,
              config.destination_request_topic));

  if (!mode_request_pub->is_ready() ||
      !path_request_pub->is_ready() ||
      !destination_request_pub->is_ready())
    return nullptr;

  SharedPtr request_publisher_ptr = SharedPtr(new RequestPublisher);
  request_publisher_ptr->_pimpl.reset(
      new Implementation(
          std::move(config),
          participant_ptr,
          mode_request_pub,
          path_request_pub,
          destination_request_pub));
  return request_publisher_ptr;
}

//==============================================================================

bool RequestPublisher::send_mode_request(
    const messages::ModeRequest& mode_request) const
{
  FreeFleetData_ModeRequest* new_mr = FreeFleetData_ModeRequest__alloc();
  convert(mode_request, *new_mr);
  bool sent = _pimpl->_mode_request_pub->write(new_mr);
  FreeFleetData_ModeRequest_free(new_mr, DDS_FREE_ALL);
  return sent;
}

//==============================================================================

bool RequestPublisher::send_path_request(
    const messages::PathRequest& path_request) const
{
  FreeFleetData_PathRequest* new_pr = FreeFleetData_PathRequest__alloc();
  convert(path_request, *new_pr);
  bool sent = _pimpl->_path_request_pub->write(new_pr);
  FreeFleetData_PathRequest_free(new_pr, DDS_FREE_ALL);
  return sent;
}

//==============================================================================

bool RequestPublisher::send_destination_request(
    const messages::DestinationRequest& destination_request) const
{
  FreeFleetData_DestinationRequest* new_dr = 
      FreeFleetData_DestinationRequest__alloc();
  convert(destination_request, *new_dr);
  bool sent = _pimpl->_destination_request_pub->write(new_dr);
  FreeFleetData_DestinationRequest_free(new_dr, DDS_FREE_ALL);
  return sent;
}

//==============================================================================

RequestPublisher::~RequestPublisher()
{}

//==============================================================================

RequestPublisher::RequestPublisher()
{}

//==============================================================================

} // namespace free_fleet
