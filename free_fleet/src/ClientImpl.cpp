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

#include "ClientImpl.hpp"
#include "messages/message_utils.hpp"

namespace free_fleet {

Client::ClientImpl::ClientImpl(const ClientConfig& _config) :
  client_config(_config)
{}

Client::ClientImpl::~ClientImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Client::ClientImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Client::ClientImpl::send_robot_state(
    const messages::RobotState& _new_robot_state)
{
  FreeFleetData_RobotState* new_rs = FreeFleetData_RobotState__alloc();
  convert(_new_robot_state, *new_rs);
  bool sent = fields.state_pub->write(new_rs);
  FreeFleetData_RobotState_free(new_rs, DDS_FREE_ALL);
  return sent;
}

bool Client::ClientImpl::read_mode_request
    (messages::ModeRequest& _mode_request)
{
  auto mode_requests = fields.mode_request_sub->read();
  if (!mode_requests.empty())
  {
    convert(*(mode_requests[0]), _mode_request);
    return true;
  }
  return false;
}

bool Client::ClientImpl::read_path_request(
    messages::PathRequest& _path_request)
{
  auto path_requests = fields.path_request_sub->read();
  if (!path_requests.empty())
  {
    convert(*(path_requests[0]), _path_request);
    return true;
  }
  return false;
}

bool Client::ClientImpl::read_destination_request(
    messages::DestinationRequest& _destination_request)
{
  auto destination_requests = fields.destination_request_sub->read();
  if (!destination_requests.empty())
  {
    convert(*(destination_requests[0]), _destination_request);
    return true;
  }
  return false;
}

} // namespace free_fleet
