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

#include "ServerImpl.hpp"
#include "messages/message_utils.hpp"

namespace free_fleet {

Server::ServerImpl::ServerImpl(const ServerConfig& _config) :
  server_config(_config)
{}

Server::ServerImpl::~ServerImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Server::ServerImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Server::ServerImpl::read_robot_states(
    std::vector<messages::RobotState>& _new_robot_states)
{
  auto robot_states = fields.robot_state_sub->read();
  if (!robot_states.empty())
  {
    _new_robot_states.clear();
    for (size_t i = 0; i < robot_states.size(); ++i)
    {
      messages::RobotState tmp_robot_state;
      convert(*(robot_states[i]), tmp_robot_state);
      _new_robot_states.push_back(tmp_robot_state);
    }
    return true;
  }
  return false;
}

bool Server::ServerImpl::send_mode_request(
    const messages::ModeRequest& _mode_request)
{
  FreeFleetData_ModeRequest* new_mr = FreeFleetData_ModeRequest__alloc();
  convert(_mode_request, *new_mr);
  bool sent = fields.mode_request_pub->write(new_mr);
  FreeFleetData_ModeRequest_free(new_mr, DDS_FREE_ALL);
  return sent;
}

bool Server::ServerImpl::send_path_request(
    const messages::PathRequest& _path_request)
{
  FreeFleetData_PathRequest* new_pr = FreeFleetData_PathRequest__alloc();
  convert(_path_request, *new_pr);
  bool sent = fields.path_request_pub->write(new_pr);
  FreeFleetData_PathRequest_free(new_pr, DDS_FREE_ALL);
  return sent;
}

bool Server::ServerImpl::send_destination_request(
    const messages::DestinationRequest& _destination_request)
{
  FreeFleetData_DestinationRequest* new_dr = 
      FreeFleetData_DestinationRequest__alloc();
  convert(_destination_request, *new_dr);
  bool sent = fields.destination_request_pub->write(new_dr);
  FreeFleetData_DestinationRequest_free(new_dr, DDS_FREE_ALL);
  return sent;
}

} // namespace free_fleet
