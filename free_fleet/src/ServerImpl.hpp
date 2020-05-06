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

#ifndef FREE_FLEET__SRC__SERVERIMPL_HPP
#define FREE_FLEET__SRC__SERVERIMPL_HPP

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>
#include <free_fleet/Server.hpp>
#include <free_fleet/ServerConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

class Server::ServerImpl
{
public:

  /// DDS related fields required for the server to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS subscribers for new incoming robot states from clients
    dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>::SharedPtr 
        robot_state_sub;

    /// DDS publisher for mode requests to be sent to clients
    dds::DDSPublishHandler<FreeFleetData_ModeRequest>::SharedPtr
        mode_request_pub;

    /// DDS publisher for path requests to be sent to clients
    dds::DDSPublishHandler<FreeFleetData_PathRequest>::SharedPtr
        path_request_pub;

    /// DDS publisher for destination requests to be sent to clients
    dds::DDSPublishHandler<FreeFleetData_DestinationRequest>::SharedPtr
        destination_request_pub;
  };

  ServerImpl(const ServerConfig& config);

  ~ServerImpl();

  void start(Fields fields);

  bool read_robot_states(std::vector<messages::RobotState>& new_robot_states);

  bool send_mode_request(const messages::ModeRequest& mode_request);

  bool send_path_request(const messages::PathRequest& path_request);

  bool send_destination_request(
      const messages::DestinationRequest& destination_request);

private:

  Fields fields;

  ServerConfig server_config;

};

} // namespace free_fleet

#endif // FREE_FLEET__SRC__SERVERIMPL_HPP
