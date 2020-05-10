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

#ifndef FREE_FLEET__SRC__CLIENTIMPL_HPP
#define FREE_FLEET__SRC__CLIENTIMPL_HPP

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>
#include <free_fleet/Client.hpp>
#include <free_fleet/ClientConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

class Client::ClientImpl
{
public:

  /// DDS related fields required for the client to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS publisher that handles sending out current robot states to the 
    /// server
    dds::DDSPublishHandler<FreeFleetData_RobotState>::SharedPtr
        state_pub;

    /// DDS subscriber for mode requests coming from the server
    dds::DDSSubscribeHandler<FreeFleetData_ModeRequest>::SharedPtr 
        mode_request_sub;

    /// DDS subscriber for path requests coming from the server
    dds::DDSSubscribeHandler<FreeFleetData_PathRequest>::SharedPtr 
        path_request_sub;

    /// DDS subscriber for destination requests coming from the server
    dds::DDSSubscribeHandler<FreeFleetData_DestinationRequest>::SharedPtr
        destination_request_sub;
  };

  ClientImpl(const ClientConfig& config);

  ~ClientImpl();

  void start(Fields fields);

  bool send_robot_state(const messages::RobotState& new_robot_state);

  bool read_mode_request(messages::ModeRequest& mode_request);

  bool read_path_request(messages::PathRequest& path_request);

  bool read_destination_request(
      messages::DestinationRequest& destination_request);

private:

  Fields fields;

  ClientConfig client_config;

};

} // namespace free_fleet

#endif // FREE_FLEET__SRC__CLIENTIMPL_HPP
