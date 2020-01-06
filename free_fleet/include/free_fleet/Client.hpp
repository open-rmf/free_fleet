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

#ifndef FREE_FLEET__SRC__CLIENT_HPP
#define FREE_FLEET__SRC__CLIENT_HPP

#include <mutex>
#include <atomic>
#include <memory>

#include <dds/dds.h>

#include <free_fleet/ClientConfig.hpp>

#include <free_fleet/dds_utils/DDSPublishHandler.hpp>
#include <free_fleet/dds_utils/DDSSubscribeHandler.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

/// Forward declaration of internal types
struct FreeFleetData_RobotState;
struct FreeFleetData_ModeRequest;
struct FreeFleetData_PathRequest;
struct FreeFleetData_DestinationRequest;

namespace free_fleet
{

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  /// Factory function that creates an instance of the Free Fleet DDS Client
  ///
  /// \param[in] config
  ///   Configuration that sets up the client to communicate with the server.
  static SharedPtr make(const ClientConfig &config);

  ///
  bool send_robot_state(const messages::RobotState& new_robot_state);

  ///
  bool read_mode_request(messages::ModeRequest& mode_request);

  ///
  bool read_path_request(messages::PathRequest& path_request);

  ///
  bool read_destination_request(
      messages::DestinationRequest& destination_request);

  /// Destructor
  ~Client();

  /// DDS related fields required for the client to operate
  struct Fields
  {
    ///
    dds_entity_t participant;

    ///
    dds::DDSPublishHandler<FreeFleetData_RobotState>::SharedPtr
        state_pub;

    ///
    dds::DDSSubscribeHandler<FreeFleetData_ModeRequest>::SharedPtr 
        mode_request_sub;

    ///
    dds::DDSSubscribeHandler<FreeFleetData_PathRequest>::SharedPtr 
        path_request_sub;

    /// 
    dds::DDSSubscribeHandler<FreeFleetData_DestinationRequest>::SharedPtr
        destination_request_sub;
  };

private:

  ClientConfig client_config;

  Fields fields;

  Client(const ClientConfig& config);

  void start(Fields fields);

};

} // namespace free_fleet

#endif // FREE_FLEET__SRC__CLIENT_HPP
