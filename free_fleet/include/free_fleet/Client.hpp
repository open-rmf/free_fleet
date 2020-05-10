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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__CLIENT_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__CLIENT_HPP

#include <memory>

#include <free_fleet/ClientConfig.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

namespace free_fleet {

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  /// Factory function that creates an instance of the Free Fleet DDS Client.
  ///
  /// \param[in] config
  ///   Configuration that sets up the client to communicate with the server.
  /// \return
  ///   Shared pointer to a free fleet client.
  static SharedPtr make(const ClientConfig& config);

  /// Attempts to send a new robot state to the free fleet server, to be 
  /// registered by the fleet management system.
  ///
  /// \param[in] new_robot_state
  ///   Current robot state to be sent to the free fleet server to update the
  ///   fleet management system.
  /// \return
  ///   True if robot state was successfully sent, false otherwise.
  bool send_robot_state(const messages::RobotState& new_robot_state);

  /// Attempts to read and receive a new mode request from the free fleet
  /// server, for commanding the robot client.
  ///
  /// \param[out] mode_request
  ///   Newly received robot mode request from the free fleet server, to be
  ///   handled by the robot client.
  /// \return
  ///   True if a new mode request was received, false otherwise.
  bool read_mode_request(messages::ModeRequest& mode_request);

  /// Attempts to read and receive a new path request from the free fleet
  /// server, for commanding the robot client.
  ///
  /// \param[out] path_request
  ///   Newly received robot path request from the free fleet server, to be
  ///   handled by the robot client.
  /// \return
  ///   True if a new path request was received, false otherwise.
  bool read_path_request(messages::PathRequest& path_request);

  /// Attempts to read and receive a new destination request from the free
  /// fleet server, for commanding the robot client.
  /// 
  /// \param[out] destination_request
  ///   Newly received robot destination request from the free fleet server,
  ///   to be handled by the robot client.
  /// \return
  ///   True if a new destination request was received, false otherwise.
  bool read_destination_request(
      messages::DestinationRequest& destination_request);

  /// Destructor
  ~Client();

private:

  /// Forward declaration and unique implementation
  class ClientImpl;

  std::unique_ptr<ClientImpl> impl;

  Client(const ClientConfig& config);

};

} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__CLIENT_HPP
