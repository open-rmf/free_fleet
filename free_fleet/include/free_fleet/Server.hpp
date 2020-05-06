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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__SERVER_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__SERVER_HPP

#include <memory>
#include <vector>

#include <free_fleet/ServerConfig.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

namespace free_fleet {

class Server
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  /// Factory function that creates an instance of the Free Fleet Server.
  ///
  /// \param[in] config
  ///   Configuration that sets up the server to communicate with the clients.
  /// \return
  ///   Shared pointer to a free fleet server.
  static SharedPtr make(const ServerConfig& config);

  /// Attempts to read new incoming robot states sent by free fleet clients
  /// over DDS.
  ///
  /// \param[out] new_robot_states
  ///   A vector of new incoming robot states sent by clients to update the
  ///   fleet management system.
  /// \return
  ///   True if new robot states were received, false otherwise.
  bool read_robot_states(std::vector<messages::RobotState>& new_robot_states);

  /// Attempts to send a new mode request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  /// 
  /// \param[in] mode_request
  ///   New mode request to be sent out to the clients.
  /// \return
  ///   True if the mode request was successfully sent, false otherwise.
  bool send_mode_request(const messages::ModeRequest& mode_request);

  /// Attempts to send a new path request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  ///
  /// \param[in] path_request
  ///   New path request to be sent out to the clients.
  /// \return
  ///   True if the path request was successfully sent, false otherwise.
  bool send_path_request(const messages::PathRequest& path_request);

  /// Attempts to send a new destination request to all the clients. Clients 
  /// are in charge to identify if requests are targetted towards them.
  ///
  /// \param[in] destination_request
  ///   New destination request to be sent out to the clients.
  /// \return
  ///   True if the destination request was successfully sent, false otherwise.
  bool send_destination_request(
      const messages::DestinationRequest& destination_request);

  /// Destructor
  ~Server();

private:

  /// Forward declaration and unique implementation
  class ServerImpl;

  std::unique_ptr<ServerImpl> impl;

  Server(const ServerConfig& config);

};

} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__SERVER_HPP
