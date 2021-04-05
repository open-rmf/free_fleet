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

#ifndef INCLUDE__FREE_FLEET__AGV__CLIENT_HPP
#define INCLUDE__FREE_FLEET__AGV__CLIENT_HPP

#include <memory>

#include <rmf_utils/impl_ptr.hpp>

#include <free_fleet/agv/StatusHandle.hpp>
#include <free_fleet/agv/CommandHandle.hpp>
#include <free_fleet/transport/Middleware.hpp>

namespace free_fleet {
namespace agv {

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  /// Factory function that creates an instance of the Free Fleet Client.
  ///
  /// \param[in] robot_name
  ///   Name of the robot.
  ///
  /// \param[in] robot_model
  ///   Model of the robot.
  ///
  /// \param[in] command_handle
  ///   Command handle implementation specific to the robot that this client is
  ///   attached to, in order to complete the requests relayed from the fleet
  ///   manager.
  ///
  /// \param[in] status_handle
  ///   Status handle implementation specific to the robot that this client is
  ///   attached to, in order to construct the standardized robot state message
  ///   that is required for updating the fleet manager.
  ///
  /// \param[in] middleware
  ///   Middleware implementation to be used between the robot client and the
  ///   fleet manager.
  ///
  /// \return
  ///   Shared pointer to a client instance that is ready to be started. If
  ///   any of the initializations fail during during this function's
  ///   execution, a nullptr will be returned.
  static SharedPtr make(
    const std::string& robot_name,
    const std::string& robot_model,
    std::shared_ptr<CommandHandle> command_handle,
    std::shared_ptr<StatusHandle> status_handle,
    std::shared_ptr<transport::Middleware> middleware);

  /// Starts the client which begins to update the fleet manager with the
  /// robot's current status, as well as polls for requests before performing
  /// them.
  ///
  /// \param[in] frequency
  ///   Frequency at which the client attempts to check for incoming requests,
  ///   command the robot and update its state upstream.
  void start(uint32_t frequency);

  class Implementation;
private:
  Client();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__AGV__CLIENT_HPP
