/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__CLIENT__INTERNAL_CLIENT_HPP
#define SRC__CLIENT__INTERNAL_CLIENT_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <optional>
#include <unordered_set>

#include <rmf_utils/Modular.hpp>

#include <free_fleet/Types.hpp>
#include <free_fleet/client/Client.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {

//==============================================================================
class Client::Implementation
{
public:
  struct DataHandle : std::enable_shared_from_this<DataHandle>
  {
    std::string robot_name;

    std::string robot_model;

    std::shared_ptr<client::CommandHandle> command_handle;

    std::optional<CommandId> command_id = std::nullopt;

    CommandId last_command_id = 0;

    std::unordered_set<uint32_t> command_ids;

    void complete_command();

    void handle_pause_request(const messages::PauseRequest& request);

    void handle_resume_request(const messages::ResumeRequest& request);

    void handle_dock_request(const messages::DockRequest& request);

    void handle_navigation_request(const messages::NavigationRequest& request);

    void handle_relocalization_request(
      const messages::RelocalizationRequest& request);

    template<class T>
    bool is_valid_request(const T& request)
    {
      if (request.robot_name() != robot_name)
        return false;

      // TODO(AA): Consider if there is the chance that this client idles for
      // more than 2 billion command ids, and suddenly gets a clashing id that
      // has overflowed and is slightly lower than the last command id.
      const auto it = command_ids.find(request.command_id());
      if (it == command_ids.end() ||
        rmf_utils::Modular(last_command_id).less_than(request.command_id()))
      {
        return true;
      }

      return false;
    }
  };

  Implementation()
  {}

  static Implementation& get(Client& client)
  {
    return *client._pimpl;
  }

  static const Implementation& get(const Client& client)
  {
    return *client._pimpl;
  }

  void set_callbacks();

  std::shared_ptr<client::StatusHandle> status_handle;
  std::shared_ptr<transport::ClientMiddleware> middleware;
  std::shared_ptr<DataHandle> data;
};

} // namespace free_fleet

#endif // SRC__CLIENT__INTERNAL_CLIENT_HPP
