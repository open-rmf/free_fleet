/*
 * Copyright (C) 2002 Open Source Robotics Foundation
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

#include <free_fleet/agv/Client.hpp>

namespace free_fleet {
namespace agv {

//==============================================================================

class Client::Implementation
{
public:

  Implementation()
  {}

  ~Implementation() = default;

  std::shared_ptr<CommandHandle> _command_handle;
  std::shared_ptr<StatusHandle> _status_handle;
  std::shared_ptr<transport::Middleware> _middleware;

  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
};

//==============================================================================

Client::SharedPtr Client::make(
    std::shared_ptr<CommandHandle> command_handle,
    std::shared_ptr<StatusHandle> status_handle,
    std::shared_ptr<transport::Middleware> middleware,
    std::shared_ptr<rmf_traffic::agv::Graph> graph)
{
  if (!command_handle || !status_handle || !middleware || !graph)
    return nullptr;

  Client::SharedPtr new_client(new Client);
  new_client->_pimpl = rmf_utils::make_impl<Implementation>(Implementation());
  new_client->_pimpl->_command_handle = std::move(command_handle);
  new_client->_pimpl->_status_handle = std::move(status_handle);
  new_client->_pimpl->_middleware = std::move(middleware);
  new_client->_pimpl->_graph = std::move(graph);

  return new_client;
}

//==============================================================================

void Client::start()
{
}

//==============================================================================

} // namespace agv
} // namespace free_fleet
