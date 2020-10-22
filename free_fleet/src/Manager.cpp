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

#include <free_fleet/Manager.hpp>

namespace free_fleet {

//==============================================================================
class Manager::Implementation
{
public:

  Implementation()
  {}

  ~Implementation() = default;

  bool _connected() const
  {
    return _middleware && _graph;
  }

  std::string _fleet_name;

  std::shared_ptr<transport::Middleware> _middleware;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
};

//==============================================================================
Manager::SharedPtr Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<transport::Middleware> middleware,
  std::shared_ptr<rmf_traffic::agv::Graph> graph)
{
  if (!middleware && !graph)
    return nullptr;

  Manager::SharedPtr new_manager(new Manager);
  new_manager->_pimpl->_fleet_name = fleet_name;
  new_manager->_pimpl->_middleware = std::move(middleware);
  new_manager->_pimpl->_graph = std::move(graph);

  return new_manager;
}

//==============================================================================
Manager::Manager()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Manager::start(uint32_t frequency)
{

}

//==============================================================================

//==============================================================================
} // namepace free_fleet
