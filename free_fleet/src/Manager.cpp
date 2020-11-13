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

#include <mutex>
#include <thread>
#include <unordered_map>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/Manager.hpp>

namespace free_fleet {

//==============================================================================
class Manager::Implementation
{
public:

  Implementation()
  {}

  Implementation(const Implementation& other)
  {}

  ~Implementation()
  {
    if (_thread.joinable())
      _thread.join();
  }

  bool _connected() const
  {
    return _middleware && _graph;
  }

  std::string _fleet_name;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<transport::Middleware> _middleware;

  std::unordered_map<std::string, messages::RobotState> _robots;

  std::mutex _mutex;
  std::thread _thread;
};

//==============================================================================
Manager::SharedPtr Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<rmf_traffic::agv::Graph> graph,
  std::shared_ptr<transport::Middleware> middleware)
{
  SharedPtr manager_ptr(new Manager);
  manager_ptr->_pimpl->_fleet_name = fleet_name;
  manager_ptr->_pimpl->_graph = std::move(graph);
  manager_ptr->_pimpl->_middleware = std::move(middleware);

  return manager_ptr;
}

//==============================================================================
Manager::Manager()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Manager::start(uint32_t frequency)
{
  const double seconds_per_iteration = 1.0 / frequency;
  const rmf_traffic::Duration duration_per_iteration =
    rmf_traffic::time::from_seconds(seconds_per_iteration);
  rmf_traffic::Time t_prev = std::chrono::steady_clock::now();

  while (_pimpl->_connected())
  {
    if (std::chrono::steady_clock::now() - t_prev < duration_per_iteration)
      continue;

    // get states
    auto states = _pimpl->_middleware->read_states();
    // for (auto )
  }
}

//==============================================================================
std::vector<std::string> Manager::robots()
{
  return {};
}

//==============================================================================
rmf_utils::optional<messages::RobotState> Manager::robot_state(
  const std::string& robot_name)
{
  return rmf_utils::nullopt;
}

//==============================================================================
std::vector<messages::RobotState> Manager::robot_states()
{
  return {};
}

//==============================================================================
void Manager::send_mode_request(const messages::ModeRequest& request)
{}

//==============================================================================
void Manager::send_navigation_request(
  const messages::NavigationRequest& request)
{}

//==============================================================================
void Manager::send_relocalization_request(
  const messages::RelocalizationRequest& request)
{}

//==============================================================================
} // namespace free_fleet
