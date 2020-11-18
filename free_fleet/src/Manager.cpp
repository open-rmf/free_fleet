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
#include <iostream>
#include <functional>
#include <unordered_map>
#include <unordered_set>

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

  template<class T>
  bool _is_request_valid(const T& request)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    const auto r_it = _robots.find(request.robot_name);
    const auto t_it = _task_ids.find(request.task_id);
    if (r_it == _robots.end() || t_it != _task_ids.end())
      return false;
    return true;
  }

  void _thread_fn()
  {
    const double seconds_per_iteration = 1.0 / _thread_frequency;
    const rmf_traffic::Duration duration_per_iteration =
      rmf_traffic::time::from_seconds(seconds_per_iteration);
    rmf_traffic::Time t_prev = std::chrono::steady_clock::now();

    while (_connected())
    {
      if (std::chrono::steady_clock::now() - t_prev < duration_per_iteration)
        continue;

      // get states
      auto states = _middleware->read_states();
      std::lock_guard<std::mutex> lock(_mutex);
      for (const auto s : states)
      {
        bool new_robot = _robots.insert({s.name, s}).second;
        if (new_robot)
        {
          std::cout << "Registered new robot: [" << s.name << "]..." << std::endl;
        }
        else
        {
          _robots[s.name] = s;
        }

        // Updates external uses of the robot's information
        if (_new_robot_state_callback_fn)
          _new_robot_state_callback_fn(s);

        // for each robot figure out whether any tasks were not received yet
        const auto it = _unreceived_task_ids.find(s.task_id);
        if (it != _unreceived_task_ids.end())
          _unreceived_task_ids.erase(it);
      }
      
      // Send out all unreceived tasks again
      for (const auto it : _unreceived_task_ids)
      {
        it.second();
      }
    }
  }

  std::string _fleet_name;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<transport::Middleware> _middleware;
  NewRobotStateCallback _new_robot_state_callback_fn;

  std::unordered_map<std::string, messages::RobotState> _robots;

  using SendRequest = std::function<void()>;

  /// TODO(AA): Cull things that happen long after
  std::unordered_map<std::string, SendRequest> _unreceived_task_ids;
  std::unordered_set<std::string> _task_ids;

  std::mutex _mutex;
  std::thread _thread;
  uint32_t _thread_frequency;
};

//==============================================================================
Manager::SharedPtr Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<rmf_traffic::agv::Graph> graph,
  std::shared_ptr<transport::Middleware> middleware,
  NewRobotStateCallback new_robot_state_callback_fn)
{
  SharedPtr manager_ptr(new Manager);
  manager_ptr->_pimpl->_fleet_name = fleet_name;
  manager_ptr->_pimpl->_graph = std::move(graph);
  manager_ptr->_pimpl->_middleware = std::move(middleware);
  manager_ptr->_pimpl->_new_robot_state_callback_fn =
    std::move(new_robot_state_callback_fn);

  return manager_ptr;
}

//==============================================================================
Manager::Manager()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Manager::start(uint32_t frequency)
{
  std::cout << "Starting manager thread..." << std::endl;
  _pimpl->_thread_frequency = frequency;
  _pimpl->_thread =
    std::thread(std::bind(&Implementation::_thread_fn, this->_pimpl));
}

//==============================================================================
std::vector<std::string> Manager::robots()
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  std::vector<std::string> result;
  result.reserve(_pimpl->_robots.size());
  for (const auto it : _pimpl->_robots)
  {
    result.push_back(it.first);
  }
  return result;
}

//==============================================================================
rmf_utils::optional<messages::RobotState> Manager::robot_state(
  const std::string& robot_name)
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  const auto it = _pimpl->_robots.find(robot_name);
  if (it == _pimpl->_robots.end())
    return rmf_utils::nullopt;
  return it->second;
}

//==============================================================================
std::vector<messages::RobotState> Manager::robot_states()
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  std::vector<messages::RobotState> states;
  states.reserve(_pimpl->_robots.size());
  for (const auto it : _pimpl->_robots)
  {
    states.push_back(it.second);
  }
  return states;
}

//==============================================================================
void Manager::send_mode_request(const messages::ModeRequest& request)
{
  if (!_pimpl->_is_request_valid(request))
    return;

  _pimpl->_middleware->send_mode_request(request);

  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  _pimpl->_unreceived_task_ids[request.task_id] =
    [this, request]()
  {
    this->_pimpl->_middleware->send_mode_request(request);
  };
}

//==============================================================================
void Manager::send_navigation_request(
  const messages::NavigationRequest& request)
{
  if (!_pimpl->_is_request_valid(request))
    return;

  _pimpl->_middleware->send_navigation_request(request);

  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  _pimpl->_unreceived_task_ids[request.task_id] =
    [this, request]()
  {
    this->_pimpl->_middleware->send_navigation_request(request);
  };
}

//==============================================================================
void Manager::send_relocalization_request(
  const messages::RelocalizationRequest& request)
{
  if (!_pimpl->_is_request_valid(request))
    return;

  _pimpl->_middleware->send_relocalization_request(request);

  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  _pimpl->_unreceived_task_ids[request.task_id] =
    [this, request]
  {
    this->_pimpl->_middleware->send_relocalization_request(request);
  };
}

//==============================================================================
} // namespace free_fleet
