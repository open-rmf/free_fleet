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

#include "agv/RobotInfo.hpp"
#include "requests/RequestInfo.hpp"
#include "requests/ModeRequestInfo.hpp"
#include "requests/NavigationRequestInfo.hpp"
#include "requests/RelocalizationRequestInfo.hpp"

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

  void _thread_fn()
  {
    const double seconds_per_iteration = 1.0 / _thread_frequency;
    const rmf_traffic::Duration duration_per_iteration =
      rmf_traffic::time::from_seconds(seconds_per_iteration);
    rmf_traffic::Time t_prev = _time_now_fn();

    while (_connected())
    {
      if (_time_now_fn() - t_prev < duration_per_iteration)
        continue;
      t_prev = _time_now_fn();

      // get states
      auto states = _middleware->read_states();
      std::lock_guard<std::mutex> lock(_mutex);
      for (const auto s : states)
      {
        const auto r_it = _robots.find(s.name);
        bool new_robot = r_it == _robots.end();
        if (new_robot)
        {
          _robots[s.name] = std::make_shared<agv::RobotInfo>(
            agv::RobotInfo{
              s,
              _time_now_fn()
            });
          std::cout << "Registered new robot: [" << s.name << "]..."
            << std::endl;
        }
        else
        {
          r_it->second->update_state(s, _time_now_fn());
        }

        // Updates external uses of the robot's information
        if (_new_robot_state_callback_fn)
          _new_robot_state_callback_fn(s);

        // for each robot figure out whether any tasks were not received yet
        const auto t_it = _unacknowledged_tasks.find(s.task_id);
        if (t_it != _unacknowledged_tasks.end())
        {
          t_it->second->acknowledged_time(_time_now_fn());
          _unacknowledged_tasks.erase(t_it);
        }
      }
      
      // Send out all unreceived tasks again
      for (const auto t_it : _unacknowledged_tasks)
      {
        t_it.second->send_request();
      }
    }
  }

  std::string _fleet_name;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<transport::Middleware> _middleware;
  TimeNow _time_now_fn;
  NewRobotStateCallback _new_robot_state_callback_fn;

  std::unordered_map<std::string, agv::RobotInfo::SharedPtr> _robots;

  /// TODO(AA): Keep track of how long the tasks have been sitting here, fail
  /// commands gracefully after a certain timeout.
  /// TODO(AA): Cull things that happen long after
  /// _tasks hold everything, check this for task ID existence
  /// _unreceived_tasks only hold onto tasks that have not been acknowledged
  /// _timed_out_tasks only hold onto tasks that have not been acknowledged for
  /// more than the timeout period.
  // std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
  //   _tasks;
  // std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
  //   _unreceived_tasks;
  // std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
  //   _timed_out_tasks;
  // std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
  //   _acknow
  // std::unordered_set<std::string> _task_ids;

  ///
  int _current_task_id = -1;
  std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
    _tasks;
  std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
    _robot_tasks;
  std::unordered_map<std::string, std::shared_ptr<requests::RequestInfo>>
    _unacknowledged_tasks;


  std::mutex _mutex;
  std::thread _thread;
  uint32_t _thread_frequency;
};

//==============================================================================
Manager::SharedPtr Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<rmf_traffic::agv::Graph> graph,
  std::shared_ptr<transport::Middleware> middleware,
  TimeNow time_now_fn,
  NewRobotStateCallback new_robot_state_callback_fn)
{
  SharedPtr manager_ptr(new Manager);
  manager_ptr->_pimpl->_fleet_name = fleet_name;
  manager_ptr->_pimpl->_graph = std::move(graph);
  manager_ptr->_pimpl->_middleware = std::move(middleware);
  manager_ptr->_pimpl->_time_now_fn = std::move(time_now_fn);
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
  return it->second->state();
}

//==============================================================================
std::vector<messages::RobotState> Manager::robot_states()
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  std::vector<messages::RobotState> states;
  states.reserve(_pimpl->_robots.size());
  for (const auto it : _pimpl->_robots)
  {
    states.push_back(it.second->state());
  }
  return states;
}

//==============================================================================
rmf_utils::optional<std::string> Manager::send_mode_request(
  const std::string& robot_name,
  const messages::RobotMode& mode,
  std::vector<messages::ModeParameter> parameters)
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (_pimpl->_robots.find(robot_name) == _pimpl->_robots.end())
    return rmf_utils::nullopt;

  messages::ModeRequest request{
    robot_name,
    std::to_string(++_pimpl->_current_task_id),
    mode,
    parameters};
  auto mode_request_info = std::make_shared<requests::ModeRequestInfo>(
    requests::ModeRequestInfo(
      request,
      [this](const messages::ModeRequest& request_msg)
    {
      this->_pimpl->_middleware->send_mode_request(request_msg);
    },
      _pimpl->_time_now_fn()));
  
  auto request_info =
    std::dynamic_pointer_cast<requests::RequestInfo>(mode_request_info);
  _pimpl->_tasks[request.task_id] = request_info;
  _pimpl->_unacknowledged_tasks[request.task_id] = request_info;

  _pimpl->_middleware->send_mode_request(request);
  return std::to_string(_pimpl->_current_task_id);
}

//==============================================================================
rmf_utils::optional<std::string> Manager::send_navigation_request(
  const std::string& robot_name,
  const std::vector<messages::Waypoint>& path)
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (path.empty() || 
    _pimpl->_robots.find(robot_name) == _pimpl->_robots.end())
    return rmf_utils::nullopt;

  const std::size_t num_wp = _pimpl->_graph->num_waypoints();
  for (const auto wp : path)
  {
    std::size_t wp_index = static_cast<std::size_t>(wp.index);
    if (wp_index >= num_wp)
      return rmf_utils::nullopt;
    
    const auto g_wp = _pimpl->_graph->get_waypoint(wp_index);
    const Eigen::Vector2d provided_loc {wp.location.x, wp.location.y};
    if (g_wp.get_map_name() != wp.location.level_name ||
      (provided_loc - g_wp.get_location()).norm() > 1e-3)
      return rmf_utils::nullopt;
  }

  messages::NavigationRequest request{
    robot_name,
    std::to_string(++_pimpl->_current_task_id),
    path};
  auto navigation_request_info =
    std::make_shared<requests::NavigationRequestInfo>(
      request,
      [this](const messages::NavigationRequest& request_msg)
  {
    this->_pimpl->_middleware->send_navigation_request(request_msg);
  },
      _pimpl->_time_now_fn());

  auto request_info =
    std::dynamic_pointer_cast<requests::RequestInfo>(navigation_request_info);
  _pimpl->_tasks[request.task_id] = request_info;
  _pimpl->_unacknowledged_tasks[request.task_id] = request_info;

  _pimpl->_middleware->send_navigation_request(request);
  return std::to_string(_pimpl->_current_task_id);
}

//==============================================================================
rmf_utils::optional<std::string> Manager::send_relocalization_request(
  const std::string& robot_name,
  const messages::Location& location,
  uint32_t last_visited_index)
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (_pimpl->_robots.find(robot_name) == _pimpl->_robots.end() ||
    last_visited_index >= _pimpl->_graph->num_waypoints())
    return rmf_utils::nullopt;

  auto wp =
    _pimpl->_graph->get_waypoint(static_cast<std::size_t>(last_visited_index));
  const double dist_from_wp =
    (Eigen::Vector2d{location.x, location.y} - wp.get_location()).norm();
  if (dist_from_wp >= 10.0)
  {
    return rmf_utils::nullopt;
  }

  messages::RelocalizationRequest request{
    robot_name,
    std::to_string(++_pimpl->_current_task_id),
    location,
    last_visited_index};
  auto relocalization_request_info =
    std::make_shared<requests::RelocalizationRequestInfo>(
      request,
      [this](const messages::RelocalizationRequest& request_msg)
  {
    this->_pimpl->_middleware->send_relocalization_request(request_msg);
  },
      _pimpl->_time_now_fn());

  auto request_info =
    std::dynamic_pointer_cast<requests::RequestInfo>(
      relocalization_request_info);
  _pimpl->_tasks[request.task_id] = request_info;
  _pimpl->_unacknowledged_tasks[request.task_id] = request_info;

  _pimpl->_middleware->send_relocalization_request(request);
  return std::to_string(_pimpl->_current_task_id);
}

//==============================================================================
} // namespace free_fleet
