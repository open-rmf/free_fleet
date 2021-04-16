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

#include <iostream>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/Manager.hpp>
#include <free_fleet/agv/RobotInfo.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include "internal_Manager.hpp"
#include "requests/RequestInfo.hpp"
#include "requests/SimpleRequestInfo.hpp"
#include "agv/internal_RobotInfo.hpp"

namespace free_fleet {

//==============================================================================
bool Manager::Implementation::connected() const
{
  return middleware && graph && to_robot_transform && time_now_fn;
}

//==============================================================================
void Manager::Implementation::run_once()
{
  // get states
  auto states = middleware->read_states();
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto s : states)
  {
    messages::RobotState transformed_state{
      s.name,
      s.model,
      s.task_id,
      s.mode,
      s.battery_percent,
      to_robot_transform->backward_transform(s.location),
      s.path_target_index};

    if (s.name.empty())
    {
      std::cout << "Somehow getting empty string!" << std::endl;
      continue;
    }

    const auto r_it = robots.find(s.name);
    bool new_robot = r_it == robots.end();
    if (new_robot)
    {
      auto new_robot = agv::RobotInfo::Implementation::make(
        transformed_state,
        graph,
        time_now_fn());
      if (new_robot)
      {
        robots[s.name] = std::move(new_robot);
        std::cout << "Registered new robot: [" << s.name << "]..."
          << std::endl;
      }
    }
    else
    {
      agv::RobotInfo::Implementation::get(*r_it->second).update_state(
        transformed_state, time_now_fn());
    }

    // Updates external uses of the robot's information
    if (r_it != robots.end() && robot_updated_callback_fn)
      robot_updated_callback_fn(*r_it->second);

    // for each robot figure out whether any tasks were not received yet
    const auto t_it = unacknowledged_tasks.find(s.task_id);
    if (t_it != unacknowledged_tasks.end())
    {
      t_it->second->acknowledged_time(time_now_fn());
      unacknowledged_tasks.erase(t_it);
    }
  }
  
  // Send out all unreceived tasks again
  for (const auto t_it : unacknowledged_tasks)
  {
    t_it.second->send_request();
  }
}

//==============================================================================
void Manager::Implementation::run(uint32_t frequency)
{
  const double seconds_per_iteration = 1.0 / frequency;
  const rmf_traffic::Duration duration_per_iteration =
    rmf_traffic::time::from_seconds(seconds_per_iteration);
  rmf_traffic::Time t_prev = time_now_fn();

  while (connected())
  {
    if (time_now_fn() - t_prev < duration_per_iteration)
      continue;
    t_prev = time_now_fn();

    run_once();
  }
}

//==============================================================================
void Manager::Implementation::start_async(uint32_t frequency)
{
  std::cout << "Starting manager thread..." << std::endl;
  async_thread =
    std::thread(
      std::bind(&free_fleet::Manager::Implementation::run, this, frequency));
}

//==============================================================================
Manager::SharedPtr Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<const rmf_traffic::agv::Graph> graph,
  std::unique_ptr<transport::ServerMiddleware> middleware,
  std::shared_ptr<const CoordinateTransformer> to_robot_transform,
  TimeNow time_now_fn,
  RobotUpdatedCallback robot_updated_callback_fn)
{
  auto make_error_fn = [](const std::string& error_msg)
  {
    std::cerr << "[Error]: " << error_msg << std::endl;
    return nullptr;
  };

  if (fleet_name.empty())
    return make_error_fn("Provided fleet name must not be empty.");
  if (!graph)
    return make_error_fn("Provided traffic graph is invalid.");
  if (!middleware)
    return make_error_fn("Provided free fleet middleware is invalid.");
  if (!to_robot_transform)
    return make_error_fn(
      "Provided free fleet CoordinateTransformer is invalid.");
  if (!time_now_fn)
    return make_error_fn("Provided time function is invalid.");

  SharedPtr manager_ptr(new Manager);
  manager_ptr->_pimpl->fleet_name = fleet_name;
  manager_ptr->_pimpl->graph = std::move(graph);
  manager_ptr->_pimpl->middleware = std::move(middleware);
  manager_ptr->_pimpl->to_robot_transform = std::move(to_robot_transform);
  manager_ptr->_pimpl->time_now_fn = std::move(time_now_fn);
  manager_ptr->_pimpl->robot_updated_callback_fn =
    std::move(robot_updated_callback_fn);
  return manager_ptr;
}

//==============================================================================
Manager::Manager()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Manager::run(uint32_t frequency)
{
  if (frequency == 0)
    throw std::range_error("[Error]: Frequency has to be greater than 0.");
  if (started())
    throw std::runtime_error("[Error]: Manager has already been started.");
  _pimpl->started = true;

  _pimpl->run(frequency);
}

//==============================================================================
void Manager::start_async(uint32_t frequency)
{
  if (frequency == 0)
    throw std::range_error("[Error]: Frequency has to be greater than 0.");
  if (started())
    throw std::runtime_error("[Error]: Manager has already been started.");

  _pimpl->start_async(frequency);
}

//==============================================================================
bool Manager::started() const
{
  return _pimpl->started.load();
}

//==============================================================================
auto Manager::robot_names() -> std::vector<std::string>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  std::vector<std::string> result;
  result.reserve(_pimpl->robots.size());
  for (const auto it : _pimpl->robots)
  {
    result.push_back(it.first);
  }
  return result;
}

//==============================================================================
auto Manager::robot(const std::string& robot_name) 
  -> std::shared_ptr<const agv::RobotInfo>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  const auto it = _pimpl->robots.find(robot_name);
  if (it == _pimpl->robots.end())
    return nullptr;
  return it->second;
}

//==============================================================================
auto Manager::all_robots() -> std::vector<std::shared_ptr<const agv::RobotInfo>>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  std::vector<std::shared_ptr<const agv::RobotInfo>> infos;
  infos.reserve(_pimpl->robots.size());
  for (const auto it : _pimpl->robots)
  {
    infos.push_back(it.second);
  }
  return infos;
}

//==============================================================================
auto Manager::request_pause(const std::string& robot_name)
  -> rmf_utils::optional<std::size_t>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
  {
    std::cerr << "[Error]: No such robot [" << robot_name << "]." << std::endl;
    return rmf_utils::nullopt;
  }

  // Handles the carry forward
  if ((++_pimpl->current_task_id) == _pimpl->idle_task_id)
    ++_pimpl->current_task_id;

  messages::PauseRequest request {
    robot_name,
    _pimpl->current_task_id
  };

  std::shared_ptr<requests::RequestInfo> request_info(
    new free_fleet::requests::SimpleRequestInfo<messages::PauseRequest>(
      request,
      [this](const messages::PauseRequest& request_msg)
    {
      this->_pimpl->middleware->send_pause_request(request_msg);
    },
      _pimpl->time_now_fn()));
  
  _pimpl->tasks[request.task_id] = request_info;
  _pimpl->unacknowledged_tasks[request.task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_resume(const std::string& robot_name)
  -> rmf_utils::optional<std::size_t>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
  {
    std::cerr << "[Error]: No such robot [" << robot_name << "]." << std::endl;
    return rmf_utils::nullopt;
  }

  // Handles the carry forward
  if ((++_pimpl->current_task_id) == _pimpl->idle_task_id)
    ++_pimpl->current_task_id;

  messages::ResumeRequest request {
    robot_name,
    _pimpl->current_task_id
  };

  std::shared_ptr<requests::RequestInfo> request_info(
    new free_fleet::requests::SimpleRequestInfo<messages::ResumeRequest>(
      request,
      [this](const messages::ResumeRequest& request_msg)
    {
      this->_pimpl->middleware->send_resume_request(request_msg);
    },
      _pimpl->time_now_fn()));
  
  _pimpl->tasks[request.task_id] = request_info;
  _pimpl->unacknowledged_tasks[request.task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_dock(
  const std::string& robot_name, const std::string& dock_name)
  -> rmf_utils::optional<std::size_t>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
  {
    std::cerr << "[Error]: No such robot [" << robot_name << "]." << std::endl;
    return rmf_utils::nullopt;
  }

  // Handles the carry forward
  if ((++_pimpl->current_task_id) == _pimpl->idle_task_id)
    ++_pimpl->current_task_id;

  messages::DockRequest request {
    robot_name,
    _pimpl->current_task_id,
    dock_name
  };

  std::shared_ptr<requests::RequestInfo> request_info(
    new free_fleet::requests::SimpleRequestInfo<messages::DockRequest>(
      request,
      [this](const messages::DockRequest& request_msg)
    {
      this->_pimpl->middleware->send_dock_request(request_msg);
    },
      _pimpl->time_now_fn()));
  
  _pimpl->tasks[request.task_id] = request_info;
  _pimpl->unacknowledged_tasks[request.task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_relocalization(
  const std::string& robot_name,
  const messages::Location& location,
  std::size_t last_visited_waypoint_index)
  -> rmf_utils::optional<std::size_t>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
  {
    std::cerr << "[Error]: No such robot [" << robot_name << "]." << std::endl;
    return rmf_utils::nullopt;
  }

  // Check if the waypoint exists
  const std::size_t num_wp = _pimpl->graph->num_waypoints();
  if (last_visited_waypoint_index >= num_wp)
  {
    std::cerr << "[Error]: Waypoint ["
      << std::to_string(last_visited_waypoint_index)
      << "] on the path does not exist on the graph." << std::endl;
    return rmf_utils::nullopt;
  }

  auto wp =
    _pimpl->graph->get_waypoint(last_visited_waypoint_index);
  const double dist_from_wp =
    (Eigen::Vector2d{location.x, location.y} - wp.get_location()).norm();
  if (dist_from_wp >= 10.0)
  {
    std::cerr << "[Error]: Last visited waypoint ["
      << std::to_string(last_visited_waypoint_index)
      << "] is too far away." << std::endl;
    return rmf_utils::nullopt;
  }

  messages::Location transformed_location =
    _pimpl->to_robot_transform->forward_transform(location);

  // Handles the carry forward
  if ((++_pimpl->current_task_id) == _pimpl->idle_task_id)
    ++_pimpl->current_task_id;

  messages::RelocalizationRequest request {
    robot_name,
    _pimpl->current_task_id,
    transformed_location,
    last_visited_waypoint_index
  };

  std::shared_ptr<requests::RequestInfo> request_info(
    new free_fleet::requests::SimpleRequestInfo<
      messages::RelocalizationRequest>(
        request,
        [this](const messages::RelocalizationRequest& request_msg)
    {
      this->_pimpl->middleware->send_relocalization_request(request_msg);
    },
      _pimpl->time_now_fn()));
  
  _pimpl->tasks[request.task_id] = request_info;
  _pimpl->unacknowledged_tasks[request.task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_navigation(
  const std::string& robot_name,
  const std::vector<messages::Waypoint>& path)
  -> rmf_utils::optional<std::size_t>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
  {
    std::cerr << "[Error]: No such robot [" << robot_name << "]." << std::endl;
    return rmf_utils::nullopt;
  }

  if (path.empty())
  {
    std::cerr << "[Error]: Path is empty." << std::endl;
    return rmf_utils::nullopt;
  }

  const std::size_t num_wp = _pimpl->graph->num_waypoints();
  std::vector<messages::Waypoint> transformed_path;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    auto wp = path[i];
    std::size_t wp_index = static_cast<std::size_t>(wp.index);

    // Check if the waypoint exists
    if (wp_index >= num_wp)
    {
      std::cerr << "[Error]: Waypoint [" << std::to_string(i)
        << "] on the path does not exist on the graph." << std::endl;
      return rmf_utils::nullopt;
    }

    // Check if the connection exists
    if (i + 1 != path.size())
    {
      auto next_wp = path[i+1];
      const rmf_traffic::agv::Graph::Lane* connecting_lane =
        _pimpl->graph->lane_from(wp.index, next_wp.index);
      if (!connecting_lane)
      {
        std::cerr << "[Error]: No connecting lane between waypoints ["
          << std::to_string(i) << "] & [" << std::to_string(i+1)
          << "] on the path." << std::endl;
        return rmf_utils::nullopt;
      }
    }
    
    // Check if the provided location matches the one found in the graph
    const auto g_wp = _pimpl->graph->get_waypoint(wp_index);
    const Eigen::Vector2d provided_loc {wp.location.x, wp.location.y};
    if (g_wp.get_map_name() != wp.location.level_name ||
      (provided_loc - g_wp.get_location()).norm() > 1e-3)
    {
      std::cerr << "[Error]: Provided waypoint [" << std::to_string(i)
        << "] on path does not match the waypoint on the graph." << std::endl;
      return rmf_utils::nullopt;
    }

    transformed_path.push_back(
      messages::Waypoint{
        wp.index,
        _pimpl->to_robot_transform->forward_transform(wp.location)});
  }

  // Handles the carry forward
  if ((++_pimpl->current_task_id) == _pimpl->idle_task_id)
    ++_pimpl->current_task_id;

  messages::NavigationRequest request {
    robot_name,
    _pimpl->current_task_id,
    transformed_path
  };

  std::shared_ptr<requests::RequestInfo> request_info(
    new free_fleet::requests::SimpleRequestInfo<messages::NavigationRequest>(
      request,
      [this](const messages::NavigationRequest& request_msg)
    {
      this->_pimpl->middleware->send_navigation_request(request_msg);
    },
      _pimpl->time_now_fn()));
  
  _pimpl->tasks[request.task_id] = request_info;
  _pimpl->unacknowledged_tasks[request.task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
} // namespace free_fleet
