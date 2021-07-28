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

#include <free_fleet/Console.hpp>
#include <free_fleet/manager/Manager.hpp>
#include <free_fleet/manager/RobotInfo.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include "internal_Manager.hpp"
#include "internal_RobotInfo.hpp"
#include "requests/RequestInfo.hpp"
#include "requests/SimpleRequestInfo.hpp"

namespace free_fleet {

//==============================================================================
void Manager::Implementation::set_callbacks()
{
  middleware->set_robot_state_callback(
    std::bind(
      &Manager::Implementation::handle_robot_state,
      this,
      std::placeholders::_1));
}

//==============================================================================
void Manager::Implementation::handle_robot_state(
  const messages::RobotState& state)
{
  std::lock_guard<std::mutex> lock(mutex);

  const std::string robot_name = state.name();
  const auto task_id = state.task_id();

  messages::RobotState transformed_state(
    state.time(),
    robot_name,
    state.model(),
    task_id,
    state.mode(),
    state.battery_percent(),
    to_robot_transform->backward_transform(state.location()),
    state.target_path_index());

  if (robot_name.empty())
  {
    fferr << "Incoming robot state's name is empty, ignoring.\n";
    return;
  }

  const auto r_it = robots.find(robot_name);
  bool new_robot = r_it == robots.end();
  if (new_robot)
  {
    auto new_robot = manager::RobotInfo::Implementation::make(
      transformed_state,
      graph,
      time_now_fn());
    if (new_robot)
    {
      robots[robot_name] = std::move(new_robot);
      ffinfo << "Registered new robot: [" << robot_name << "]\n";
    }
  }
  else
  {
    manager::RobotInfo::Implementation::update_state(
      *r_it->second,
      transformed_state,
      time_now_fn());
  }

  // Updates external uses of the robot's information
  if (r_it != robots.end() && robot_updated_callback_fn)
    robot_updated_callback_fn(*r_it->second);

  // for each robot figure out whether any tasks were not received yet
  if (task_id.has_value())
  {
    const auto t_it = unacknowledged_tasks.find(task_id.value());
    if (t_it != unacknowledged_tasks.end())
    {
      t_it->second->acknowledge_request();
      unacknowledged_tasks.erase(t_it);
    }
  }
}

//==============================================================================
auto Manager::make(
  const std::string& fleet_name,
  std::shared_ptr<const rmf_traffic::agv::Graph> graph,
  std::unique_ptr<transport::ServerMiddleware> middleware,
  std::shared_ptr<const manager::CoordinateTransformer> to_robot_transform,
  TimeNow time_now_fn,
  RobotUpdatedCallback robot_updated_callback_fn)
  -> std::shared_ptr<Manager>
{
  auto make_error_fn = [](const std::string& error_msg)
  {
    fferr << error_msg << "\n";
    return nullptr;
  };

  if (fleet_name.empty())
    return make_error_fn("Provided fleet name must not be empty.");
  if (!graph)
    return make_error_fn("Provided traffic graph cannot be null.");
  if (!middleware)
    return make_error_fn("Provided free fleet middleware cannot be null.");
  if (!to_robot_transform)
    return make_error_fn(
      "Provided free fleet manager::CoordinateTransformer cannot be null.");
  if (!time_now_fn)
    return make_error_fn("Provided time function cannot be null.");

  std::shared_ptr<Manager> manager_ptr(new Manager);
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
void Manager::run_once()
{
  // Send out all unreceived tasks again
  for (const auto t_it : _pimpl->unacknowledged_tasks)
  {
    t_it.second->send_request();
  }
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
  -> std::shared_ptr<const manager::RobotInfo>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  const auto it = _pimpl->robots.find(robot_name);
  if (it == _pimpl->robots.end())
    return nullptr;
  return it->second;
}

//==============================================================================
auto Manager::all_robots()
  -> std::vector<std::shared_ptr<const manager::RobotInfo>>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  std::vector<std::shared_ptr<const manager::RobotInfo>> infos;
  infos.reserve(_pimpl->robots.size());
  for (const auto it : _pimpl->robots)
  {
    infos.push_back(it.second);
  }
  return infos;
}

//==============================================================================
auto Manager::request_pause(const std::string& robot_name)
  -> std::optional<TaskId>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
    return std::nullopt;

  const TaskId request_task_id = ++_pimpl->current_task_id;
  messages::PauseRequest request(robot_name, request_task_id); 

  std::shared_ptr<manager::RequestInfo> request_info(
    new manager::SimpleRequestInfo<messages::PauseRequest>(
      request,
      [this](const messages::PauseRequest& request_msg)
    {
      this->_pimpl->middleware->send_pause_request(request_msg);
    },
      [this](){return _pimpl->time_now_fn();}));

  _pimpl->tasks[request_task_id] = request_info;
  _pimpl->unacknowledged_tasks[request_task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_resume(const std::string& robot_name)
  -> std::optional<TaskId>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
    return std::nullopt;

  const TaskId request_task_id = ++_pimpl->current_task_id;
  messages::ResumeRequest request(robot_name, request_task_id);

  std::shared_ptr<manager::RequestInfo> request_info(
    new manager::SimpleRequestInfo<messages::ResumeRequest>(
      request,
      [this](const messages::ResumeRequest& request_msg)
    {
      this->_pimpl->middleware->send_resume_request(request_msg);
    },
      [this](){return _pimpl->time_now_fn();}));
  
  _pimpl->tasks[request_task_id] = request_info;
  _pimpl->unacknowledged_tasks[request_task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_dock(
  const std::string& robot_name, const std::string& dock_name)
  -> std::optional<TaskId>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
    return std::nullopt;

  const TaskId request_task_id = ++_pimpl->current_task_id;
  messages::DockRequest request(
    robot_name,
    request_task_id,
    dock_name);

  std::shared_ptr<manager::RequestInfo> request_info(
    new manager::SimpleRequestInfo<messages::DockRequest>(
      request,
      [this](const messages::DockRequest& request_msg)
    {
      this->_pimpl->middleware->send_dock_request(request_msg);
    },
      [this](){return _pimpl->time_now_fn();}));
  
  _pimpl->tasks[request_task_id] = request_info;
  _pimpl->unacknowledged_tasks[request_task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_relocalization(
  const std::string& robot_name,
  const messages::Location& location,
  std::size_t last_visited_waypoint_index) -> std::optional<TaskId>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
    return std::nullopt;

  // Check if the waypoint exists
  const std::size_t num_wp = _pimpl->graph->num_waypoints();
  if (last_visited_waypoint_index >= num_wp)
  {
    fferr << "Waypoint [" << last_visited_waypoint_index
      << "] on the path does not exist on the graph.\n";
    return std::nullopt;
  }

  auto wp =
    _pimpl->graph->get_waypoint(last_visited_waypoint_index);
  const double dist_from_wp =
    (location.coordinates() - wp.get_location()).norm();
  if (dist_from_wp >= 10.0)
  {
    fferr << "Last visited waypoint [" << last_visited_waypoint_index
      << "] is too far away.\n";
    return std::nullopt;
  }

  messages::Location transformed_location =
    _pimpl->to_robot_transform->forward_transform(location);

  const TaskId request_task_id = ++_pimpl->current_task_id;
  messages::RelocalizationRequest request(
    robot_name,
    request_task_id,
    transformed_location,
    last_visited_waypoint_index);

  std::shared_ptr<manager::RequestInfo> request_info(
    new manager::SimpleRequestInfo<
      messages::RelocalizationRequest>(
        request,
        [this](const messages::RelocalizationRequest& request_msg)
    {
      this->_pimpl->middleware->send_relocalization_request(request_msg);
    },
        [this](){return _pimpl->time_now_fn();}));
  
  _pimpl->tasks[request_task_id] = request_info;
  _pimpl->unacknowledged_tasks[request_task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
auto Manager::request_navigation(
  const std::string& robot_name,
  const std::vector<Manager::NavigationPoint>& path)
  -> std::optional<TaskId>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (_pimpl->robots.find(robot_name) == _pimpl->robots.end())
    return std::nullopt;

  if (path.empty())
  {
    fferr << "Requested path is empty.\n";
    return std::nullopt;
  }

  const std::size_t num_wp = _pimpl->graph->num_waypoints();
  std::vector<messages::Waypoint> transformed_path;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const auto& nav_point = path[i];

    // Check if the waypoint exists
    if (nav_point.waypoint_index >= num_wp)
    {
      fferr << "Navigation point [" << i
        << "] on the path does not exist on the graph.\n";
      return std::nullopt;
    }

    // Check if the connection exists
    if (i + 1 != path.size())
    {
      const rmf_traffic::agv::Graph::Lane* connecting_lane =
        _pimpl->graph->lane_from(
          nav_point.waypoint_index, path[i+1].waypoint_index);
      if (!connecting_lane)
      {
        fferr << "No connecting lane between navigation points [" << i
          << "] & [" << (i+1) << "] on the path.\n";
        return std::nullopt;
      }
    }
    
    const auto g_wp = _pimpl->graph->get_waypoint(nav_point.waypoint_index);
    if (nav_point.yaw.has_value())
    {
      messages::Location g_loc =
        messages::Location(
          g_wp.get_map_name(), g_wp.get_location(), nav_point.yaw.value());

      transformed_path.push_back(
        messages::Waypoint(
          nav_point.waypoint_index,
          _pimpl->to_robot_transform->forward_transform(g_loc)));
    }
    else
    {
      messages::Location g_loc =
        messages::Location(g_wp.get_map_name(), g_wp.get_location());

      transformed_path.push_back(
        messages::Waypoint(
          nav_point.waypoint_index,
          _pimpl->to_robot_transform->forward_transform(g_loc)));
    }
  }

  const TaskId request_task_id = ++_pimpl->current_task_id;
  messages::NavigationRequest request(
    robot_name,
    request_task_id,
    transformed_path);

  std::shared_ptr<manager::RequestInfo> request_info(
    new manager::SimpleRequestInfo<messages::NavigationRequest>(
      request,
      [this](const messages::NavigationRequest& request_msg)
    {
      this->_pimpl->middleware->send_navigation_request(request_msg);
    },
      _pimpl->time_now_fn));
  
  _pimpl->tasks[request_task_id] = request_info;
  _pimpl->unacknowledged_tasks[request_task_id] = request_info;
  request_info->send_request();
  return _pimpl->current_task_id;
}

//==============================================================================
} // namespace free_fleet
