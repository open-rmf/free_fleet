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

#include <chrono>
#include <iostream>
#include <unordered_set>

#include <rmf_traffic/Time.hpp>

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

  bool _connected() const
  {
    return _command_handle && _status_handle && _middleware;
  }

  template<class T> 
  bool _is_valid_request(const T& request)
  {
    if (request.robot_name == _robot_name &&
      _task_ids.find(request.task_id) == _task_ids.end())
      return true;
    return false;
  }

  std::string _robot_name;
  std::string _robot_model;

  // TODO(AA): handle overflow of uint32_t
  uint32_t _task_id;
  std::unordered_set<uint32_t> _task_ids;

  std::shared_ptr<CommandHandle> _command_handle;
  std::shared_ptr<StatusHandle> _status_handle;
  std::shared_ptr<transport::Middleware> _middleware;
};

//==============================================================================
Client::SharedPtr Client::make(
  const std::string& robot_name,
  const std::string& robot_model,
  std::shared_ptr<CommandHandle> command_handle,
  std::shared_ptr<StatusHandle> status_handle,
  std::shared_ptr<transport::Middleware> middleware)
{
  auto make_error_fn = [](const std::string& error_msg)
  {
    std::cerr << error_msg << std::endl;
    return nullptr;
  };

  if (robot_name.empty())
    return make_error_fn("Provided robot name must not be empty.");
  if (robot_model.empty())
    return make_error_fn("Provided robot model must not be empty.");
  if (!command_handle)
    return make_error_fn("Provided command handle is invalid.");
  if (!status_handle)
    return make_error_fn("Provided status handle is invalid.");
  if (!middleware)
    return make_error_fn("Provided middleware is invalid.");

  Client::SharedPtr new_client(new Client);
  new_client->_pimpl->_robot_name = robot_name;
  new_client->_pimpl->_robot_model = robot_model;
  new_client->_pimpl->_command_handle = std::move(command_handle);
  new_client->_pimpl->_status_handle = std::move(status_handle);
  new_client->_pimpl->_middleware = std::move(middleware);

  return new_client;
}

//==============================================================================
Client::Client()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Client::start(uint32_t frequency)
{
  const double seconds_per_iteration = 1.0 / frequency;
  const rmf_traffic::Duration duration_per_iteration =
    rmf_traffic::time::from_seconds(seconds_per_iteration);
  rmf_traffic::Time t_prev = std::chrono::steady_clock::now();

  while (_pimpl->_connected())
  {
    if (std::chrono::steady_clock::now() - t_prev < duration_per_iteration)
      continue;

    // send state
    free_fleet::messages::RobotState new_state {
      _pimpl->_robot_name,
      _pimpl->_robot_model,
      _pimpl->_task_id,
      _pimpl->_status_handle->mode(),
      _pimpl->_status_handle->battery_percent(),
      _pimpl->_status_handle->location(),
      static_cast<uint32_t>(
        _pimpl->_status_handle->target_path_waypoint_index())
    };
    _pimpl->_middleware->send_state(new_state);

    // read mode request
    auto mode_request = _pimpl->_middleware->read_mode_request();
    if (mode_request.has_value() && 
      _pimpl->_is_valid_request(mode_request.value()))
    {
      auto request = mode_request.value();
      _pimpl->_task_ids.insert(request.task_id);
      _pimpl->_task_id = request.task_id;
      if (request.mode.mode == messages::RobotMode::MODE_PAUSED)
        _pimpl->_command_handle->stop();
      else if (request.mode.mode == messages::RobotMode::MODE_MOVING)
        _pimpl->_command_handle->resume();
      continue;
    }
    
    // read relocalization request
    auto relocalization_request =
      _pimpl->_middleware->read_relocalization_request();
    if (relocalization_request.has_value() &&
      _pimpl->_is_valid_request(relocalization_request.value()))
    {
      auto request = relocalization_request.value();
      _pimpl->_task_ids.insert(request.task_id);
      _pimpl->_task_id = request.task_id;
      free_fleet::agv::CommandHandle::RequestCompleted callback =
        [this]() { _pimpl->_task_id = 0; };
      _pimpl->_command_handle->relocalize(
        request.location,
        callback);
      continue;
    }

    // read navigation request
    auto navigation_request = _pimpl->_middleware->read_navigation_request();
    if (navigation_request.has_value() &&
      _pimpl->_is_valid_request(navigation_request.value()))
    {
      auto request = navigation_request.value();
      _pimpl->_task_ids.insert(request.task_id);
      _pimpl->_task_id = request.task_id;
      free_fleet::agv::CommandHandle::RequestCompleted callback =
        [this]() { _pimpl->_task_id = 0; };
      _pimpl->_command_handle->follow_new_path(
        request.path,
        callback);
    }
  }
}

//==============================================================================

} // namespace agv
} // namespace free_fleet
