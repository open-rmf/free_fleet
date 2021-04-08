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
#include <thread>
#include <future>
#include <iostream>
#include <unordered_set>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/agv/Client.hpp>
#include "internal_Client.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
bool Client::Implementation::connected() const
{
  return command_handle && status_handle && middleware;
}

//==============================================================================
void Client::Implementation::run_once()
{
  // send state
  free_fleet::messages::RobotState new_state {
    robot_name,
    robot_model,
    task_id,
    status_handle->mode(),
    status_handle->battery_percent(),
    status_handle->location(),
    static_cast<uint32_t>(status_handle->target_path_waypoint_index())
  };
  middleware->send_state(new_state);

  // read mode request
  auto mode_request = middleware->read_mode_request();
  if (mode_request.has_value() && is_valid_request(mode_request.value()))
  {
    auto request = mode_request.value();
    task_ids.insert(request.task_id);
    task_id = request.task_id;
    if (request.mode.mode == messages::RobotMode::MODE_PAUSED)
      command_handle->stop();
    else if (request.mode.mode == messages::RobotMode::MODE_MOVING)
      command_handle->resume();
    return;
  }
  
  // read relocalization request
  auto relocalization_request = middleware->read_relocalization_request();
  if (relocalization_request.has_value() &&
    is_valid_request(relocalization_request.value()))
  {
    auto request = relocalization_request.value();
    task_ids.insert(request.task_id);
    task_id = request.task_id;
    free_fleet::agv::CommandHandle::RequestCompleted callback =
      [this]() { task_id = 0; };
    command_handle->relocalize(request.location, callback);
    return;
  }

  // read navigation request
  auto navigation_request = middleware->read_navigation_request();
  if (navigation_request.has_value() &&
    is_valid_request(navigation_request.value()))
  {
    auto request = navigation_request.value();
    task_ids.insert(request.task_id);
    task_id = request.task_id;
    free_fleet::agv::CommandHandle::RequestCompleted callback =
      [this]() { task_id = 0; };
    command_handle->follow_new_path(request.path, callback);
  }
}

//==============================================================================
void Client::Implementation::run(uint32_t frequency)
{
  const double seconds_per_iteration = 1.0 / frequency;
  const rmf_traffic::Duration duration_per_iteration =
    rmf_traffic::time::from_seconds(seconds_per_iteration);
  rmf_traffic::Time t_prev = std::chrono::steady_clock::now();

  while (connected())
  {
    if (std::chrono::steady_clock::now() - t_prev < duration_per_iteration)
      continue;
    t_prev = std::chrono::steady_clock::now();

    run_once();
  }
}

//==============================================================================
void Client::Implementation::start_async(uint32_t frequency)
{
  async_thread =
    std::thread(std::bind(&Client::Implementation::run, this, frequency));
}

//==============================================================================
Client::SharedPtr Client::make(
  const std::string& robot_name,
  const std::string& robot_model,
  std::shared_ptr<CommandHandle> command_handle,
  std::shared_ptr<StatusHandle> status_handle,
  std::unique_ptr<transport::ClientMiddleware> middleware)
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
  new_client->_pimpl = rmf_utils::make_impl<Implementation>(Implementation());
  new_client->_pimpl->robot_name = robot_name;
  new_client->_pimpl->robot_model = robot_model;
  new_client->_pimpl->command_handle = std::move(command_handle);
  new_client->_pimpl->status_handle = std::move(status_handle);
  new_client->_pimpl->middleware = std::move(middleware);
  return new_client;
}

//==============================================================================
Client::Client()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Client::run(uint32_t frequency)
{
  if (frequency == 0)
    throw std::range_error("[Error]: Frequency has to be greater than 0.");
  if (started())
    throw std::runtime_error("[Error]: Client has already been started.");
  _pimpl->started = true;

  _pimpl->run(frequency);
}

//==============================================================================
void Client::start_async(uint32_t frequency)
{
  if (frequency == 0)
    throw std::range_error("[Error]: Frequency has to be greater than 0.");
  if (started())
    throw std::runtime_error("[Error]: Client has already been started.");

  _pimpl->start_async(frequency);
}

//==============================================================================
bool Client::started() const
{
  return _pimpl->started.load();
}

//==============================================================================

} // namespace agv
} // namespace free_fleet
