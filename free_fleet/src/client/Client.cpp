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

#include <free_fleet/Console.hpp>
#include <free_fleet/client/Client.hpp>
#include "internal_Client.hpp"

namespace free_fleet {

//==============================================================================
void Client::Implementation::set_callbacks()
{
  using namespace std::placeholders;
  middleware->set_pause_request_callback(
    std::bind(&Implementation::handle_pause_request, this, _1));
  middleware->set_resume_request_callback(
    std::bind(&Implementation::handle_resume_request, this, _1));
  middleware->set_dock_request_callback(
    std::bind(&Implementation::handle_dock_request, this, _1));
  middleware->set_navigation_request_callback(
    std::bind(&Implementation::handle_navigation_request, this, _1));
  middleware->set_relocalization_request_callback(
    std::bind(&Implementation::handle_relocalization_request, this, _1));
}

//==============================================================================
void Client::Implementation::complete_task()
{
  assert(task_id.has_value());
  last_task_id = task_id.value();
  task_id = std::nullopt; 
}

//==============================================================================
void Client::Implementation::run_once()
{
  // send state
  free_fleet::messages::RobotState new_state(
    status_handle->time(),
    robot_name,
    robot_model,
    task_id,
    status_handle->mode(),
    status_handle->battery_percent(),
    status_handle->location(),
    status_handle->target_path_waypoint_index());
  middleware->send_state(new_state);
}

//==============================================================================
void Client::Implementation::handle_pause_request(
  const messages::PauseRequest& request)
{
  if (!is_valid_request(request))
    return;
  task_id = request.task_id();
  task_ids.insert(task_id.value());
  command_handle->stop(
    [this](){complete_task();});
}

//==============================================================================
void Client::Implementation::handle_resume_request(
  const messages::ResumeRequest& request)
{
  if (!is_valid_request(request))
    return;
  task_id = request.task_id();
  task_ids.insert(task_id.value());
  command_handle->resume(
    [this](){complete_task();});
}

//==============================================================================
void Client::Implementation::handle_dock_request(
  const messages::DockRequest& request)
{
  if (!is_valid_request(request))
    return;
  task_id = request.task_id();
  task_ids.insert(task_id.value());
  command_handle->dock(
    request.dock_name(),
    [this](){complete_task();});
}

//==============================================================================
void Client::Implementation::handle_navigation_request(
  const messages::NavigationRequest& request)
{
  if (!is_valid_request(request))
    return;
  task_id = request.task_id();
  task_ids.insert(task_id.value());
  command_handle->follow_new_path(
    request.path(),
    [this](){complete_task();});
}

//==============================================================================
void Client::Implementation::handle_relocalization_request(
  const messages::RelocalizationRequest& request)
{
  if (!is_valid_request(request))
    return;
  task_id = request.task_id();
  task_ids.insert(task_id.value());
  command_handle->relocalize(
    request.location(),
    [this](){complete_task();});
}

//==============================================================================
auto Client::make(
  const std::string& robot_name,
  const std::string& robot_model,
  std::shared_ptr<client::CommandHandle> command_handle,
  std::shared_ptr<client::StatusHandle> status_handle,
  std::unique_ptr<transport::ClientMiddleware> middleware)
  -> std::shared_ptr<Client>
{
  auto make_error_fn = [](const std::string& error_msg)
  {
    fferr << error_msg << "\n";
    return nullptr;
  };

  if (robot_name.empty())
    return make_error_fn("Provided robot name must not be empty.");
  if (robot_model.empty())
    return make_error_fn("Provided robot model must not be empty.");
  if (!command_handle)
    return make_error_fn("Provided command handle cannot be null.");
  if (!status_handle)
    return make_error_fn("Provided status handle cannot be null.");
  if (!middleware)
    return make_error_fn("Provided middleware cannot be null.");

  std::shared_ptr<Client> new_client(new Client);
  new_client->_pimpl = rmf_utils::make_impl<Implementation>(Implementation());
  new_client->_pimpl->robot_name = robot_name;
  new_client->_pimpl->robot_model = robot_model;
  new_client->_pimpl->command_handle = std::move(command_handle);
  new_client->_pimpl->status_handle = std::move(status_handle);
  new_client->_pimpl->middleware = std::move(middleware);
  new_client->_pimpl->set_callbacks();
  return new_client;
}

//==============================================================================
Client::Client()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void Client::run_once()
{
  _pimpl->run_once();
}

//==============================================================================
} // namespace free_fleet
