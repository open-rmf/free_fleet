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
#include <unordered_set>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/Console.hpp>
#include <free_fleet/client/Client.hpp>
#include "internal_Client.hpp"

namespace free_fleet {

//==============================================================================
void Client::Implementation::DataHandle::complete_command()
{
  if (!command_id.has_value())
  {
    fferr << "complete_command called without a valid command ID.\n";
    return;
  }

  last_command_id = command_id.value();
  command_completed = true;
}

//==============================================================================
void Client::Implementation::DataHandle::handle_pause_request(
  const messages::PauseRequest& request)
{
  if (!is_valid_request(request))
    return;

  command_completed = false;
  command_id = request.command_id();
  command_ids.insert(command_id.value());
  command_handle->stop(
    [d = shared_from_this()]()
    {
      d->complete_command();
    });
}

//==============================================================================
void Client::Implementation::DataHandle::handle_resume_request(
  const messages::ResumeRequest& request)
{
  if (!is_valid_request(request))
    return;
  command_completed = false;
  command_id = request.command_id();
  command_ids.insert(command_id.value());
  command_handle->resume(
    [d = shared_from_this()]()
    {
      d->complete_command();
    });
}

//==============================================================================
void Client::Implementation::DataHandle::handle_dock_request(
  const messages::DockRequest& request)
{
  if (!is_valid_request(request))
    return;
  command_completed = false;

  if (!command_handle->dock(
      request.dock_name(),
      [d = shared_from_this()]()
      {
        d->complete_command();
      }))
  {
    fferr << "Dock request unrecognized by client.\n";
    return;
  }

  command_id = request.command_id();
  command_ids.insert(command_id.value());
}

//==============================================================================
void Client::Implementation::DataHandle::handle_navigation_request(
  const messages::NavigationRequest& request)
{
  if (!is_valid_request(request))
    return;
  command_completed = false;
  command_id = request.command_id();
  command_ids.insert(command_id.value());
  command_handle->follow_new_path(
    request.path(),
    [d = shared_from_this()]()
    {
      d->complete_command();
    });
}

//==============================================================================
void Client::Implementation::DataHandle::handle_relocalization_request(
  const messages::RelocalizationRequest& request)
{
  if (!is_valid_request(request))
    return;
  command_completed = false;
  command_id = request.command_id();
  command_ids.insert(command_id.value());
  command_handle->relocalize(
    request.location(),
    [d = shared_from_this()]()
    {
      d->complete_command();
    });
}

//==============================================================================
void Client::Implementation::set_callbacks()
{
  middleware->set_pause_request_callback(
    [d = data](const messages::PauseRequest& req)
    {
      d->handle_pause_request(req);
    });
  middleware->set_resume_request_callback(
    [d = data](const messages::ResumeRequest& req)
    {
      d->handle_resume_request(req);
    });
  middleware->set_dock_request_callback(
    [d = data](const messages::DockRequest& req)
    {
      d->handle_dock_request(req);
    });
  middleware->set_navigation_request_callback(
    [d = data](const messages::NavigationRequest& req)
    {
      d->handle_navigation_request(req);
    });
  middleware->set_relocalization_request_callback(
    [d = data](const messages::RelocalizationRequest& req)
    {
      d->handle_relocalization_request(req);
    });
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

  std::shared_ptr<Implementation::DataHandle> data(
    new Implementation::DataHandle);
  data->robot_name = robot_name;
  data->robot_model = robot_model;
  data->command_handle = std::move(command_handle);

  std::shared_ptr<Client> new_client(new Client);
  new_client->_pimpl->status_handle = std::move(status_handle);
  new_client->_pimpl->middleware =
    std::shared_ptr<transport::ClientMiddleware>(middleware.release());
  new_client->_pimpl->data = std::move(data);
  new_client->_pimpl->set_callbacks();

  return new_client;
}

//==============================================================================
Client::Client()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
}

//==============================================================================
void Client::run_once()
{
  free_fleet::messages::RobotState new_state(
    _pimpl->status_handle->time(),
    _pimpl->data->robot_name,
    _pimpl->data->robot_model,
    _pimpl->data->command_id,
    _pimpl->data->command_completed,
    _pimpl->status_handle->mode(),
    _pimpl->status_handle->battery_percent(),
    _pimpl->status_handle->location(),
    _pimpl->status_handle->target_path_waypoint_index());
  _pimpl->middleware->send_state(new_state);
}

//==============================================================================
} // namespace free_fleet
