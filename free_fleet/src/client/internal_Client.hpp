/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__CLIENT__INTERNAL_CLIENT_HPP
#define SRC__CLIENT__INTERNAL_CLIENT_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <optional>
#include <unordered_set>

#include <rmf_utils/Modular.hpp>

#include <free_fleet/Types.hpp>
#include <free_fleet/client/Client.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {

//==============================================================================
class Client::Implementation
{
public:

  Implementation()
  {}

  Implementation(const Implementation&)
  {
    // This is only used during the construction of the implementation class.
  }

  ~Implementation()
  {
    if (started.load() && async_thread.joinable())
      async_thread.join();
  }

  static Implementation& get(Client& client)
  {
    return *client._pimpl;
  }

  static const Implementation& get(const Client& client)
  {
    return *client._pimpl;
  }

  void set_callbacks();

  template<class T> 
  bool is_valid_request(const T& request)
  {
    if (request.robot_name() != robot_name)
      return false;

    // TODO(AA): Consider if there is the chance that this client idles for more
    // than 2 billion task ids, and suddenly gets a clashing id that has
    // overflowed and is slightly lower than the last task id.
    const auto it = task_ids.find(request.task_id());
    if (it == task_ids.end() ||
      rmf_utils::Modular(last_task_id).less_than(request.task_id()))
      return true;
    return false;
  }

  void complete_task();

  void run_once();

  void handle_pause_request(const messages::PauseRequest& request);

  void handle_resume_request(const messages::ResumeRequest& request);

  void handle_dock_request(const messages::DockRequest& request);

  void handle_navigation_request(const messages::NavigationRequest& request);

  void handle_relocalization_request(
    const messages::RelocalizationRequest& request);

  std::string robot_name;
  std::string robot_model;

  std::shared_ptr<client::CommandHandle> command_handle;
  std::shared_ptr<client::StatusHandle> status_handle;
  std::unique_ptr<transport::ClientMiddleware> middleware;

  // TODO(AA): handle overflow of uint32_t
  std::optional<TaskId> task_id = std::nullopt;
  TaskId last_task_id = 0; 
  std::unordered_set<uint32_t> task_ids;

  std::atomic<bool> started = false;
  std::thread async_thread;
};

} // namespace free_fleet

#endif // SRC__CLIENT__INTERNAL_CLIENT_HPP
