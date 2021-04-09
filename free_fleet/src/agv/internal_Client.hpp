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

#ifndef SRC__AGV__INTERNAL_CLIENT_HPP
#define SRC__AGV__INTERNAL_CLIENT_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <variant>
#include <unordered_set>

#include <free_fleet/agv/Client.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {
namespace agv {

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

  bool connected() const;

  void set_callbacks();

  template<class T> 
  bool is_valid_request(const T& request)
  {
    if (request.robot_name == robot_name &&
      task_ids.find(request.task_id) == task_ids.end())
      return true;
    return false;
  }

  void run_once();

  void run(uint32_t frequency);

  void start_async(uint32_t frequency);

  void handle_mode_request(const messages::ModeRequest& request);

  void handle_navigation_request(const messages::NavigationRequest& request);

  void handle_relocalization_request(
    const messages::RelocalizationRequest& request);

  std::string robot_name;
  std::string robot_model;

  std::shared_ptr<CommandHandle> command_handle;
  std::shared_ptr<StatusHandle> status_handle;
  std::unique_ptr<transport::ClientMiddleware> middleware;

  // TODO(AA): handle overflow of uint32_t
  uint32_t task_id = 0;
  std::unordered_set<uint32_t> task_ids;

  std::atomic<bool> started = false;
  std::thread async_thread;
};

} // namespace agv
} // namespace free_fleet

#endif // SRC__AGV__INTERNAL_CLIENT_HPP
