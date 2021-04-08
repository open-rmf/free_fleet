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

#ifndef SRC__INTERNAL_MANAGER_HPP
#define SRC__INTERNAL_MANAGER_HPP

#include <mutex>
#include <atomic>
#include <thread>

#include <free_fleet/Manager.hpp>

#include "requests/RequestInfo.hpp"

namespace free_fleet {

//==============================================================================
class Manager::Implementation
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

  static Implementation& get(Manager& manager)
  {
    return *manager._pimpl;
  }

  static const Implementation& get(const Manager& manager)
  {
    return *manager._pimpl;
  }

  bool connected() const;

  void run_once();

  void run(uint32_t frequency);

  void start_async(uint32_t frequency);

  std::string fleet_name;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;
  std::unique_ptr<transport::Middleware> middleware;
  std::shared_ptr<const CoordinateTransformer> to_robot_transform;
  TimeNow time_now_fn;
  RobotUpdatedCallback robot_updated_callback_fn;

  std::unordered_map<std::string, std::shared_ptr<agv::RobotInfo>> robots;

  // Reserving task ID 0 for empty tasks
  const uint32_t idle_task_id = 0;
  uint32_t current_task_id = idle_task_id;
  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    tasks;
  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    unacknowledged_tasks;

  std::atomic<bool> started = false;
  std::mutex mutex;
  std::thread async_thread;
};

} // namespace free_fleet

#endif // SRC__INTERNAL_MANAGER_HPP
