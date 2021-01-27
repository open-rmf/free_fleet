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
    if (_thread.joinable())
      _thread.join();
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

  void start(uint32_t frequency);

  void run_once();

  void thread_fn();

  std::string _fleet_name;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<transport::Middleware> _middleware;
  std::shared_ptr<CoordinateTransformer> _to_robot_transform;
  TimeNow _time_now_fn;
  RobotUpdatedCallback _robot_updated_callback_fn;

  std::unordered_map<std::string, std::shared_ptr<agv::RobotInfo>> _robots;

  // Reserving task ID 0 for empty tasks
  const uint32_t _idle_task_id = 0;
  uint32_t _current_task_id = _idle_task_id;
  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    _tasks;
  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    _unacknowledged_tasks;

  std::mutex _mutex;
  std::thread _thread;
  uint32_t _thread_frequency;

  bool _started = false;
};

} // namespace free_fleet

#endif // SRC__INTERNAL_MANAGER_HPP
