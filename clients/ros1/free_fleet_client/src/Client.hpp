/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
#define FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP

#include <chrono>
#include <memory>

#include <ros/ros.h>
#include <dds/dds.h>

#include "FreeFleet.h"

namespace free_fleet
{

class Client
{
public:
  
  using SharedPtr = std::shared_ptr<Client>;
  using Duration = std::chrono::steady_clock::duration;

  static SharedPtr make(
      const std::string& fleet_name,
      Duration publish_frequency = std::chrono::milliseconds(500));

  ~Client();

private:
  std::string fleet_name;
  Duration publish_frequency;

  std::unique_ptr<dds_entity_t> participant;
  std::unique_ptr<dds_entity_t> robot_state_topic;

  Client(
      const std::string& fleet_name,
      Duration publish_frequency = std::chrono::milliseconds(500));

};

} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
