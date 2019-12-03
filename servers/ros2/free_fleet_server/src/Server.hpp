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

#ifndef FREEFLEETSERVER__SRC__SERVER_HPP
#define FREEFLEETSERVER__SRC__SERVER_HPP

#include <mutex>
#include <chrono>
#include <memory>
#include <limits>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include "free_fleet/FreeFleet.h"

#include "dds/dds.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet
{

struct ServerConfig
{
  std::string fleet_name;

  uint32_t dds_domain = std::numeric_limits<uint32_t>::max();
  std::string dds_robot_state_topic = "robot_state";
};

class Server : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(const ServerConfig& _config);

  ~Server();

  bool is_ready();

  void start();

private:

  std::atomic<bool> ready;

  rclcpp::TimerBase::SharedPtr update_timer;

  dds_return_t return_code;

  dds_entity_t participant;

  ServerConfig server_config;

  dds::DDSSubscribeHandler<FreeFleetData_RobotState>::SharedPtr 
      robot_state_sub;

  Server(const ServerConfig& _config);

  void update_callback();

};

} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__SERVER_HPP
