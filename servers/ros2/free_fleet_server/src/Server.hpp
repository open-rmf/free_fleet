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
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include "free_fleet/FreeFleet.h"

#include "dds/dds.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

#include "RobotInfo.hpp"

namespace free_fleet
{

struct ServerConfig
{
  std::string fleet_name;

  std::string fleet_state_topic = "fleet_state";
  std::string mode_command_topic = "mode_command";
  std::string path_command_topic = "path_command";
  std::string destination_command_topic = "destination_command";

  uint32_t dds_domain = std::numeric_limits<uint32_t>::max();
  std::string dds_robot_state_topic = "robot_state";

  double update_state_frequency = 10.0;
  double publish_state_frequency = 1.0;
  uint32_t max_samples_per_update = 10;
};

class Server : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  static SharedPtr make(const ServerConfig& _config);

  ~Server();

  bool is_ready();

  void start();

private:

  ServerConfig server_config;

  std::atomic<bool> ready;

  dds_return_t return_code;

  dds_entity_t participant;

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  // --------------------------------------------------------------------------

  rclcpp::TimerBase::SharedPtr update_state_timer;

  using DDSRobotStateSub = dds::DDSSubscribeHandler<FreeFleetData_RobotState>;
  DDSRobotStateSub::SharedPtr dds_robot_state_sub;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStatePub = rclcpp::Publisher<FleetState>;
  FleetStatePub::SharedPtr fleet_state_pub;

  std::unordered_map<std::string, RobotInfo> robot_infos;

  void update_state_callback();

  // --------------------------------------------------------------------------

  using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
  using ModeRequestSub = rclcpp::Subscription<ModeRequest>;
  ModeRequestSub::SharedPtr mode_request_sub;

  void mode_request_callback(ModeRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using PathRequestSub = rclcpp::Subscription<DestinationRequest>;
  PathRequestSub::SharedPtr destination_request_sub;

  void path_request_callback(PathRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  using DestinationRequest = rmf_fleet_msgs::msg::DestinationRequest;
  using DestinationRequestSub = rclcpp::Subscription<PathRequest>;
  DestinationRequestSub::SharedPtr path_request_sub;

  void destination_request_callback(DestinationRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  Server(const ServerConfig& _config);

};

} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__SERVER_HPP
