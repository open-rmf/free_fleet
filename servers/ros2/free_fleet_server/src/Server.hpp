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

#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include "free_fleet/FreeFleet.h"

#include "dds/dds.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet
{

struct ServerConfig
{
  std::string fleet_name = "fleet_name";

  std::string fleet_state_topic = "fleet_state";
  std::string mode_request_topic = "mode_request";
  std::string path_request_topic = "path_request";
  std::string destination_request_topic = "destination_request";

  uint32_t dds_domain = std::numeric_limits<uint32_t>::max();
  std::string dds_robot_state_topic = "robot_state";
  std::string dds_mode_request_topic = "mode_request";
  std::string dds_path_request_topic = "path_request";
  std::string dds_destination_request_topic = "destination_request";

  double update_state_frequency = 10.0;
  double publish_state_frequency = 1.0;
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

  using DDSRobotStateSub = dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>;
  DDSRobotStateSub::SharedPtr dds_robot_state_sub;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStatePub = rclcpp::Publisher<FleetState>;
  FleetStatePub::SharedPtr fleet_state_pub;

  using Location = rmf_fleet_msgs::msg::Location;
  using RobotState = rmf_fleet_msgs::msg::RobotState;

  std::mutex robot_states_mutex;
  std::unordered_map<std::string, RobotState> robot_states;

  void dds_to_ros_location(const FreeFleetData_Location& dds_location, Location& ros_location)
      const;

  void dds_to_ros_robot_state(
      const std::shared_ptr<const FreeFleetData_RobotState>& dds_robot_state,
      RobotState& ros_robot_state) const;

  void get_fleet_state(FleetState& fleet_state);

  void update_state_callback();

  // TODO: clean up very very old states of stagnant or dead robots

  bool is_request_valid(const std::string& fleet_name, const std::string& robot_name);

  // --------------------------------------------------------------------------

  using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
  using ModeRequestSub = rclcpp::Subscription<ModeRequest>;
  ModeRequestSub::SharedPtr mode_request_sub;

  using DDSModeRequestPub = dds::DDSPublishHandler<FreeFleetData_ModeRequest>;
  DDSModeRequestPub::SharedPtr dds_mode_request_pub;

  void mode_request_callback(ModeRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using PathRequestSub = rclcpp::Subscription<PathRequest>;
  PathRequestSub::SharedPtr path_request_sub;

  using DDSPathRequestPub = dds::DDSPublishHandler<FreeFleetData_PathRequest>;
  DDSPathRequestPub::SharedPtr dds_path_request_pub;

  void path_request_callback(PathRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  using DestinationRequest = rmf_fleet_msgs::msg::DestinationRequest;
  using DestinationRequestSub = rclcpp::Subscription<DestinationRequest>;
  DestinationRequestSub::SharedPtr destination_request_sub;

  using DDSDestinationRequestPub = dds::DDSPublishHandler<FreeFleetData_DestinationRequest>;
  DDSDestinationRequestPub::SharedPtr dds_destination_request_pub;

  void destination_request_callback(DestinationRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  Server(const ServerConfig& _config);

};

} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__SERVER_HPP
