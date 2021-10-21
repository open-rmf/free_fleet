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

#ifndef FREE_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP
#define FREE_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP

#include <mutex>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>

#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include <free_fleet/Server.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotState.hpp>

#include "ServerNodeConfig.hpp"

namespace free_fleet
{
namespace ros2
{

class ServerNode : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<ServerNode>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(
      const ServerNodeConfig& config,
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  ~ServerNode();

  struct Fields
  {
    // Free fleet server
    Server::SharedPtr server;
  };

  void print_config();

private:

  bool is_request_valid(
      const std::string& fleet_name, const std::string& robot_name);

  void transform_fleet_to_rmf(
      const rmf_fleet_msgs::msg::Location& fleet_frame_location,
      rmf_fleet_msgs::msg::Location& rmf_frame_location) const;

  void transform_rmf_to_fleet(
      const rmf_fleet_msgs::msg::Location& rmf_frame_location,
      rmf_fleet_msgs::msg::Location& fleet_frame_location) const;

  // --------------------------------------------------------------------------

  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
      mode_request_sub;

  void handle_mode_request(rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
      path_request_sub;

  void handle_path_request(rmf_fleet_msgs::msg::PathRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  rclcpp::Subscription<rmf_fleet_msgs::msg::DestinationRequest>::SharedPtr
      destination_request_sub;

  void handle_destination_request(
      rmf_fleet_msgs::msg::DestinationRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  rclcpp::CallbackGroup::SharedPtr update_state_callback_group;

  rclcpp::TimerBase::SharedPtr update_state_timer;

  std::mutex robot_states_mutex;

  std::unordered_map<std::string, rmf_fleet_msgs::msg::RobotState>
      robot_states;

  void update_state_callback();

  // --------------------------------------------------------------------------

  rclcpp::CallbackGroup::SharedPtr
      fleet_state_pub_callback_group;

  rclcpp::TimerBase::SharedPtr fleet_state_pub_timer;

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr
      fleet_state_pub;

  void publish_fleet_state();

  // --------------------------------------------------------------------------

  ServerNodeConfig server_node_config;

  void setup_config();

  bool is_ready();

  Fields fields;

  ServerNode(
      const ServerNodeConfig& config, const rclcpp::NodeOptions& options);

  void start(Fields fields);

};

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP
