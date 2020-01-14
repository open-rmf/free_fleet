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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>

#include <free_fleet/Server.hpp>

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

  ServerNodeConfig server_node_config;

  Fields fields;

  ServerNode(
      const ServerNodeConfig& config, 
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  void start(Fields fields);

};

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP
