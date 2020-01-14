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

#include "ServerNode.hpp"

namespace free_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(
    const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  SharedPtr server(new ServerNode(_config, _node_options));
  return server;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(
    const ServerNodeConfig& _config, 
    const rclcpp::NodeOptions& _node_options) :
  Node(_config.fleet_name + "_node", _node_options),
  server_node_config(_config)
{}

void ServerNode::print_config()
{
  server_node_config.print_config();
}

void ServerNode::start(Fields _fields)
{
  fields = std::move(_fields);
}

} // namespace ros2
} // namespace free_fleet
