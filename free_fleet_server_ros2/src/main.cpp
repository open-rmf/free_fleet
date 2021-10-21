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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "ServerNode.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Greetings from free_fleet_server_ros2" << std::endl;

  free_fleet::ros2::ServerNodeConfig server_node_config =
      free_fleet::ros2::ServerNodeConfig::make();
  server_node_config.fleet_name = "free_fleet_server_ros2";

  auto server_node = free_fleet::ros2::ServerNode::make(server_node_config);
  if (!server_node)
    return 1;

  rclcpp::executors::MultiThreadedExecutor executor {
      rclcpp::ExecutorOptions(), 2};
  executor.add_node(server_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
