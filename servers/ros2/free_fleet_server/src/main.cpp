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

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "Server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Greetings from free_fleet_server." << std::endl;

  auto server = free_fleet::Server::make();
  if (!server)
    return 1;

  rclcpp::executors::MultiThreadedExecutor executor {
      rclcpp::executor::ExecutorArgs(), 2};
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
  std::cout << "all done!" << std::endl;
  return 0;
}
