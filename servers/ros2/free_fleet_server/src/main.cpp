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

#include <rclcpp/rclcpp.hpp>

#include "Server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Greetings from free_fleet_server." << std::endl;

  rclcpp::executors::MultiThreadedExecutor executor;
  auto server = free_fleet::Server::make();
  if (!server)
  {  
    std::cout << "Server: unable to initialize." << std::endl;
    return 1;
  }

  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
  std::cout << "all done!" << std::endl;
  return 0;
}

