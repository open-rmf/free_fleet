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

#ifndef FREE_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP
#define FREE_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP

#include <string>

namespace free_fleet
{
namespace ros2
{

struct ServerNodeConfig
{
  
  std::string fleet_name = "fleet_name";

  std::string fleet_state_topic = "fleet_state";
  std::string mode_request_topic = "mode_request";
  std::string path_request_topic = "path_request";
  std::string destination_request_topic = "destination_request";

  int dds_domain = 42;
  std::string dds_robot_state_topic = "robot_state";
  std::string dds_mode_request_topic = "mode_request";
  std::string dds_path_request_topic = "path_request";
  std::string dds_destination_request_topic = "destination_request";

  double update_state_frequency = 10.0;
  double publish_state_frequency = 10.0;

  // the transformation order of operations from the server to the client is:
  // 1) scale
  // 2) rotate
  // 3) translate
  double scale = 1.0;
  double rotation = 0.0;
  double translation_x = 0.0;
  double translation_y = 0.0;

  void print_config() const;

  ServerConfig get_server_config() const;

  static ServerNodeConfig make();

};

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP
