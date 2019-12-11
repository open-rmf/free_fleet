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

#ifndef FREEFLEETSERVER__SRC__SERVERCONFIG_HPP
#define FREEFLEETSERVER__SRC__SERVERCONFIG_HPP

#include <array>
#include <iostream>

namespace free_fleet
{

struct ServerConfig
{
  std::string fleet_name = "";

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
  double publish_state_frequency = 1.0;

  std::array<double, 9> transformation =
  {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };

  void print_config();
};

} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__SERVERCONFIG_HPP
