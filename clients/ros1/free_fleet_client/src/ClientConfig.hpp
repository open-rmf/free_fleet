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

#ifndef FREEFLEETCLIENT__SRC__CLIENTCONFIG_HPP
#define FREEFLEETCLIENT__SRC__CLIENTCONFIG_HPP

#include <iostream>

#include <ros/ros.h>

namespace free_fleet
{

struct ClientConfig
{
  std::string fleet_name = "fleet_name";
  std::string robot_name = "robot_name";
  std::string robot_model = "robot_model";

  std::string battery_state_topic = "/battery_state";
  std::string level_name_topic = "/level_name";

  std::string map_frame = "map";
  std::string robot_frame = "base_footprint";
  
  std::string move_base_server_name = "move_base";

  int dds_domain = 42;
  std::string dds_state_topic = "robot_state";
  std::string dds_mode_request_topic = "mode_request";
  std::string dds_path_request_topic = "path_request";
  std::string dds_destination_request_topic = "destination_request";

  double update_frequency = 10.0;
  double publish_frequency = 1.0;

  double max_dist_to_first_waypoint = 10.0;

  void print_config() const;

  static ClientConfig make();

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key, 
      std::string& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      int& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      double& param_out);

};

} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__CLIENTCONFIG_HPP
