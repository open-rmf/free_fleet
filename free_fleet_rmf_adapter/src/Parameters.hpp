/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef FREE_FLEET_RMF_ADAPTER__SRC__PARAMETERS_HPP
#define FREE_FLEET_RMF_ADAPTER__SRC__PARAMETERS_HPP

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <free_fleet/StateSubscriber.hpp>
#include <free_fleet/RequestPublisher.hpp>

#include <rmf_traffic/agv/VehicleTraits.hpp>

#include "RmfFrameTransformer.hpp"

namespace free_fleet {

struct RmfAdapterParameters
{
  std::string fleet_name = "fleet_name";

  int domain_id = 42;
  std::string robot_state_topic = "robot_state";
  std::string mode_request_topic = "mode_request";
  std::string path_request_topic = "path_request";
  std::string destination_request_topic = "destination_request";

  // the transformation order of operations from the server to the client is:
  // 1) scale
  // 2) rotate
  // 3) translate
  double scale = 1.0;
  double rotation = 0.0;
  double translation_x = 0.0;
  double translation_y = 0.0;

  double linear_velocity = 0.7;
  double angular_velocity = 0.3;
  double linear_acceleration = 0.5;
  double angular_acceleration = 1.5;
  double footprint_radius = 0.5;
  double vicinity_radius = 1.5;
  bool reversible = true;

  std::string graph_file = "graph_file";

  bool perform_deliveries = false;

  bool get_parameters(std::shared_ptr<rclcpp::Node> node);

  void print_config() const;

  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> vehicle_traits() const;

  StateSubscriber::Config state_subscriber_config() const;

  RequestPublisher::Config request_publisher_config() const;

  RmfFrameTransformer::Transformation frame_transformation() const;
};

} // namespace free_fleet

#endif // FREE_FLEET_RMF_ADAPTER__SRC__PARAMETERS_HPP
