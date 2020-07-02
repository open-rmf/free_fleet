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

#include "Parameters.hpp"

namespace free_fleet {

//==============================================================================

bool RmfAdapterParameters::get_parameters(std::shared_ptr<rclcpp::Node> node)
{
  // TODO
  return true;
}

//==============================================================================

void RmfAdapterParameters::print_config() const
{
  printf("FREE FLEET - RMF ADAPTER PARAMETERS\n");
  printf("  domain ID: %d\n", domain_id);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", robot_state_topic.c_str());
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
  printf("COORDINATE TRANSFORMATION\n");
  printf("  translation x (meters): %.3f\n", translation_x);
  printf("  translation y (meters): %.3f\n", translation_y);
  printf("  rotation (radians): %.3f\n", rotation);
  printf("  scale: %.3f\n", scale);
  printf("VEHICLE TRAITS\n");
  printf("  linear velocity: %.3f\n", linear_velocity);
  printf("  angular velocity: %.3f\n", angular_velocity);
  printf("  linear acceleration: %.3f\n", linear_acceleration);
  printf("  angular acceleration: %.3f\n", angular_acceleration);
  printf("  footprint radius: %.3f\n", footprint_radius);
  printf("  vicinity radius: %.3f\n", vicinity_radius);
  printf("  reversible: %s\n", (reversible ? "Yes" : "No"));
  printf("NAVIGATION GRAPH\n");
  printf("  path: %s\n", graph_file.c_str());
  printf("ROBOT TRAITS\n");
  printf("  perform deliveries: %s\n", (perform_deliveries ? "Yes" : "No"));
}

//==============================================================================

std::shared_ptr<const rmf_traffic::agv::VehicleTraits>
  RmfAdapterParameters::vehicle_traits() const
{
  auto traits = std::shared_ptr<rmf_traffic::agv::VehicleTraits>(
      new rmf_traffic::agv::VehicleTraits {
          {linear_velocity, linear_acceleration},
          {angular_velocity, angular_acceleration},
          rmf_traffic::Profile {
              rmf_traffic::geometry::make_final_convex<
                  rmf_traffic::geometry::Circle>(footprint_radius),
              rmf_traffic::geometry::make_final_convex<
                  rmf_traffic::geometry::Circle>(vicinity_radius)
          }
      });
  traits->get_differential()->set_reversible(reversible);
  return traits;
}

//==============================================================================

StateSubscriber::Config RmfAdapterParameters::state_subscriber_config() const
{
  return StateSubscriber::Config {
    domain_id,
    robot_state_topic
  };
}

//==============================================================================

RequestPublisher::Config RmfAdapterParameters::request_publisher_config() const
{
  return RequestPublisher::Config {
    domain_id,
    mode_request_topic,
    path_request_topic,
    destination_request_topic
  };
}

//==============================================================================

RmfFrameTransformer::Transformation 
    RmfAdapterParameters::frame_transformation() const
{
  return RmfFrameTransformer::Transformation {
    scale,
    rotation,
    translation_x,
    translation_y
  };
}

//==============================================================================

} // namespace free_fleet
