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

#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/ModeRequest.hpp>

#include "utilities.hpp"
#include "RobotCommand.hpp"

namespace free_fleet {

//==============================================================================

void RobotCommand::Config::print_config() const
{
  printf("ROBOT COMMAND CONFIGURATION\n");
  printf("  domain ID: %d\n", domain_id);
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  robot name: %s\n", robot_name.c_str());
  printf("  TOPICS\n");
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
  printf("COORDINATE TRANSFORMATION\n");
  printf("  translation x (meters): %.3f\n", translation_x);
  printf("  translation y (meters): %.3f\n", translation_y);
  printf("  rotation (radians): %.3f\n", rotation);
  printf("  scale: %.3f\n", scale);
}

//==============================================================================

RequestPublisher::Config RobotCommand::Config::request_publisher_config() const
{
  return RequestPublisher::Config {
    domain_id,
    mode_request_topic,
    path_request_topic,
    destination_request_topic};
}

//==============================================================================

RmfFrameTransformer::Transformation RobotCommand::Config::transformation() const
{
  return RmfFrameTransformer::Transformation {
    scale,
    rotation,
    translation_x,
    translation_y};
}

//==============================================================================

RobotCommand::SharedPtr RobotCommand::make(
    std::shared_ptr<rclcpp::Node> node,
    Config config)
{
  RequestPublisher::SharedPtr request_publisher =
      RequestPublisher::make(config.request_publisher_config());
  if (!request_publisher)
    return nullptr;

  RmfFrameTransformer::SharedPtr frame_transformer =
      RmfFrameTransformer::make(config.transformation());
  if (!frame_transformer)
    return nullptr;

  RobotComamd::SharedPtr command_ptr(new RobotCommand);
  command_ptr->_active = false;
  command_ptr->_node = std::move(node);
  command_ptr->_request_publisher = std::move(request_publisher);
  command_ptr->_frame_transformer = std::move(frame_transformer);
  commadn_ptr->_config = std::move(config);
  return command_ptr;
}

//==============================================================================

void RobotCommand::follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    std::function<void()> path_finished_callback) final
{
}

//==============================================================================

void RobotCommand::stop() final
{
  messages::RobotMode mode_msg = { messages::RobotMode::MODE_PAUSED };
  messages::ModeRequest msg = {
    _config.fleet_name,
    _config.robot_name,
    std::move(mode_msg),
    generate_random_task_id(20)
  };
  if (!_request_publisher->send_mode_request(msg))
    RCLCPP_ERROR(_node->get_logger(), "Failed to send a stop command.");
}

//==============================================================================

void RobotCommand::dock(
    const std::string& dock_name,
    std::function<void()> docking_finished_callback) final
{
  // Free fleet robots are by default without any docking procedure. This
  // function is implemented as a no-op.
}

//==============================================================================

RobotCommand::RobotCommand()
{}

//==============================================================================

} // namespace free_fleet
