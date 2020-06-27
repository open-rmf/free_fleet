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

#ifndef FREE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
#define FREE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP

#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

namespace free_fleet
{
namespace ros2
{

void to_ff_message(
    const rmf_fleet_msgs::msg::Location& in_msg, messages::Location& out_msg);

void to_ff_message(
    const rmf_fleet_msgs::msg::ModeRequest& in_msg, 
    messages::ModeRequest& out_msg);

void to_ff_message(
    const rmf_fleet_msgs::msg::PathRequest& in_msg, 
    messages::PathRequest& out_msg);

void to_ff_message(
    const rmf_fleet_msgs::msg::DestinationRequest& in_msg, 
    messages::DestinationRequest& out_msg);

// ----------------------------------------------------------------------------

void to_ros_message(
    const messages::Location& in_msg,
    rmf_fleet_msgs::msg::Location& out_msg);

void to_ros_message(
    const messages::RobotState& in_msg,
    rmf_fleet_msgs::msg::RobotState& out_msg);

} // namespace ros2
} // namespace free_fleet


#endif // FREE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
