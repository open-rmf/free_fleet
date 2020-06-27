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

#include "utilities.hpp"

namespace free_fleet
{
namespace ros2
{

void to_ff_message(
    const rmf_fleet_msgs::msg::Location& _in_msg, messages::Location& _out_msg)
{
  _out_msg.sec = _in_msg.t.sec;
  _out_msg.nanosec = _in_msg.t.nanosec;
  _out_msg.x = _in_msg.x;
  _out_msg.y = _in_msg.y;
  _out_msg.yaw = _in_msg.yaw;
  _out_msg.level_name = _in_msg.level_name;
}

void to_ff_message(
    const rmf_fleet_msgs::msg::ModeRequest& _in_msg, 
    messages::ModeRequest& _out_msg)
{
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.robot_name = _in_msg.robot_name;
  _out_msg.mode.mode = _in_msg.mode.mode;
  _out_msg.task_id = _in_msg.task_id;
}

void to_ff_message(
    const rmf_fleet_msgs::msg::PathRequest& _in_msg, 
    messages::PathRequest& _out_msg)
{
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.robot_name = _in_msg.robot_name;

  _out_msg.path.clear();
  for (size_t i = 0; i < _in_msg.path.size(); ++i)
  {
    messages::Location tmp_loc_msg;
    to_ff_message(_in_msg.path[i], tmp_loc_msg);
    _out_msg.path.push_back(tmp_loc_msg);
  }

  _out_msg.task_id = _in_msg.task_id;
}

void to_ff_message(
    const rmf_fleet_msgs::msg::DestinationRequest& _in_msg, 
    messages::DestinationRequest& _out_msg)
{
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.robot_name = _in_msg.robot_name;
  to_ff_message(_in_msg.destination, _out_msg.destination);
  _out_msg.task_id = _in_msg.task_id;
}

void to_ros_message(
    const messages::Location& _in_msg,
    rmf_fleet_msgs::msg::Location& _out_msg)
{
  _out_msg.t.sec = _in_msg.sec;
  _out_msg.t.nanosec = _in_msg.nanosec;
  _out_msg.x = _in_msg.x;
  _out_msg.y = _in_msg.y;
  _out_msg.yaw = _in_msg.yaw;
  _out_msg.level_name = _in_msg.level_name;
}

void to_ros_message(
    const messages::RobotState& _in_msg,
    rmf_fleet_msgs::msg::RobotState& _out_msg)
{
  _out_msg.name = _in_msg.name;
  _out_msg.model = _in_msg.model;
  _out_msg.task_id = _in_msg.task_id;
  _out_msg.mode.mode = _in_msg.mode.mode;
  _out_msg.battery_percent = _in_msg.battery_percent;

  to_ros_message(_in_msg.location, _out_msg.location);

  _out_msg.path = {};
  for (size_t i = 0; i < _in_msg.path.size(); ++i)
  {
    rmf_fleet_msgs::msg::Location tmp_loc;
    to_ros_message(_in_msg.path[i], tmp_loc);
    _out_msg.path.push_back(tmp_loc);
  }
}

} // namespace ros2
} // namespace free_fleet
