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

#include "Server.hpp"

#include "dds_utils/common.hpp"

namespace free_fleet
{

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server(new Server(_config));
  if (!server->is_ready())
    return nullptr;

  return server;
}

Server::~Server()
{}

bool Server::is_ready()
{
  return ready;
}

Server::Server(const ServerConfig& _config) :
  Node(_config.fleet_name + "_free_fleet_server"),
  server_config(_config)
{
  ready = false;

  participant = dds_create_participant(
    static_cast<dds_domainid_t>(server_config.dds_domain), NULL, NULL);

  dds_robot_state_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>(
          participant, &FreeFleetData_RobotState_desc,
          server_config.dds_robot_state_topic));
  if (!dds_robot_state_sub->is_ready())
    return;

  ready = true;
}

void Server::start()
{
  if (!is_ready())
  {
    RCLCPP_ERROR(get_logger(), "Server: is not ready, can't start");
    return;
  }

  using namespace std::chrono_literals;
  
  robot_states.clear();
  update_state_timer = create_wall_timer(
      100ms, std::bind(&Server::update_state_callback, this));

  using std::placeholders::_1;
  
  mode_request_sub = create_subscription<ModeRequest>(
      server_config.mode_request_topic, 10, std::bind(&Server::mode_request_callback, this, _1));
  
  path_request_sub = create_subscription<PathRequest>(
      server_config.path_request_topic, 10, std::bind(&Server::path_request_callback, this, _1));

  destination_request_sub = create_subscription<DestinationRequest>(
      server_config.destination_request_topic, 10, 
      std::bind(&Server::destination_request_callback, this, _1));
}

void Server::dds_to_ros_location(
    const FreeFleetData_Location& _dds_location, Location& _ros_location) const
{
  _ros_location.t.sec = _dds_location.sec;
  _ros_location.t.nanosec = _dds_location.nanosec;
  _ros_location.x = _dds_location.x;
  _ros_location.y = _dds_location.y;
  _ros_location.yaw = _dds_location.yaw;
  _ros_location.level_name = std::string(_dds_location.level_name);
}

void Server::dds_to_ros_robot_state(
    const std::shared_ptr<const FreeFleetData_RobotState>& _dds_robot_state, 
    RobotState& _ros_robot_state) const
{
  _ros_robot_state.name = std::string(_dds_robot_state->name);
  _ros_robot_state.model = std::string(_dds_robot_state->model);
  _ros_robot_state.task_id = std::string(_dds_robot_state->task_id);
  _ros_robot_state.mode.mode = _dds_robot_state->mode.mode;
  _ros_robot_state.battery_percent = _dds_robot_state->battery_percent;
  
  dds_to_ros_location(_dds_robot_state->location, _ros_robot_state.location);
  
  _ros_robot_state.path = {};
  for (uint32_t i = 0; i < _dds_robot_state->path._length; ++i)
  {
    Location new_point;
    dds_to_ros_location(_dds_robot_state->path._buffer[i], new_point);
    _ros_robot_state.path.push_back(new_point);
  }
}

void Server::get_fleet_state(FleetState& _fleet_state)
{
  _fleet_state.name = server_config.fleet_name;
  _fleet_state.robots = {};

  ReadLock robot_states_lock(robot_states_mutex);
  for (const auto it : robot_states)
    _fleet_state.robots.push_back(it.second);
}

void Server::update_state_callback()
{
  auto incoming_dds_states = dds_robot_state_sub->read();
  
  for (auto dds_robot_state : incoming_dds_states)
  {
    RobotState ros_robot_state;
    dds_to_ros_robot_state(dds_robot_state, ros_robot_state);

    WriteLock robot_states_lock(robot_states_mutex);
    auto it = robot_states.find(ros_robot_state.name);
    if (it == robot_states.end())
      RCLCPP_INFO(get_logger(), "registered a new robot, name: " + ros_robot_state.name);

    robot_states[ros_robot_state.name] = ros_robot_state;
  }
}

bool Server::is_request_valid(const std::string& _fleet_name, const std::string& _robot_name)
{
  if (_fleet_name != server_config.fleet_name)
    return false;

  ReadLock robot_states_lock(robot_states_mutex);
  auto it = robot_states.find(_robot_name);
  if (it == robot_states.end())
    return false;
  return true;
}

void Server::mode_request_callback(ModeRequest::UniquePtr _msg)
{
  if (!is_request_valid(_msg->fleet_name, _msg->robot_name))
    return;

  FreeFleetData_ModeRequest* dds_msg;
  dds_msg = FreeFleetData_ModeRequest__alloc();
  dds_msg->fleet_name = common::dds_string_alloc_and_copy(_msg->fleet_name);
  dds_msg->robot_name = common::dds_string_alloc_and_copy(_msg->robot_name);
  dds_msg->mode.mode = _msg->mode.mode;
  dds_msg->task_id = common::dds_string_alloc_and_copy(_msg->task_id);

  if (dds_mode_request_pub->write(dds_msg))
    RCLCPP_INFO(get_logger(), "published a ModeRequest over DDS.");

  FreeFleetData_ModeRequest_free(dds_msg, DDS_FREE_ALL);
}

void Server::path_request_callback(PathRequest::UniquePtr _msg)
{
  if (!is_request_valid(_msg->fleet_name, _msg->robot_name))
    return;

  FreeFleetData_PathRequest* dds_msg;
  dds_msg = FreeFleetData_PathRequest__alloc();
  dds_msg->fleet_name = common::dds_string_alloc_and_copy(_msg->fleet_name);
  dds_msg->robot_name = common::dds_string_alloc_and_copy(_msg->robot_name);
  dds_msg->task_id = common::dds_string_alloc_and_copy(_msg->task_id);
  
  uint32_t num_locations = static_cast<uint32_t>(_msg->path.size());
  dds_msg->path._maximum = num_locations;
  dds_msg->path._length = num_locations;
  dds_msg->path._buffer = FreeFleetData_PathRequest_path_seq_allocbuf(num_locations);
  for (uint32_t i = 0; i < num_locations; ++i)
  {
    dds_msg->path._buffer[i].sec = _msg->path[i].t.sec;
    dds_msg->path._buffer[i].nanosec = _msg->path[i].t.nanosec;
    dds_msg->path._buffer[i].x = _msg->path[i].x;
    dds_msg->path._buffer[i].y = _msg->path[i].y;
    dds_msg->path._buffer[i].yaw = _msg->path[i].yaw;
    dds_msg->path._buffer[i].level_name = 
        common::dds_string_alloc_and_copy(_msg->path[i].level_name);
  }
  dds_msg->path._release = false;

  if (dds_path_request_pub->write(dds_msg))
    RCLCPP_INFO(get_logger(), "published a PathRequest over DDS.");

  FreeFleetData_PathRequest_free(dds_msg, DDS_FREE_ALL);
}

void Server::destination_request_callback(DestinationRequest::UniquePtr _msg)
{
  if (!is_request_valid(_msg->fleet_name, _msg->robot_name))
    return;

  FreeFleetData_DestinationRequest* dds_msg;
  dds_msg = FreeFleetData_DestinationRequest__alloc();
  dds_msg->fleet_name = common::dds_string_alloc_and_copy(_msg->fleet_name);
  dds_msg->robot_name = common::dds_string_alloc_and_copy(_msg->robot_name);
  dds_msg->location.sec = _msg->destination.t.sec;
  dds_msg->location.nanosec = _msg->destination.t.nanosec;
  dds_msg->location.x = _msg->destination.x;
  dds_msg->location.y = _msg->destination.y;
  dds_msg->location.yaw = _msg->destination.yaw;
  dds_msg->location.level_name = common::dds_string_alloc_and_copy(_msg->destination.level_name);
  dds_msg->task_id = common::dds_string_alloc_and_copy(_msg->task_id);

  if (dds_destination_request_pub->write(dds_msg))
    RCLCPP_INFO(get_logger(), "published a DestinationRequest over DDS.");

  FreeFleetData_DestinationRequest_free(dds_msg, DDS_FREE_ALL);
}

} // namespace free_fleet
