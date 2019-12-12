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

#include <chrono>

#include "Server.hpp"

#include "math/math.hpp"
#include "dds_utils/common.hpp"

namespace free_fleet
{

Server::SharedPtr Server::make(
    const std::string& _node_name, const rclcpp::NodeOptions& _node_options)
{
  SharedPtr server(new Server(_node_name, _node_options));

  /// Wait for at least 10 seconds for the necessary parameters to be registered 
  /// on the parameter server, after declaring it.
  ///
  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(
          end_time - start_time).count() < 10)
  {
    rclcpp::spin_some(server);
    server->setup_config();
    if (server->is_ready())
      break;
    RCLCPP_INFO(server->get_logger(), "waiting for configuration parameters.");
    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server->is_ready())
  {
    RCLCPP_ERROR(server->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server->print_config();
  
  if (!server->setup_dds())
  {
    RCLCPP_ERROR(server->get_logger(),"unable to setup DDS components");
    return nullptr;
  }

  server->start();
  return server;
}

Server::~Server()
{}

Server::Server(
    const std::string& _node_name, const rclcpp::NodeOptions& _node_options) :
  Node(_node_name, _node_options)
{}

void Server::setup_config()
{
  get_parameter("fleet_name", server_config.fleet_name);
  get_parameter("fleet_state_topic", server_config.fleet_state_topic);
  get_parameter("mode_request_topic", server_config.mode_request_topic);
  get_parameter("path_request_topic", server_config.path_request_topic);
  get_parameter(
      "destination_request_topic", server_config.destination_request_topic);
  get_parameter("dds_domain", server_config.dds_domain);
  get_parameter("dds_robot_state_topic", server_config.dds_robot_state_topic);
  get_parameter("dds_mode_request_topic", server_config.dds_mode_request_topic);
  get_parameter("dds_path_request_topic", server_config.dds_path_request_topic);
  get_parameter(
      "dds_destination_request_topic",
      server_config.dds_destination_request_topic);
  get_parameter("update_state_frequency", server_config.update_state_frequency);
  get_parameter(
      "publish_state_frequency", server_config.publish_state_frequency);

  std::vector<double> transformation_param;
  get_parameter("transformation", transformation_param);
  if (transformation_param.size() != 9)
    RCLCPP_WARN(
        get_logger(), 
        "parameter transformation needs to be a 9-element array.");
  else
    for (size_t i = 0; i < 9; ++i)
      server_config.transformation[i] = transformation_param[i];
}

bool Server::setup_dds()
{
  participant = dds_create_participant(
    static_cast<dds_domainid_t>(server_config.dds_domain), NULL, NULL);

  dds_robot_state_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>(
          participant, &FreeFleetData_RobotState_desc,
          server_config.dds_robot_state_topic));
  if (!dds_robot_state_sub->is_ready())
    return false;

  dds_mode_request_pub.reset(
      new dds::DDSPublishHandler<FreeFleetData_ModeRequest>(
          participant, &FreeFleetData_ModeRequest_desc,
          server_config.dds_mode_request_topic));
  dds_path_request_pub.reset(
      new dds::DDSPublishHandler<FreeFleetData_PathRequest>(
          participant, &FreeFleetData_PathRequest_desc,
          server_config.dds_path_request_topic));
  dds_destination_request_pub.reset(
      new dds::DDSPublishHandler<FreeFleetData_DestinationRequest>(
          participant, &FreeFleetData_DestinationRequest_desc,
          server_config.dds_destination_request_topic));
  if (!dds_mode_request_pub->is_ready() ||
      !dds_path_request_pub->is_ready() ||
      !dds_destination_request_pub->is_ready())
    return false;

  return true;
}

bool Server::is_ready()
{
  if (server_config.fleet_name == "")
    return false;
  return true;
}

void Server::start()
{
  fleet_to_rmf_transform = math::convert(server_config.transformation);
  fleet_to_rmf_yaw = fleet_to_rmf_transform.eulerAngles(0, 1, 2);
  rmf_to_fleet_transform = fleet_to_rmf_transform.inverse();

  update_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  fleet_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  using namespace std::chrono_literals;
  
  robot_states.clear();

  update_state_timer = create_wall_timer(
      100ms, std::bind(&Server::update_state_callback, this), 
      update_callback_group);

  fleet_state_pub_timer = create_wall_timer(
      1s, std::bind(&Server::publish_fleet_state, this),
      fleet_callback_group);

  fleet_state_pub = 
      create_publisher<FleetState>(server_config.fleet_state_topic, 10);

  auto mode_request_sub_opt = rclcpp::SubscriptionOptions();
  mode_request_sub_opt.callback_group = fleet_callback_group;
  mode_request_sub = create_subscription<ModeRequest>(
      server_config.mode_request_topic, 
      rclcpp::QoS(10),
      [&](ModeRequest::UniquePtr msg)
      {
        mode_request_callback(std::move(msg));
      },
      mode_request_sub_opt);
  
  auto path_request_sub_opt = rclcpp::SubscriptionOptions();
  path_request_sub_opt.callback_group = fleet_callback_group;
  path_request_sub = create_subscription<PathRequest>(
      server_config.path_request_topic, 
      rclcpp::QoS(10), 
      [&](PathRequest::UniquePtr msg)
      {
        path_request_callback(std::move(msg)); 
      },
      path_request_sub_opt);

  auto destination_request_sub_opt = rclcpp::SubscriptionOptions();
  destination_request_sub_opt.callback_group = fleet_callback_group;
  destination_request_sub = create_subscription<DestinationRequest>(
      server_config.destination_request_topic, 
      rclcpp::QoS(10),
      [&](DestinationRequest::UniquePtr msg)
      {
        destination_request_callback(std::move(msg));
      },
      destination_request_sub_opt);
}

void Server::print_config()
{
  server_config.print_config();
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
  {
    RobotState transformed_robot_state = it.second;
    math::transform_robot_state(
        fleet_to_rmf_transform, fleet_to_rmf_yaw, transformed_robot_state);
    _fleet_state.robots.push_back(transformed_robot_state);
  }
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
      RCLCPP_INFO(
          get_logger(), 
          "registered a new robot, name: " + ros_robot_state.name);

    robot_states[ros_robot_state.name] = ros_robot_state;
  }
}

bool Server::is_request_valid(
    const std::string& _fleet_name, const std::string& _robot_name)
{
  if (_fleet_name != server_config.fleet_name)
    return false;

  ReadLock robot_states_lock(robot_states_mutex);
  auto it = robot_states.find(_robot_name);
  if (it == robot_states.end())
    return false;
  return true;
}

void Server::publish_fleet_state()
{
  FleetState new_fleet_state;
  get_fleet_state(new_fleet_state);

  fleet_state_pub->publish(new_fleet_state);
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
  dds_msg->path._buffer = 
      FreeFleetData_PathRequest_path_seq_allocbuf(num_locations);
  for (uint32_t i = 0; i < num_locations; ++i)
  {
    Location transformed_waypoint = _msg->path[i];
    math::transform_location(
        rmf_to_fleet_transform, -fleet_to_rmf_yaw, transformed_waypoint);

    dds_msg->path._buffer[i].sec = transformed_waypoint.t.sec;
    dds_msg->path._buffer[i].nanosec = transformed_waypoint.t.nanosec;
    dds_msg->path._buffer[i].x = transformed_waypoint.x;
    dds_msg->path._buffer[i].y = transformed_waypoint.y;
    dds_msg->path._buffer[i].yaw = transformed_waypoint.yaw;
    dds_msg->path._buffer[i].level_name = 
        common::dds_string_alloc_and_copy(transformed_waypoint.level_name);
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

  Location transformed_destination = _msg->destination;
  math::transform_location(
      rmf_to_fleet_transform, -fleet_to_rmf_yaw, transformed_destination);

  FreeFleetData_DestinationRequest* dds_msg;
  dds_msg = FreeFleetData_DestinationRequest__alloc();
  dds_msg->fleet_name = common::dds_string_alloc_and_copy(_msg->fleet_name);
  dds_msg->robot_name = common::dds_string_alloc_and_copy(_msg->robot_name);
  dds_msg->destination.sec = transformed_destination.t.sec;
  dds_msg->destination.nanosec = transformed_destination.t.nanosec;
  dds_msg->destination.x = transformed_destination.x;
  dds_msg->destination.y = transformed_destination.y;
  dds_msg->destination.yaw = transformed_destination.yaw;
  dds_msg->destination.level_name = 
      common::dds_string_alloc_and_copy(transformed_destination.level_name);
  dds_msg->task_id = common::dds_string_alloc_and_copy(_msg->task_id);

  if (dds_destination_request_pub->write(dds_msg))
    RCLCPP_INFO(get_logger(), "published a DestinationRequest over DDS.");

  FreeFleetData_DestinationRequest_free(dds_msg, DDS_FREE_ALL);
}

} // namespace free_fleet
