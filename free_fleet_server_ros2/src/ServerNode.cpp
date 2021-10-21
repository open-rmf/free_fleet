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

#include <Eigen/Geometry>

#include <free_fleet/Server.hpp>
#include <free_fleet/ServerConfig.hpp>

#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

#include "utilities.hpp"
#include "ServerNode.hpp"

namespace free_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(
    const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the free fleet server node
  SharedPtr server_node(new ServerNode(_config, _node_options));

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count() < 10)
  {
    rclcpp::spin_some(server_node);

    server_node->setup_config();
    if (server_node->is_ready())
      break;
    RCLCPP_INFO(
        server_node->get_logger(), "waiting for configuration parameters.");

    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server_node->is_ready())
  {
    RCLCPP_ERROR(
        server_node->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server_node->print_config();

  // Starting the free fleet server
  ServerConfig server_config =
      server_node->server_node_config.get_server_config();
  Server::SharedPtr server = Server::make(server_config);
  if (!server)
    return nullptr;

  server_node->start(Fields{
    std::move(server)
  });

  return server_node;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(
    const ServerNodeConfig& _config,
    const rclcpp::NodeOptions& _node_options) :
  Node(_config.fleet_name + "_node", _node_options),
  server_node_config(_config)
{}

void ServerNode::print_config()
{
  server_node_config.print_config();
}

void ServerNode::setup_config()
{
  get_parameter("fleet_name", server_node_config.fleet_name);
  get_parameter("fleet_state_topic", server_node_config.fleet_state_topic);
  get_parameter("mode_request_topic", server_node_config.mode_request_topic);
  get_parameter("path_request_topic", server_node_config.path_request_topic);
  get_parameter(
      "destination_request_topic",
      server_node_config.destination_request_topic);
  get_parameter("dds_domain", server_node_config.dds_domain);
  get_parameter("dds_robot_state_topic",
      server_node_config.dds_robot_state_topic);
  get_parameter("dds_mode_request_topic",
      server_node_config.dds_mode_request_topic);
  get_parameter("dds_path_request_topic",
      server_node_config.dds_path_request_topic);
  get_parameter(
      "dds_destination_request_topic",
      server_node_config.dds_destination_request_topic);
  get_parameter("update_state_frequency",
      server_node_config.update_state_frequency);
  get_parameter(
      "publish_state_frequency", server_node_config.publish_state_frequency);

  get_parameter("translation_x", server_node_config.translation_x);
  get_parameter("translation_y", server_node_config.translation_y);
  get_parameter("rotation", server_node_config.rotation);
  get_parameter("scale", server_node_config.scale);
}

bool ServerNode::is_ready()
{
  if (server_node_config.fleet_name == "fleet_name")
    return false;
  return true;
}

void ServerNode::start(Fields _fields)
{
  fields = std::move(_fields);

  {
    WriteLock robot_states_lock(robot_states_mutex);
    robot_states.clear();
  }

  using namespace std::chrono_literals;

  // --------------------------------------------------------------------------
  // First callback group that handles getting updates from all the clients
  // available

  update_state_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  update_state_timer = create_wall_timer(
      100ms, std::bind(&ServerNode::update_state_callback, this),
      update_state_callback_group);

  // --------------------------------------------------------------------------
  // Second callback group that handles publishing fleet states to RMF, and
  // handling requests from RMF to be sent down to the clients

  fleet_state_pub_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  fleet_state_pub =
      create_publisher<rmf_fleet_msgs::msg::FleetState>(
          server_node_config.fleet_state_topic, 10);

  fleet_state_pub_timer = create_wall_timer(
      std::chrono::seconds(1) / server_node_config.publish_state_frequency,
      std::bind(&ServerNode::publish_fleet_state, this),
      fleet_state_pub_callback_group);

  // --------------------------------------------------------------------------
  // Mode request handling

  auto mode_request_sub_opt = rclcpp::SubscriptionOptions();

  mode_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  mode_request_sub = create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      server_node_config.mode_request_topic, rclcpp::QoS(10),
      [&](rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
      {
        handle_mode_request(std::move(msg));
      },
      mode_request_sub_opt);

  // --------------------------------------------------------------------------
  // Path reqeust handling

  auto path_request_sub_opt = rclcpp::SubscriptionOptions();

  path_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  path_request_sub = create_subscription<rmf_fleet_msgs::msg::PathRequest>(
      server_node_config.path_request_topic, rclcpp::QoS(10),
      [&](rmf_fleet_msgs::msg::PathRequest::UniquePtr msg)
      {
        handle_path_request(std::move(msg));
      },
      path_request_sub_opt);

  // --------------------------------------------------------------------------
  // Destination reqeust handling

  auto destination_request_sub_opt = rclcpp::SubscriptionOptions();

  destination_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  destination_request_sub =
      create_subscription<rmf_fleet_msgs::msg::DestinationRequest>(
          server_node_config.destination_request_topic, rclcpp::QoS(10),
          [&](rmf_fleet_msgs::msg::DestinationRequest::UniquePtr msg)
          {
            handle_destination_request(std::move(msg));
          },
          destination_request_sub_opt);
}

bool ServerNode::is_request_valid(
    const std::string& _fleet_name, const std::string& _robot_name)
{
  if (_fleet_name != server_node_config.fleet_name)
    return false;

  ReadLock robot_states_lock(robot_states_mutex);
  auto it = robot_states.find(_robot_name);
  if (it == robot_states.end())
    return false;
  return true;
}

void ServerNode::transform_fleet_to_rmf(
    const rmf_fleet_msgs::msg::Location& _fleet_frame_location,
    rmf_fleet_msgs::msg::Location& _rmf_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d translated =
      Eigen::Vector2d(_fleet_frame_location.x, _fleet_frame_location.y)
      - Eigen::Vector2d(
          server_node_config.translation_x, server_node_config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(-server_node_config.rotation) * translated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d scaled = 1.0 / server_node_config.scale * rotated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);

  _rmf_frame_location.x = scaled[0];
  _rmf_frame_location.y = scaled[1];
  _rmf_frame_location.yaw =
      _fleet_frame_location.yaw - server_node_config.rotation;

  _rmf_frame_location.t = _fleet_frame_location.t;
  _rmf_frame_location.level_name = _fleet_frame_location.level_name;
}

void ServerNode::transform_rmf_to_fleet(
    const rmf_fleet_msgs::msg::Location& _rmf_frame_location,
    rmf_fleet_msgs::msg::Location& _fleet_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d scaled =
      server_node_config.scale *
      Eigen::Vector2d(_rmf_frame_location.x, _rmf_frame_location.y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(server_node_config.rotation) * scaled;

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d translated =
      rotated +
      Eigen::Vector2d(
          server_node_config.translation_x, server_node_config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  _fleet_frame_location.x = translated[0];
  _fleet_frame_location.y = translated[1];
  _fleet_frame_location.yaw =
      _rmf_frame_location.yaw + server_node_config.rotation;

  _fleet_frame_location.t = _rmf_frame_location.t;
  _fleet_frame_location.level_name = _rmf_frame_location.level_name;
}

void ServerNode::handle_mode_request(
    rmf_fleet_msgs::msg::ModeRequest::UniquePtr _msg)
{
  messages::ModeRequest ff_msg;
  to_ff_message(*(_msg.get()), ff_msg);
  fields.server->send_mode_request(ff_msg);
}

void ServerNode::handle_path_request(
    rmf_fleet_msgs::msg::PathRequest::UniquePtr _msg)
{
  for (std::size_t i = 0; i < _msg->path.size(); ++i)
  {
    rmf_fleet_msgs::msg::Location fleet_frame_waypoint;
    transform_rmf_to_fleet(_msg->path[i], fleet_frame_waypoint);
    _msg->path[i] = fleet_frame_waypoint;
  }

  messages::PathRequest ff_msg;
  to_ff_message(*(_msg.get()), ff_msg);
  fields.server->send_path_request(ff_msg);
}

void ServerNode::handle_destination_request(
    rmf_fleet_msgs::msg::DestinationRequest::UniquePtr _msg)
{
  rmf_fleet_msgs::msg::Location fleet_frame_destination;
  transform_rmf_to_fleet(_msg->destination, fleet_frame_destination);
  _msg->destination = fleet_frame_destination;

  messages::DestinationRequest ff_msg;
  to_ff_message(*(_msg.get()), ff_msg);
  fields.server->send_destination_request(ff_msg);
}

void ServerNode::update_state_callback()
{
  std::vector<messages::RobotState> new_robot_states;
  fields.server->read_robot_states(new_robot_states);

  for (const messages::RobotState& ff_rs : new_robot_states)
  {
    rmf_fleet_msgs::msg::RobotState ros_rs;
    to_ros_message(ff_rs, ros_rs);

    WriteLock robot_states_lock(robot_states_mutex);
    auto it = robot_states.find(ros_rs.name);
    if (it == robot_states.end())
      RCLCPP_INFO(
          get_logger(),
          "registered a new robot: [%s]",
          ros_rs.name.c_str());

    robot_states[ros_rs.name] = ros_rs;
  }
}

void ServerNode::publish_fleet_state()
{
  rmf_fleet_msgs::msg::FleetState fleet_state;
  fleet_state.name = server_node_config.fleet_name;
  fleet_state.robots.clear();

  ReadLock robot_states_lock(robot_states_mutex);
  for (const auto it : robot_states)
  {
    const auto fleet_frame_rs = it.second;
    rmf_fleet_msgs::msg::RobotState rmf_frame_rs;

    transform_fleet_to_rmf(fleet_frame_rs.location, rmf_frame_rs.location);

    // RCLCPP_INFO(
    //     get_logger(),
    //     "robot location: (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f)",
    //     fleet_frame_rs.location.x,
    //     fleet_frame_rs.location.y,
    //     fleet_frame_rs.location.yaw,
    //     rmf_frame_rs.location.x,
    //     rmf_frame_rs.location.y,
    //     rmf_frame_rs.location.yaw);

    rmf_frame_rs.name = fleet_frame_rs.name;
    rmf_frame_rs.model = fleet_frame_rs.model;
    rmf_frame_rs.task_id = fleet_frame_rs.task_id;
    rmf_frame_rs.mode = fleet_frame_rs.mode;
    rmf_frame_rs.battery_percent = fleet_frame_rs.battery_percent;

    rmf_frame_rs.path.clear();
    for (const auto& fleet_frame_path_loc : fleet_frame_rs.path)
    {
      rmf_fleet_msgs::msg::Location rmf_frame_path_loc;

      transform_fleet_to_rmf(fleet_frame_path_loc, rmf_frame_path_loc);

      rmf_frame_rs.path.push_back(rmf_frame_path_loc);
    }

    fleet_state.robots.push_back(rmf_frame_rs);
  }
  fleet_state_pub->publish(fleet_state);
}

} // namespace ros2
} // namespace free_fleet
