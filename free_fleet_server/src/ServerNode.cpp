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

namespace free_fleet {
namespace ros2 {

//==============================================================================

void ServerNode::Config::print_config() const
{
  setbuf(stdout, NULL);
  printf("ROS 2 SERVER CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  update state frequency: %.1f\n", update_state_frequency);
  printf("  publish state frequency: %.1f\n", publish_state_frequency);
  printf("  TOPICS\n");
  printf("    fleet state: %s\n", fleet_state_topic.c_str());
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
  printf("SERVER-CLIENT DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", dds_robot_state_topic.c_str());
  printf("    mode request: %s\n", dds_mode_request_topic.c_str());
  printf("    path request: %s\n", dds_path_request_topic.c_str());
  printf("    destination request: %s\n",
      dds_destination_request_topic.c_str());
  printf("COORDINATE TRANSFORMATION\n");
  printf("  translation x (meters): %.3f\n", translation_x);
  printf("  translation y (meters): %.3f\n", translation_y);
  printf("  rotation (radians): %.3f\n", rotation);
  printf("  scale: %.3f\n", scale);
}

//==============================================================================

Server::Config ServerNode::Config::get_server_config() const
{
  Server::Config server_config {
    dds_domain,
    dds_robot_state_topic,
    dds_mode_request_topic,
    dds_path_request_topic,
    dds_destination_request_topic};
  return server_config;
}

//==============================================================================

ServerNode::SharedPtr ServerNode::make(
    Config config, const rclcpp::NodeOptions& node_options)
{
  // Starting the free fleet server node
  SharedPtr server_node(new ServerNode(config, node_options));

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count() < 10)
  {
    rclcpp::spin_some(server_node);

    server_node->get_params();
    if (server_node->params_configured())
      break;
    RCLCPP_INFO(
        server_node->get_logger(), "waiting for configuration parameters.");
    
    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server_node->params_configured())
  {
    RCLCPP_ERROR(
        server_node->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server_node->_config.print_config();

  // Starting the free fleet server
  Server::Config server_config = server_node->_config.get_server_config();
  Server::SharedPtr server = Server::make(server_config);
  if (!server)
    return nullptr;

  server_node->_server = std::move(server);
  server_node->init_ros();
  return server_node;
}

//==============================================================================

ServerNode::~ServerNode()
{}

//==============================================================================

void ServerNode::init_ros()
{
  {
    WriteLock robot_states_lock(robot_states_mutex);
    robot_states.clear();
  }

  using namespace std::chrono_literals;

  // First callback group that handles getting updates from all the clients
  // available
  update_state_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  update_state_timer = create_wall_timer(
      100ms, std::bind(&ServerNode::update_state_callback, this),
      update_state_callback_group);

  // Second callback group that handles publishing fleet states to RMF, and
  // handling requests from RMF to be sent down to the clients
  fleet_state_pub_callback_group = create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  fleet_state_pub = 
      create_publisher<rmf_fleet_msgs::msg::FleetState>(
          _config.fleet_state_topic, 10);

  fleet_state_pub_timer = create_wall_timer(
      200ms, std::bind(&ServerNode::publish_fleet_state, this),
      fleet_state_pub_callback_group);

  // Mode request handling
  auto mode_request_sub_opt = rclcpp::SubscriptionOptions();
  mode_request_sub_opt.callback_group = fleet_state_pub_callback_group;
  mode_request_sub = create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      _config.mode_request_topic, rclcpp::QoS(10),
      [&](rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
      {
        handle_mode_request(std::move(msg));
      },
      mode_request_sub_opt);

  // Path reqeust handling
  auto path_request_sub_opt = rclcpp::SubscriptionOptions();
  path_request_sub_opt.callback_group = fleet_state_pub_callback_group;
  path_request_sub = create_subscription<rmf_fleet_msgs::msg::PathRequest>(
      _config.path_request_topic, rclcpp::QoS(10),
      [&](rmf_fleet_msgs::msg::PathRequest::UniquePtr msg)
      {
        handle_path_request(std::move(msg));
      },
      path_request_sub_opt);

  // Destination reqeust handling
  auto destination_request_sub_opt = rclcpp::SubscriptionOptions();
  destination_request_sub_opt.callback_group = fleet_state_pub_callback_group;
  destination_request_sub = 
      create_subscription<rmf_fleet_msgs::msg::DestinationRequest>(
          _config.destination_request_topic, rclcpp::QoS(10),
          [&](rmf_fleet_msgs::msg::DestinationRequest::UniquePtr msg)
          {
            handle_destination_request(std::move(msg));
          },
          destination_request_sub_opt);
}

//==============================================================================

void ServerNode::update_state_callback()
{
  std::vector<messages::RobotState> new_robot_states;
  _server->read_robot_states(new_robot_states);

  for (const messages::RobotState& ff_rs : new_robot_states)
  {
    rmf_fleet_msgs::msg::RobotState ros_rs;
    to_ros_message(ff_rs, ros_rs);

    WriteLock robot_states_lock(robot_states_mutex);
    auto it = robot_states.find(ros_rs.name);
    if (it == robot_states.end())
      RCLCPP_INFO(
          get_logger(),
          "registered a new robot: " + ros_rs.name);

    robot_states[ros_rs.name] = ros_rs;
  }
}

//==============================================================================

bool ServerNode::is_request_valid(
    const std::string& _fleet_name, const std::string& _robot_name)
{
  if (_fleet_name != _config.fleet_name)
    return false;

  ReadLock robot_states_lock(robot_states_mutex);
  auto it = robot_states.find(_robot_name);
  if (it == robot_states.end())
    return false;
  return true;
}

//==============================================================================

void ServerNode::transform_fleet_to_rmf(
    const rmf_fleet_msgs::msg::Location& _fleet_frame_location,
    rmf_fleet_msgs::msg::Location& _rmf_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d translated =
      Eigen::Vector2d(_fleet_frame_location.x, _fleet_frame_location.y)
      - Eigen::Vector2d(_config.translation_x, _config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(-_config.rotation) * translated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d scaled = 1.0 / _config.scale * rotated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);
      
  _rmf_frame_location.x = scaled[0];
  _rmf_frame_location.y = scaled[1];
  _rmf_frame_location.yaw = 
      _fleet_frame_location.yaw - _config.rotation;

  _rmf_frame_location.t = _fleet_frame_location.t;
  _rmf_frame_location.level_name = _fleet_frame_location.level_name;
}

//==============================================================================

void ServerNode::transform_rmf_to_fleet(
    const rmf_fleet_msgs::msg::Location& _rmf_frame_location,
    rmf_fleet_msgs::msg::Location& _fleet_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d scaled = 
      _config.scale * 
      Eigen::Vector2d(_rmf_frame_location.x, _rmf_frame_location.y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(_config.rotation) * scaled;
  
  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d translated =
      rotated + Eigen::Vector2d(_config.translation_x, _config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  _fleet_frame_location.x = translated[0];
  _fleet_frame_location.y = translated[1];
  _fleet_frame_location.yaw = 
      _rmf_frame_location.yaw + _config.rotation;

  _fleet_frame_location.t = _rmf_frame_location.t;
  _fleet_frame_location.level_name = _rmf_frame_location.level_name;
}

//==============================================================================

void ServerNode::handle_mode_request(
    rmf_fleet_msgs::msg::ModeRequest::UniquePtr _msg)
{
  messages::ModeRequest ff_msg;
  to_ff_message(*(_msg.get()), ff_msg);
  _server->send_mode_request(ff_msg);
}

//==============================================================================

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
  _server->send_path_request(ff_msg);
}

void ServerNode::handle_destination_request(
    rmf_fleet_msgs::msg::DestinationRequest::UniquePtr _msg)
{
  rmf_fleet_msgs::msg::Location fleet_frame_destination;
  transform_rmf_to_fleet(_msg->destination, fleet_frame_destination);
  _msg->destination = fleet_frame_destination;

  messages::DestinationRequest ff_msg;
  to_ff_message(*(_msg.get()), ff_msg);
  _server->send_destination_request(ff_msg);
}

//==============================================================================

void ServerNode::publish_fleet_state()
{
  rmf_fleet_msgs::msg::FleetState fleet_state;
  fleet_state.name = _config.fleet_name;
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

//==============================================================================

void ServerNode::get_params()
{
  get_parameter("fleet_name", _config.fleet_name);
  get_parameter("fleet_state_topic", _config.fleet_state_topic);
  get_parameter("mode_request_topic", _config.mode_request_topic);
  get_parameter("path_request_topic", _config.path_request_topic);
  get_parameter("destination_request_topic", _config.destination_request_topic);
  get_parameter("dds_domain", _config.dds_domain);
  get_parameter("dds_robot_state_topic", _config.dds_robot_state_topic);
  get_parameter("dds_mode_request_topic", _config.dds_mode_request_topic);
  get_parameter("dds_path_request_topic", _config.dds_path_request_topic);
  get_parameter("dds_destination_request_topic", 
      _config.dds_destination_request_topic);
  get_parameter("update_state_frequency", _config.update_state_frequency);
  get_parameter("publish_state_frequency", _config.publish_state_frequency);

  get_parameter("translation_x", _config.translation_x);
  get_parameter("translation_y", _config.translation_y);
  get_parameter("rotation", _config.rotation);
  get_parameter("scale", _config.scale);
}

//==============================================================================

bool ServerNode::params_configured() const
{
  if (_config.fleet_name == "fleet_name")
    return false;
  return true;
}

//==============================================================================

ServerNode::ServerNode(
    Config config, 
    const rclcpp::NodeOptions& _node_options)
: Node(config.fleet_name + "_node", _node_options),
  _config(std::move(config))
{}

} // namespace ros2
} // namespace free_fleet
