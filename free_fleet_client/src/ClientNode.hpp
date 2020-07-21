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

#ifndef FREE_FLEET_CLIENT__SRC__CLIENTNODE_HPP
#define FREE_FLEET_CLIENT__SRC__CLIENTNODE_HPP

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <free_fleet/Client.hpp>

namespace free_fleet
{

class ClientNode : public rclcpp::Node
{
public:

  struct Config
  {
    std::string fleet_name = "fleet_name";
    std::string robot_name = "robot_name";
    std::string robot_model = "robot_model";

    int wait_timeout = 10;
    int publish_state_frequency = 5;
    int handle_request_frequency = 10;
    double max_dist_to_first_waypoint = 10.0;

    std::string battery_state_topic = "/battery_state";
    std::string level_name_topic = "/level_name";
    std::string action_server_name = "NavigateToPose";

    std::string map_frame = "map";
    std::string robot_frame = "base_footprint";

    int dds_domain = 42;
    std::string dds_robot_state_topic = "robot_state";
    std::string dds_mode_request_topic = "mode_request";
    std::string dds_path_request_topic = "path_request";
    std::string dds_destination_request_topic = "destination_request";

    void print_config() const;

    Client::Config get_client_config() const;
  };

  using SharedPtr = std::shared_ptr<ClientNode>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  static SharedPtr make(Config config);

  ~ClientNode();

private:

  void init_ros();

  void publish_state();

  void handle_request();

  tf2_ros::Buffer _tf2_buffer;
  tf2_ros::TransformListener _tf2_listener;

  rclcpp_action::Client<NavigateToPose>::SharedPtr _action_client;

  rclcpp::TimerBase::SharedPtr _publish_state_timer;
  rclcpp::TimerBase::SharedPtr _handle_request_timer;

  Config _config;
  Client::SharedPtr _client;

  void get_params();

  bool params_configured() const;

  ClientNode(Config config);

};

} // namespace free_fleet

#endif // FREE_FLEET_CLIENT__SRC__CLIENTNODE_HPP
