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

#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ClientNode.hpp"

// #include <rclcpp_action/rclcpp_action.hpp>

// #include <nav2_msgs/action/navigate_to_pose.hpp>

// class ClientNode : public rclcpp::Node
// {
// public:
//   using SharedPtr = std::shared_ptr<ClientNode>;
//   using NavigateToPose = nav2_msgs::action::NavigateToPose;

//   static SharedPtr make(const std::string node_name)
//   {
//     SharedPtr client_node(new ClientNode(node_name));
//     if (!client_node)
//       return nullptr;

//     std::string action_name = "/NavigateToPose";
//     auto action_client = rclcpp_action::create_client<NavigateToPose>(
//         client_node, action_name);
//     // Make sure the server is actually there before continuing
//     RCLCPP_INFO(
//         client_node->get_logger(), 
//         "Waiting for \"%s\" action server", 
//         action_name.c_str());
//     action_client->wait_for_action_server();

//     client_node->_action_client = std::move(action_client);
//     return client_node;
//   }

// private:
//   std::shared_ptr<rclcpp_action::Client<NavigateToPose>> _action_client;

//   ClientNode(const std::string node_name)
//   : Node(node_name)
//   {}
// };


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Greetings from free_fleet_client" << std::endl;

  free_fleet::ClientNode::Config client_node_config;

  auto client_node = free_fleet::ClientNode::make(client_node_config);
  // if (!client_node)
  //   return 1;

  // rclcpp::exe


  // auto node = ClientNode::make("free_fleet_client_node");
  // if (!node)
  // {
  //   std::cout << "Something went wrong with creating a client" << std::endl;
  //   return 1;
  // }

  std::cout << "all done" << std::endl;
  return 0;
}
