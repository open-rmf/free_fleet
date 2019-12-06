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

#include <rclcpp/rclcpp.hpp>

#include "Server.hpp"
#include "../third_party/cxxopts/cxxopts.hpp"

free_fleet::ServerConfig parse(int argc, char** argv)
{
  try {
    cxxopts::Options options(
        "FreeFleetServer",
        "Free Fleet Server for working with Free Fleet Clients");
    
    free_fleet::ServerConfig default_config;

    options.add_options()
      ("f,fleet-name", "fleet name",
          cxxopts::value<std::string>()->default_value(
              default_config.fleet_name))
      ("fleet-state-topic", "ROS 2 fleet state topic",
          cxxopts::value<std::string>()->default_value(
              default_config.fleet_state_topic))
      ("mode-request-topic", "ROS 2 mode request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.mode_request_topic))
      ("path-request-topic", "ROS 2 path request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.path_request_topic))
      ("destination-request-topic", "ROS 2 destination request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.destination_request_topic))
      ("dds-domain", "domain ID for DDS communication",
          cxxopts::value<uint32_t>()->default_value(
              std::to_string(default_config.dds_domain)))
      ("dds-robot-state-topic", "DDS robot state topic",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_robot_state_topic))
      ("dds-mode-request-topic", "DDS mode request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_mode_request_topic))
      ("dds-path-request-topic", "DDS path request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_path_request_topic))
      ("dds-destination-request-topic", "DDS destination request topic",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_destination_request_topic))
      ("update-state-frequency", 
          "frequency at which the server updates all the states and requests",
          cxxopts::value<double>()->default_value(
              std::to_string(default_config.update_state_frequency)))
      ("publish-state-frequency", 
          "frequency at which the server publishes fleet states",
          cxxopts::value<double>()->default_value(
              std::to_string(default_config.publish_state_frequency)))
      ("help", "Prints help")
    ;

    auto results = options.parse(argc, argv);
    if (results.count("help"))
    {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }

    return free_fleet::ServerConfig {
      results["fleet-name"].as<std::string>(),
      results["fleet-state-topic"].as<std::string>(),
      results["mode-request-topic"].as<std::string>(),
      results["path-request-topic"].as<std::string>(),
      results["destination-request-topic"].as<std::string>(),
      results["dds-domain"].as<uint32_t>(),
      results["dds-robot-state-topic"].as<std::string>(),
      results["dds-mode-request-topic"].as<std::string>(),
      results["dds-path-request-topic"].as<std::string>(),
      results["dds-destination-request-topic"].as<std::string>(),
      results["update-state-frequency"].as<double>(),
      results["publish-state-frequency"].as<double>()
    };
  }
  catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }
}

int main(int argc, char **argv)
{
  free_fleet::ServerConfig config = parse(argc, argv);
  std::cout << "Starting FreeFleetServer with configuration: " << std::endl;
  std::cout << "Fleet name: " << config.fleet_name << std::endl;
  std::cout << "ROS 2 - fleet state topic: " << config.fleet_state_topic 
      << std::endl;
  std::cout << "ROS 2 - mode request topic: " << config.mode_request_topic
      << std::endl;
  std::cout << "ROS 2 - path request topic: " << config.path_request_topic
      << std::endl;
  std::cout << "ROS 2 - destination request topic: " 
      << config.destination_request_topic << std::endl;
  std::cout << "DDS - domain: " << config.dds_domain << std::endl;
  std::cout << "DDS - robot state topic: " << config.dds_robot_state_topic
      << std::endl;
  std::cout << "DDS - mode request topic: " << config.dds_mode_request_topic
      << std::endl;
  std::cout << "DDS - path request topic: " << config.dds_path_request_topic
      << std::endl;
  std::cout << "DDS - destination request topic: " 
      << config.dds_destination_request_topic << std::endl;
  std::cout << "Server - update state frequency: "
      << config.update_state_frequency << std::endl;
  std::cout << "Server - publish state frequency: "
      << config.publish_state_frequency << std::endl;

  rclcpp::init(argc, argv);
  std::cout << "Greetings from free_fleet_server." << std::endl;

  rclcpp::executors::MultiThreadedExecutor executor;
  auto server = free_fleet::Server::make(config);
  if (!server || !server->is_ready())
  {  
    std::cout << "Server: unable to initialize." << std::endl;
    return 1;
  }

  server->start();
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
  std::cout << "all done!" << std::endl;
  return 0;
}

