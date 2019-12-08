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

#include <yaml-cpp/yaml.h>

#include "Server.hpp"
#include "dds_utils/common.hpp"
#include "../third_party/cxxopts/cxxopts.hpp"


free_fleet::ServerConfig parse_config(const cxxopts::ParseResult& _results)
{
  free_fleet::ServerConfig server_config;

  std::string config_file_path = _results["config"].as<std::string>();
  if (config_file_path == "")
    return server_config;

  const YAML::Node yaml_config = YAML::LoadFile(config_file_path);
  if (!yaml_config)
    return server_config;

  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["fleet_name"], server_config.fleet_name);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["fleet_state_topic"], server_config.fleet_state_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["mode_request_topic"], server_config.mode_request_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["path_request_topic"], server_config.path_request_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["destination_request_topic"], 
      server_config.destination_request_topic);
  free_fleet::common::get_param_if_available<uint32_t>(
      yaml_config["dds_domain"], server_config.dds_domain);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["dds_robot_state_topic"], 
      server_config.dds_robot_state_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["dds_mode_request_topic"], 
      server_config.dds_mode_request_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["dds_path_request_topic"], 
      server_config.dds_path_request_topic);
  free_fleet::common::get_param_if_available<std::string>(
      yaml_config["dds_destination_request_topic"], 
      server_config.dds_destination_request_topic);
  free_fleet::common::get_param_if_available<double>(
      yaml_config["update_state_frequency"], 
      server_config.update_state_frequency);
  free_fleet::common::get_param_if_available<double>(
      yaml_config["publish_state_frequency"], 
      server_config.publish_state_frequency);

  const YAML::Node transformation_node = yaml_config["transformation"];
  if (transformation_node && 
      transformation_node.Type() == YAML::NodeType::Sequence &&
      transformation_node.size() == 9)
  {
    for (size_t i = 0; i < 9; ++i)
      server_config.transformation[i] = transformation_node[i].as<double>();
  }
  return server_config;
}

free_fleet::ServerConfig parse(int argc, char** argv)
{
  try {
    cxxopts::Options options(
        "FreeFleetServer",
        "Free Fleet Server for working with Free Fleet Clients");
    
    free_fleet::ServerConfig default_config;

    options.add_options()
      ("c,config", "config file",
          cxxopts::value<std::string>()->default_value(""))
      ("help", "Prints help")
    ;

    cxxopts::ParseResult results = options.parse(argc, argv);
    if (results.count("help"))
    {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }
    return parse_config(results);
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
  std::cout << "Map - transformation to RMF frame: " << std::endl;
  std::cout << config.transformation[0] << " " << config.transformation[1]
      << " " << config.transformation[2] << std::endl;
  std::cout << config.transformation[3] << " " << config.transformation[4]
      << " " << config.transformation[5] << std::endl;
  std::cout << config.transformation[6] << " " << config.transformation[7]
      << " " << config.transformation[8] << std::endl;
  std::cout << std::endl;

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

