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

#include <iostream>

#include <dds/dds.h>

#include "cxxopts/cxxopts.hpp"

#include "Client.hpp"
#include "free_fleet/FreeFleet.h"


/// We should expect no loss in communications between the robot stack and this
/// node, as they would normally be running on the same computer.
/// Troubleshooting and error handling will be implemented later on after DP2, 
/// right now the client expects all the fields to be updated via the different
/// topics regularly, which can then be published through DDS to the server.
///
/// Notes: 
/// * the time of the state will be tied to the transform
/// * location comes from listening to transforms
/// * battery from listening to a std_msgs/Float32
/// * mode is derived from all other sources of information
/// * level name will be a std_msgs/String
/// * calling robot commands using move_base_msgs/MoveBaseAction
/// 
/// TODO: 
/// * need to sync up the level topic with the transform of the robot, one
/// might be published after the other, the server might catch fire then
/// * handle losing connection with action server
///

free_fleet::ClientConfig parse(int argc, char** argv)
{
  try {
    cxxopts::Options options(
        "FreeFleetClient", 
        "Free Fleet Client for ROS 1 navigation stack robots.");

    free_fleet::ClientConfig default_config;

    options.add_options()
      ("f,fleet-name", "fleet name", cxxopts::value<std::string>())
      ("r,robot-name", "robot name", cxxopts::value<std::string>())
      ("m,robot-model", "robot model", cxxopts::value<std::string>())
      ("battery-state-topic", "battery state topic over ROS 1",
          cxxopts::value<std::string>()->default_value(
              default_config.battery_state_topic))
      ("level-name-topic", "level name topic over ROS 1",
          cxxopts::value<std::string>()->default_value(
              default_config.level_name_topic))
      ("map-frame", "name of map frame for the robot",
          cxxopts::value<std::string>()->default_value(
              default_config.map_frame))
      ("robot-frame", "name of robot frame",
          cxxopts::value<std::string>()->default_value(
              default_config.robot_frame))
      ("move-base", "name of the move base server",
          cxxopts::value<std::string>()->default_value(
              default_config.move_base_server_name))
      ("dds-domain", "domain ID for DDS communication",
          cxxopts::value<uint32_t>()->default_value(
              std::to_string(default_config.dds_domain)))
      ("dds-state-topic", "name DDS topic for robot states",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_state_topic))
      ("dds-mode-topic", "name DDS topic for robot mode commands",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_mode_command_topic))
      ("dds-path-topic", "name DDS topic for robot path commands",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_path_command_topic))
      ("dds-location-topic", "name DDS topic for location commands",
          cxxopts::value<std::string>()->default_value(
              default_config.dds_location_command_topic))
      ("update-frequency", 
          "frequency at which the client updates all the states and commands",
          cxxopts::value<float>()->default_value(
              std::to_string(default_config.update_frequency)))
      ("publish-frequency", "frequency at which the client publishes states",
          cxxopts::value<float>()->default_value(
              std::to_string(default_config.publish_frequency)))
      ("help", "Prints help")
    ;
      
    auto results = options.parse(argc, argv);
    if (results.count("help"))
    {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }

    // handle mandatory arguments
    if (!results.count("f") || !results.count("fleet-name"))
    {
      std::cout << "FreeFleetClient: fleet name must be specified" 
          << std::endl;
      exit(1);
    }
    if (!results.count("r") || !results.count("robot-name"))
    {
      std::cout << "FreeFleetClient: robot name must be specified" 
          << std::endl;
      exit(1);
    }
    if (!results.count("m") || !results.count("robot-model"))
    {
      std::cout << "FreeFleetClient: robot model must be specified"
          << std::endl;
      exit(1);
    }

    return free_fleet::ClientConfig {
      results["fleet-name"].as<std::string>(),
      results["robot-name"].as<std::string>(),
      results["robot-model"].as<std::string>(),
      results["battery-state-topic"].as<std::string>(),
      results["level-name-topic"].as<std::string>(),
      results["map-frame"].as<std::string>(),
      results["robot-frame"].as<std::string>(),
      results["move-base"].as<std::string>(),
      results["dds-domain"].as<uint32_t>(),
      results["dds-state-topic"].as<std::string>(),
      results["dds-mode-topic"].as<std::string>(),
      results["dds-path-topic"].as<std::string>(),
      results["dds-location-topic"].as<std::string>(),
      results["update-frequency"].as<float>(),
      results["publish-frequency"].as<float>()
    };
  }
  catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }
}

int main(int argc, char** argv)
{
  free_fleet::ClientConfig config = parse(argc, argv);
  std::cout << "Starting FreeFleetClient with configuration: " << std::endl;
  std::cout << "Fleet name: " << config.fleet_name << std::endl;
  std::cout << "Robot name: " << config.robot_name << std::endl;
  std::cout << "Robot model: " << config.robot_model << std::endl;
  std::cout << "ROS 1 - battery state topic: " << config.battery_state_topic 
      << std::endl;
  std::cout << "ROS 1 - level name topic:    " << config.level_name_topic
      << std::endl;
  std::cout << "ROS 1 - map frame:           " << config.map_frame 
      << std::endl;
  std::cout << "ROS 1 - robot frame:         " << config.robot_frame
      << std::endl;
  std::cout << "ROS 1 - move base server:    " << config.move_base_server_name
      << std::endl;
  std::cout << "DDS - domain:                       " 
      << config.dds_domain << std::endl;
  std::cout << "DDS - robot state topic:            " 
      << config.dds_state_topic << std::endl;
  std::cout << "DDS - robot mode command topic:     " 
      << config.dds_mode_command_topic << std::endl;
  std::cout << "DDS - robot path command topic:     "
      << config.dds_path_command_topic << std::endl;
  std::cout << "DDS - robot location command topic: "
      << config.dds_location_command_topic << std::endl;
  std::cout << "Client - update frequency: " << config.update_frequency
      << " Hz" << std::endl;
  std::cout << "Client - publish frequency: " << config.publish_frequency
      << " Hz" << std::endl;

  // Initialize all the ROS 1 items
  ros::init(argc, argv, "free_fleet_client");
  ros::NodeHandle ros_node_handle;
  ROS_INFO("greetings from free_fleet_client");

  auto client = free_fleet::Client::make(config);

  // Checks if the DDS client was created and is ready to roll
  if (!client->is_ready())
  {
    ROS_ERROR("Client: unable to initialize.");
    return 1;
  }

  // Start running the client with the starting state
  client->start();
  return 0;
}
