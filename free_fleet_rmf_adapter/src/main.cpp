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
#include <mutex>
#include <chrono>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <free_fleet/Participant.hpp>
#include <free_fleet/StateSubscriber.hpp>
#include <free_fleet/RequestPublisher.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include "utilities.hpp"
#include "Parameters.hpp"
#include "RobotCommand.hpp"

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  std::unordered_map<std::string, free_fleet::RobotCommand::SharedPtr> robots;

  std::mutex mutex;

  free_fleet::RmfAdapterParameters parameters;

  free_fleet::Participant::SharedPtr participant;

  free_fleet::StateSubscriber::SharedPtr state_subscriber;

  free_fleet::RequestPublisher::SharedPtr request_publisher;

  free_fleet::RmfFrameTransformer::SharedPtr frame_transformer;

  rclcpp::TimerBase::SharedPtr update_timer;

  void add_robot(
      const std::string& fleet_name,
      const free_fleet::messages::RobotState& state)
  {
    const auto& robot_name = state.name;

    free_fleet::RobotCommand::Config conf {
      parameters.domain_id,
      parameters.mode_request_topic,
      parameters.path_request_topic,
      parameters.destination_request_topic,
      fleet_name,
      robot_name,
      parameters.scale,
      parameters.rotation,
      parameters.translation_x,
      parameters.translation_y,
    };
    const auto command = free_fleet::RobotCommand::make(adapter->node(), conf);

    // TODO: handle level name
    const auto& l = state.location;
    fleet->add_robot(
        command, robot_name, traits->profile(),
        rmf_traffic::agv::compute_plan_starts(
            *graph, state.location.level_name, {l.x, l.y, l.yaw},
            rmf_traffic_ros2::convert(adapter->node()->now())),
        [c = weak_from_this(), command, robot_name = std::move(robot_name)](
        const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
    {
      const auto connections = c.lock();
      if (!connections)
        return;
      
      std::lock_guard<std::mutex> lock(connections->mutex);

      command->set_updater(updater);
      connections->robots[robot_name] = command;
    });
  }
};

//==============================================================================

std::shared_ptr<Connections> make_fleet(
    const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;

  const auto& node = adapter->node();
  if (!connections->parameters.get_parameters(node))
    return nullptr;
  const std::string fleet_name = connections->parameters.fleet_name;

  connections->traits = connections->parameters.vehicle_traits();

  connections->graph =
      std::make_shared<rmf_traffic::agv::Graph>(
          rmf_fleet_adapter::agv::parse_graph(
              connections->parameters.graph_file, *connections->traits));

  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  connections->fleet = adapter->add_fleet(
        fleet_name, *connections->traits, *connections->graph);

  if (connections->parameters.perform_deliveries)
    connections->fleet->accept_delivery_requests(
          [](const rmf_task_msgs::msg::Delivery&){ return true; });

  connections->participant = free_fleet::Participant::make(
      connections->parameters.domain_id);
  
  connections->state_subscriber = free_fleet::StateSubscriber::make(
      connections->parameters.state_subscriber_config(),
      connections->participant);
  
  connections->request_publisher = free_fleet::RequestPublisher::make(
      connections->parameters.request_publisher_config(),
      connections->participant);

  connections->frame_transformer = free_fleet::RmfFrameTransformer::make(
      connections->parameters.frame_transformation());

  using namespace std::chrono_literals;
  connections->update_timer = node->create_wall_timer(
      10ms, 
      [c = std::weak_ptr<Connections>(connections), fleet_name]()
  {
    const auto connections = c.lock();
    if (!connections)
      return;

    std::vector<free_fleet::messages::RobotState> incoming_states;
    if (conenctions->state_subscriber->read_robot_states(incoming_states))
    {
      for (const auto& rs : incoming_states))
      {
        const auto insertion = connections->robots.insert({rs.name, nullptr});
        const bool new_robot = insertion.second;
        if (new_robot)
        {
          // We have not seen this robot before, so let's add it ot the fleet
          connections->add_robot(fleet_name, rs);
        }

        const auto& command = insertion.first->second;
        if (command)
        {
          // We are ready to command this robot, so let's update its state
          command->update_state(rs);
        }
      }
    }
  });

  return connections;
}

//==============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  // Start running the adapter and wait until it gets stopped by SIGINT
  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");
  adapter->start().wait();
  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");
  rclcpp::shutdown();
}
