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

#ifndef FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP
#define FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP

#include <deque>
#include <mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/Client.hpp>
#include <free_fleet/messages/Location.hpp>

#include "ClientNodeConfig.hpp"

namespace free_fleet
{
namespace ros1
{

class ClientNode
{
public:

  using SharedPtr = std::shared_ptr<ClientNode>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  using MoveBaseClient = 
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using MoveBaseClientSharedPtr = std::shared_ptr<MoveBaseClient>;
  using GoalState = actionlib::SimpleClientGoalState;

  static SharedPtr make(const ClientNodeConfig& config);

  ~ClientNode();

  struct Fields
  {
    /// Free fleet client
    Client::SharedPtr client;

    /// move base action client
    MoveBaseClientSharedPtr move_base_client;
  };

  void print_config();

private:

  // --------------------------------------------------------------------------
  // Basic ROS 1 items

  std::unique_ptr<ros::NodeHandle> node;

  std::unique_ptr<ros::Rate> update_rate;

  std::unique_ptr<ros::Rate> publish_rate;

  // --------------------------------------------------------------------------
  // Battery handling

  ros::Subscriber battery_percent_sub;

  std::mutex battery_state_mutex;

  sensor_msgs::BatteryState current_battery_state;

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg);

  // --------------------------------------------------------------------------
  // Level name handling 
  // TODO: decide on a better way to get level data
  // TODO: add functionality to handle level transition

  ros::Subscriber level_name_sub;

  std::mutex level_name_mutex;
  
  std_msgs::String current_level_name;

  void level_name_callback_fn(const std_msgs::String& msg);
  
  // --------------------------------------------------------------------------
  // Robot transform handling

  tf2_ros::Buffer tf2_buffer;

  tf2_ros::TransformListener tf2_listener;

  std::mutex robot_transform_mutex;

  geometry_msgs::TransformStamped current_robot_transform;

  geometry_msgs::TransformStamped previous_robot_transform;

  bool get_robot_transform();

  // --------------------------------------------------------------------------
  // Mode handling

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  std::atomic<bool> emergency;

  std::atomic<bool> paused;

  messages::RobotMode get_robot_mode();

  bool read_mode_request();

  // --------------------------------------------------------------------------
  // Path request handling

  bool read_path_request();

  // --------------------------------------------------------------------------
  // Destination request handling

  bool read_destination_request();

  // --------------------------------------------------------------------------
  // Task handling

  bool is_valid_request(
      const std::string& request_fleet_name,
      const std::string& request_robot_name,
      const std::string& request_task_id);

  move_base_msgs::MoveBaseGoal location_to_move_base_goal(
      const messages::Location& location) const;

  std::mutex task_id_mutex;

  std::string current_task_id;

  struct Goal
  {
    std::string level_name;
    move_base_msgs::MoveBaseGoal goal;
    bool sent = false;
    ros::Time goal_end_time;
  };

  std::mutex goal_path_mutex;

  std::deque<Goal> goal_path;

  void read_requests();

  void handle_requests();

  void publish_robot_state();

  // --------------------------------------------------------------------------
  // Threads and thread functions

  std::thread update_thread;

  std::thread publish_thread;

  void update_thread_fn();

  void publish_thread_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;

  Fields fields;

  ClientNode(const ClientNodeConfig& config);

  void start(Fields fields);

};

} // namespace ros1
} // namespace free_fleet

#endif // FREE_FLEET_CLIENT_ROS1__SRC__CLIENTNODE_HPP
