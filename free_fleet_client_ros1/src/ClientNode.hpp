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

  static SharedPtr make(const ClientNodeConfig& config);

  ~ClientNode();

  struct Fields
  {
    Client::SharedPtr client;
  };

  void print_config();

private:

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  std::mutex robot_transform_mutex;
  geometry_msgs::TransformStamped current_robot_transform;
  geometry_msgs::TransformStamped previous_robot_transform;

  ros::Subscriber battery_percent_sub;
  ros::Subscriber level_name_sub;

  std::mutex battery_state_mutex;
  sensor_msgs::BatteryState current_battery_state;

  std::mutex level_name_mutex;
  std_msgs::String current_level_name;

  std::mutex task_id_mutex;
  std::string current_task_id;

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg);

  void level_name_callback_fn(const std_msgs::String& msg);

  bool get_robot_transform();

  uint32_t get_robot_mode();

  void publish_robot_state();

  struct Goal
  {
    std::string level_name;
    move_base_msgs::MoveBaseGoal goal;
    bool sent = false;
    ros::Time wait_at_goal_time;
  };

  std::atomic<bool> paused;

  std::mutex goal_path_mutex;
  std::deque<Goal> goal_path;

  using MoveBaseClient = 
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using GoalState = actionlib::SimpleClientGoalState;

  MoveBaseClient move_base_client;

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
