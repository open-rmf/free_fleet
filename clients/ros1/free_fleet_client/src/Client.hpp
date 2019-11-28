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

#ifndef FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
#define FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP

#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <limits>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet_msgs/RobotMode.h>
#include <free_fleet_msgs/Location.h>
#include <free_fleet_msgs/PathSequence.h>

#include <dds/dds.h>

#include "free_fleet/FreeFleet.h"


namespace free_fleet
{

struct ClientConfig
{
  std::string fleet_name;
  std::string robot_name;
  std::string robot_model;

  std::string battery_state_topic = "/battery_state";
  std::string level_name_topic = "/level_name";
  std::string path_topic = "/path";
  // path is questionable, maybe the client should derive the path on its own
  // based on the path commands

  std::string map_frame = "map";
  std::string robot_frame = "base_footprint";
  
  std::string move_base_server_name = "move_base";

  uint32_t dds_domain = std::numeric_limits<uint32_t>::max();
  std::string dds_state_topic = "robot_state";
  std::string dds_mode_command_topic = "robot_mode_command";
  std::string dds_path_command_topic = "robot_path_command";
  std::string dds_location_command_topic = "robot_location_command";

  float state_publish_frequency = 1.0;
  float operate_frequency = 0.1;
};

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;
  using Duration = std::chrono::steady_clock::duration;

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  /// Factory function that creates an instance of the Free Fleet DDS Client
  ///
  static SharedPtr make(const ClientConfig& config);

  /// Desctructor
  ~Client();

  /// Checks that the Client is ready to start
  ///
  bool is_ready();

  /// Starts the subscriptions to all the different topics and starts 
  /// publishing the state over DDS to the server
  void start();

  /// Holds all artifacts needed to subscribe over DDS
  ///
  struct DDSSubscribeHandler
  {
    using SharedPtr = std::shared_ptr<DDSSubscribeHandler>;

    dds_entity_t topic;
    dds_entity_t reader;
    void* samples[1];
    dds_sample_info_t infos[1];

    bool read();
  };

  DDSSubscribeHandler::SharedPtr make_dds_subscribe_handler(
      dds_topic_descriptor_t* topic_desc, const std::string& topic_name,
      size_t alloc_size);

private:

  ClientConfig client_config;

  std::atomic<bool> ready;

  dds_return_t return_code;
  dds_entity_t participant;

  std::unique_ptr<ros::NodeHandle> node;
  std::unique_ptr<ros::Rate> rate;

  // --------------------------------------------------------------------------
  // Everything needed for sending out robot states

  ros::Time last_write_time;

  dds_entity_t state_topic;
  dds_entity_t state_writer;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped robot_transform_stamped;
  geometry_msgs::TransformStamped prev_robot_transform_stamped;

  // create other subscribers here for updates
  ros::Subscriber mode_sub;
  ros::Subscriber battery_percent_sub;
  ros::Subscriber level_name_sub;
  ros::Subscriber path_sub;

  std::mutex robot_state_mutex;
  FreeFleetData_RobotState robot_state;

  std::mutex battery_state_mutex;
  sensor_msgs::BatteryState battery_state;

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg);

  void level_name_callback_fn(const std_msgs::String& msg);

  void path_callback_fn(const free_fleet_msgs::PathSequence& msg);

  bool get_robot_transform();

  void get_robot_mode();

  void publish_robot_state();

  // --------------------------------------------------------------------------
  // Everything needed for receiving and handling mode commands

  dds_entity_t mode_command_topic;
  dds_entity_t mode_command_reader;
  FreeFleetData_RobotMode* mode_command_msg;
  void* mode_command_samples[1];
  dds_sample_info_t mode_command_infos[1];

  bool read_mode_commands();

  // --------------------------------------------------------------------------
  // Everything needed for receiving and handling location commands

  dds_entity_t location_command_topic;
  dds_entity_t location_command_reader;
  FreeFleetData_Location* location_command_msg;
  void* location_command_samples[1];
  dds_sample_info_t location_command_infos[1];

  bool read_location_commands();

  // --------------------------------------------------------------------------
  // Everythign needed for receiving and handling path commands

  dds_entity_t path_command_topic;
  dds_entity_t path_command_reader;
  FreeFleetData_RobotState_path_seq* path_command_msg;
  void* path_command_samples[1];
  dds_sample_info_t path_command_infos[1];

  bool read_path_commands();

  // --------------------------------------------------------------------------

  move_base_msgs::MoveBaseGoal location_command_goal;

  std::vector<move_base_msgs::MoveBaseGoal> goal_path;

  using MoveBaseClient = 
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  MoveBaseClient move_base_client;

  std::thread run_thread;

  Client(const ClientConfig& config);

  void handle_commands();

  void run_thread_fn();

  // --------------------------------------------------------------------------
  // Some math related utilities

  float get_yaw_from_transform(
      const geometry_msgs::TransformStamped& transform_stamped) const; 

  geometry_msgs::Quaternion get_quat_from_yaw(float yaw) const;

  bool is_close(double x, double y, double min = 0.05) const;

  bool is_transform_close(
      const geometry_msgs::TransformStamped& transform_1,
      const geometry_msgs::TransformStamped& transform_2) const;

  // --------------------------------------------------------------------------
  // some C memory related stuff

  char* dds_string_alloc_and_copy(const std::string& str);

};

} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
