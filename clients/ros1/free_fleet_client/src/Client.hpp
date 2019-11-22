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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet_msgs/RobotMode.h>
#include <free_fleet_msgs/Location.h>
#include <free_fleet_msgs/PathSequence.h>

#include <dds/dds.h>

#include "FreeFleet.h"


namespace free_fleet
{

struct ClientConfig
{
  std::string fleet_name;
  std::string robot_name;
  std::string robot_model;

  std::string mode_topic = "/robot_mode";
  std::string battery_state_topic = "/battery_state";
  std::string level_name_topic = "/level_name";
  std::string path_topic = "/path";

  std::string map_frame = "map";
  std::string target_frame = "base_footprint";
  
  std::string move_base_action_name = "move_base";

  dds_domainid_t dds_domain = DDS_DOMAIN_DEFAULT;
  std::string dds_state_topic = "robot_state";
  std::string dds_command_topic = "robot_command";
  using Duration = std::chrono::steady_clock::duration;
  Duration publish_frequency = std::chrono::milliseconds(1000);
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

  /// Updates the Client with the newest RobotState, in order to be passed to
  /// the server.
  ///
  void update_robot_state(const FreeFleetData_RobotState& state);
  
  struct DDSPublishHandler
  {
    dds_entity_t topic;
    dds_entity_t writer;
  };

  struct DDSSubscribeHandler
  {
    dds_entity_t topic;
    dds_entity_t reader;
  };

private:

  ClientConfig client_config;

  std::atomic<bool> ready;

  dds_return_t return_code;
  dds_entity_t participant;
  dds_qos_t* qos;

  std::unique_ptr<ros::NodeHandle> node;
  std::unique_ptr<ros::Rate> rate;

  // --------------------------------------------------------------------------
  // Everything needed for sending out robot states

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped robot_transform_stamped;

  // create other subscribers here for updates
  ros::Subscriber mode_sub;
  ros::Subscriber battery_percent_sub;
  ros::Subscriber level_name_sub;
  ros::Subscriber path_sub;

  std::mutex robot_state_mutex;
  FreeFleetData_RobotState robot_state;

  DDSPublishHandler robot_state_pub;

  // --------------------------------------------------------------------------
  // Everything needed for receiving commands and passing it down
  
  void* location_command_samples[1];
  dds_sample_info_t infos[1];

  FreeFleetData_Location location_command;

  using MoveBaseClient = 
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  MoveBaseClient move_base_client;

  DDSSubscribeHandler location_command_sub;

  // --------------------------------------------------------------------------

  std::thread run_thread;

  Client(const ClientConfig& config);

  bool make_publish_handler(
      const dds_topic_descriptor_t* descriptor,
      const std::string& topic_name,
      DDSPublishHandler& publish_handler);

  bool make_subscribe_handler(
      const dds_topic_descriptor_t* descriptor,
      const std::string& topic_name,
      DDSSubscribeHandler& subscriber_handler);

  bool get_robot_transform();

  void mode_callback_fn(const free_fleet_msgs::RobotMode& msg);

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg);

  void level_name_callback_fn(const std_msgs::String& msg);

  void path_callback_fn(const free_fleet_msgs::PathSequence& msg);

  bool publish_robot_state();

  bool read_commands();

  bool send_commands();

  float get_yaw_from_transform(
      const geometry_msgs::TransformStamped& transform_stamped) const; 

  void run_thread_fn();

  // --------------------------------------------------------------------------
  // some C memory related stuff

  char* dds_string_alloc_and_copy(const std::string& str);

};

} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
