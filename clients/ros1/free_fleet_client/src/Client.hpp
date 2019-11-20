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
#include <dds/dds.h>

#include "FreeFleet.h"

namespace free_fleet
{

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;
  using Duration = std::chrono::steady_clock::duration;

  /// Factory function that creates an instance of the Free Fleet DDS Client
  ///
  static SharedPtr make(
      const std::string& fleet_name,
      Duration publish_frequency = std::chrono::milliseconds(500));

  /// Desctructor
  ~Client();

  /// Checks that the Client is ready to start
  ///
  bool is_ready();

  /// Starts the Client with a starting state
  ///
  bool start(const FreeFleetData_RobotState& start_state);

  /// Updates the Client with the newest RobotState, in order to be passed to
  /// the server.
  ///
  void update_robot_state(const FreeFleetData_RobotState& state);
  
  struct PublishHandler
  {
    dds_entity_t topic;
    dds_entity_t writer;
  };

private:
  std::string fleet_name;
  Duration publish_frequency;
  std::atomic<bool> ready;

  dds_return_t return_code;
  dds_entity_t participant;
  dds_qos_t* qos;

  std::unique_ptr<ros::Rate> rate;
  PublishHandler robot_state_pub;
  // more publishers and subscribers to come

  std::mutex robot_state_mutex;
  FreeFleetData_RobotState robot_state;

  std::thread run_thread;

  Client(
      const std::string& fleet_name,
      Duration publish_frequency = std::chrono::milliseconds(500));

  bool make_publish_handler(
      const dds_topic_descriptor_t* descriptor,
      const std::string& topic_name,
      PublishHandler& publisher);

  void run_thread_fn();

};

} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__FREEFLEETCLIENT_HPP
