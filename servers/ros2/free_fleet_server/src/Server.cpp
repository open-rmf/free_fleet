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

#include <mutex>
#include <chrono>
#include <memory>
#include <limits>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include "free_fleet/FreeFleet.h"

#include "dds/dds.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet
{

struct ServerConfig
{
  std::string fleet_name;

  uint32_t dds_domain = std::numeric_limits<uint32_t>::max();
  std::string dds_robot_state_topic = "robot_state";
};

class Server : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(const ServerConfig& _config)
  {
    SharedPtr server(new Server(_config));
    if (!server->is_ready())
      return nullptr;

    return server;
  }

  ~Server()
  {}

  bool is_ready()
  {
    return ready;
  }

  void start()
  {
    if (!is_ready())
    {
      RCLCPP_ERROR(get_logger(), "Server: is not ready, can't start");
      return;
    }

    using namespace std::chrono_literals;

    update_timer = create_wall_timer(
        100ms, std::bind(&Server::update_callback, this));
  }

private:

  std::atomic<bool> ready;

  rclcpp::TimerBase::SharedPtr update_timer;

  dds_return_t return_code;

  dds_entity_t participant;

  ServerConfig server_config;

  dds::DDSSubscribeHandler<FreeFleetData_RobotState>::SharedPtr robot_state_sub;

  Server(const ServerConfig& _config) :
    Node(_config.fleet_name + "_free_fleet_server"),
    server_config(_config)
  {
    ready = false;

    participant = dds_create_participant(
      static_cast<dds_domainid_t>(server_config.dds_domain), NULL, NULL);

    robot_state_sub.reset(
        new dds::DDSSubscribeHandler<FreeFleetData_RobotState>(
            participant, &FreeFleetData_RobotState_desc,
            server_config.dds_robot_state_topic));
    if (!robot_state_sub->is_ready())
      return;

    ready = true;
  }

  void update_callback()
  {
    auto new_robot_state = robot_state_sub->read();

    if (!new_robot_state)
      RCLCPP_INFO(get_logger(), "getting nothing yet.");
    else
      RCLCPP_INFO(get_logger(), "got a state through dds!");
  }

};

} // namespace free_fleet

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  free_fleet::ServerConfig config {"fake_flet"};
  auto server = free_fleet::Server::make(config);
  if (server->is_ready())
  {
    server->start();
    rclcpp::spin(server);
  }
  rclcpp::shutdown();
  std::cout << "all done!" << std::endl;
  return 0;

  // (void)argc;
  // (void)argv;

  // dds_entity_t participant = 
  //     dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  // if (participant < 0)
  //   DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  // dds_entity_t topic = dds_create_topic(
  //     participant, &FreeFleetData_RobotState_desc, "robot_state", NULL, NULL);
  // if (topic < 0)
  //   DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  // dds_qos_t* qos = dds_create_qos ();
  // dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  // dds_entity_t reader = dds_create_reader(participant, topic, qos, NULL);
  // if (reader < 0)
  //   DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  // dds_delete_qos(qos);

  // FreeFleetData_RobotState* state_msg;
  // int depth = 10;
  // void* samples[depth];
  // dds_sample_info_t infos[depth];

  // for (int i = 0; i < depth; ++i)
  //   samples[i] = FreeFleetData_RobotState__alloc();

  // // something awesome happens here

  // for (int i = 0; i < depth; ++i)
  //   FreeFleetData_RobotState_free(samples[i], DDS_FREE_ALL);

  // std::cout << "all done!" << std::endl;
  // return 0;
}
