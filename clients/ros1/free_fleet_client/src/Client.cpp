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

#include "Client.hpp"

namespace free_fleet
{

std::shared_ptr<Client> Client::make(
    const std::string& _fleet_name,
    Duration _publish_frequency)
{
  std::shared_ptr<Client> client(new Client(_fleet_name, _publish_frequency));
  return client;
}

Client::Client(
    const std::string& _fleet_name,
    Duration _publish_frequency)
: fleet_name(_fleet_name),
  publish_frequency(_publish_frequency)
{
  ready = false;

  participant = dds_create_participant(0, NULL, NULL);
  if (participant < 0)
  {
    ROS_FATAL("couldn't create DDS participate: %s", 
        dds_strretcode(-participant));
    return;
  }

  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);

  rate.reset(new ros::Rate(1.0));

  if (!make_publish_handler(
      &FreeFleetData_RobotState_desc, 
      fleet_name + "/robot_state",
      robot_state_pub))
    return;
  // more publishers and subscribers to come

  ready = true;
}

Client::~Client()
{
  if (run_thread.joinable())
    run_thread.join();

  dds_delete_qos(qos);
  return_code = dds_delete(participant);
  if (return_code != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

bool Client::make_publish_handler(
    const dds_topic_descriptor_t* _descriptor,
    const std::string& _topic_name,
    PublishHandler& publish_handler)
{
  publish_handler.topic = dds_create_topic(
      participant, _descriptor, _topic_name.c_str(), NULL, NULL);
  if (publish_handler.topic < 0)
  {
    ROS_FATAL("couldn't create DDS topic: %s", 
        dds_strretcode(-publish_handler.topic));
    return false;
  }

  publish_handler.writer = dds_create_writer(
      participant, publish_handler.topic, qos, NULL);
  return true;
}

bool Client::is_ready()
{
  return ready;
}

bool Client::start(const FreeFleetData_RobotState& _state)
{
  if (!is_ready())
    return false;

  update_robot_state(_state);
  run_thread = std::thread(std::bind(&Client::run_thread_fn, this));
}

void Client::update_robot_state(const FreeFleetData_RobotState& _state)
{
  std::unique_lock<std::mutex> robot_state_lock(robot_state_mutex);
  robot_state = _state;
}

void Client::run_thread_fn()
{
  while (ros::ok())
  {
    ros::spinOnce();

    uint32_t status = 0;
    dds_get_status_changes(robot_state_pub.writer, &status);
    dds_sleepfor(DDS_MSECS(20));

    {
      std::unique_lock<std::mutex> robot_state_lock(robot_state_mutex);
      return_code = dds_write(robot_state_pub.writer, &robot_state);

      ROS_INFO("dds publishing: msg sec %d", robot_state.location.sec);

      if (return_code != DDS_RETCODE_OK)
      {
        ROS_ERROR("dds write failed: %s", dds_strretcode(-return_code));
        break;
      }
    }

    // TODO: getting commands

    rate->sleep();
  }
}

} // namespace free_fleet
