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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace free_fleet
{

std::shared_ptr<Client> Client::make(const ClientConfig& _config)
{
  std::shared_ptr<Client> client(new Client(_config));
  return client;
}

Client::Client(const ClientConfig& _config)
: client_config(_config),
  tf2_listener(tf2_buffer)
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

  if (!make_publish_handler(
      &FreeFleetData_RobotState_desc, 
      client_config.fleet_name + "/robot_state",
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

  dds_return_t rc = 
      dds_set_status_mask(
          publish_handler.writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));
    return false;
  }
  return true;
}

bool Client::is_ready()
{
  return ready && robot_state_pub.is_ok();
}

bool Client::start(const FreeFleetData_RobotState& _state)
{
  if (!is_ready())
    return false;

  robot_name = _state.name;
  node.reset(new ros::NodeHandle(robot_name + "_node"));
  rate.reset(new ros::Rate(1.0));

  battery_percent_sub = node->subscribe(
      client_config.battery_state_topic, 1,
      &Client::battery_state_callback_fn, this);

  level_name_sub = node->subscribe(
      client_config.level_name_topic, 1, 
      &Client::level_name_callback_fn, this);

  {
    WriteLock robot_state_lock(robot_state_mutex);
    robot_state = _state;
  }

  run_thread = std::thread(std::bind(&Client::run_thread_fn, this));
  return true;
}

bool Client::PublishHandler::is_ok()
{
  uint32_t status;
  dds_return_t rc = dds_get_status_changes(writer, &status);
  if (rc != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));
    return false;
  }

  if (!(status & DDS_PUBLICATION_MATCHED_STATUS))
    return false;
  return true;
}

bool Client::get_robot_transform()
{
  try {
    robot_transform_stamped = tf2_buffer.lookupTransform(
        client_config.target_frame, 
        client_config.map_frame, 
        ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  WriteLock robot_state_lock(robot_state_mutex);
  robot_state.location.sec = robot_transform_stamped.header.stamp.sec;
  robot_state.location.nanosec = robot_transform_stamped.header.stamp.nsec;
  robot_state.location.x = robot_transform_stamped.transform.translation.x;
  robot_state.location.y = robot_transform_stamped.transform.translation.y;
  robot_state.location.yaw = get_yaw_from_transform(robot_transform_stamped);
  return true;
}

void Client::battery_state_callback_fn(const sensor_msgs::BatteryState& _msg)
{
  WriteLock robot_state_lock(robot_state_mutex);
  robot_state.battery_percent = _msg.percentage;
}

void Client::level_name_callback_fn(const std_msgs::String& _msg)
{
  std::string prev_level_name;

  {
    ReadLock robot_state_lock(robot_state_mutex);
    prev_level_name = robot_state.location.level_name;
  }

  // if its the same level, or a new transform hasn't come in, we ignore
  if (_msg.data == prev_level_name || !get_robot_transform())
    return;

  WriteLock robot_state_lock(robot_state_mutex);
  std::copy(
      _msg.data.begin(), 
      _msg.data.end(), 
      robot_state.location.level_name);
}

bool Client::publish_robot_state()
{
  ReadLock robot_state_lock(robot_state_mutex);
  return_code = dds_write(robot_state_pub.writer, &robot_state);

  ROS_INFO("dds publishing: msg sec %d", robot_state.location.sec);

  if (return_code != DDS_RETCODE_OK)
  {
    ROS_ERROR("dds write failed: %s", dds_strretcode(-return_code));
    return false;
  }
  return true;
}

float Client::get_yaw_from_transform(
    const geometry_msgs::TransformStamped& _transform_stamped) const
{
  tf2::Quaternion tf2_quat;
  fromMsg(_transform_stamped.transform.rotation, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat);

  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

void Client::run_thread_fn()
{
  while (node->ok())
  {
    ros::spinOnce();

    // Update the robot's current known pose
    if (!get_robot_transform())
      continue;
    
    // Publish the updated robot state
    if (!publish_robot_state())
      continue;

    // TODO: getting commands

    rate->sleep();
  }
}

} // namespace free_fleet
