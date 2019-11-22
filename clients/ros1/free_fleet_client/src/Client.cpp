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
  tf2_listener(tf2_buffer),
  move_base_client(_config.move_base_action_name, true)
{
  ready = false;

  participant = dds_create_participant(client_config.dds_domain, NULL, NULL);
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

  if (!make_subscribe_handler(
      &FreeFleetData_Location_desc,
      client_config.fleet_name + "/location_command",
      location_command_sub))
    return;

  location_command_samples[0] = FreeFleetData_Location__alloc();

  // more publishers and subscribers to come

  /// setting up the move base action client, wait for server
  ros::Time t_0(ros::Time::now());
  ros::Time t_now(ros::Time::now());
  bool action_client_done = false;
  while ((t_now - t_0).toSec() < 10.0)
  {
    action_client_done = move_base_client.waitForServer(ros::Duration(0.5));
    if (action_client_done)
      break;

    ROS_WARN("waiting for connection with move base action server: %s",
        client_config.move_base_action_name.c_str());

    t_now = ros::Time::now();
    ros::Duration(1.0).sleep(); 
  }
  if (!action_client_done)
  {
    ROS_ERROR("timed out waiting for action server: %s",
        client_config.move_base_action_name.c_str());
    ready = false;
    return;
  }

  ready = true;
}

Client::~Client()
{
  if (run_thread.joinable())
  {
    run_thread.join();
    ROS_INFO("Client: run_thread joined.");
  }

  dds_delete_qos(qos);
  return_code = dds_delete(participant);
  if (return_code != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }

  WriteLock robot_state_lock(robot_state_mutex);
  dds_string_free(robot_state.name);
  dds_string_free(robot_state.model);
  FreeFleetData_Location_free(robot_state.path._buffer, DDS_FREE_ALL);
}

bool Client::make_publish_handler(
    const dds_topic_descriptor_t* _descriptor,
    const std::string& _topic_name,
    DDSPublishHandler& _publish_handler)
{
  _publish_handler.topic = dds_create_topic(
      participant, _descriptor, _topic_name.c_str(), NULL, NULL);
  if (_publish_handler.topic < 0)
  {
    ROS_FATAL("couldn't create DDS topic: %s", 
        dds_strretcode(-_publish_handler.topic));
    return false;
  }

  _publish_handler.writer = dds_create_writer(
      participant, _publish_handler.topic, qos, NULL);
  if (_publish_handler.writer < 0)
  {
    ROS_FATAL("couldn't create DDS writer: %s\n", 
        dds_strretcode(-_publish_handler.writer));
    return false;
  }

  return true;
}

bool Client::make_subscribe_handler(
    const dds_topic_descriptor_t* _descriptor,
    const std::string& _topic_name,
    DDSSubscribeHandler& _subscribe_handler)
{
  _subscribe_handler.topic = dds_create_topic(
      participant, _descriptor, _topic_name.c_str(), NULL, NULL);
  if (_subscribe_handler.topic < 0)
  {
    ROS_FATAL("couldn't create DDS topic: %s",
        dds_strretcode(-_subscribe_handler.topic));
    return false;
  }

  _subscribe_handler.reader = dds_create_reader(
      participant, _subscribe_handler.topic, qos, NULL);
  if (_subscribe_handler.reader < 0)
  {
    ROS_FATAL("couldn't create DDS reader: %s", 
        dds_strretcode(-_subscribe_handler.reader));
    return false;
  }
  
  return true;
}

bool Client::is_ready()
{
  return ready;
}

void Client::start()
{
  if (!is_ready())
  {
    ROS_ERROR("Client: is not ready, can't start.");
    return;
  }

  node.reset(new ros::NodeHandle(client_config.robot_name + "_node"));
  rate.reset(new ros::Rate(1.0));

  mode_sub = node->subscribe(
      client_config.mode_topic, 1,
      &Client::mode_callback_fn, this);

  battery_percent_sub = node->subscribe(
      client_config.battery_state_topic, 1,
      &Client::battery_state_callback_fn, this);

  level_name_sub = node->subscribe(
      client_config.level_name_topic, 1, 
      &Client::level_name_callback_fn, this);

  path_sub = node->subscribe(
      client_config.path_topic, 1, 
      &Client::path_callback_fn, this);

  FreeFleetData_RobotMode starting_mode
      {FreeFleetData_RobotMode_Constants_MODE_IDLE};
  {
    WriteLock robot_state_lock(robot_state_mutex);
    
    dds_string_free(robot_state.name);
    robot_state.name = dds_string_alloc_and_copy(client_config.robot_name);

    dds_string_free(robot_state.model);
    robot_state.model = dds_string_alloc_and_copy(client_config.robot_model);

    robot_state.mode = starting_mode;
  }

  ROS_INFO("Client: starting run thread.");
  run_thread = std::thread(std::bind(&Client::run_thread_fn, this));
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

void Client::mode_callback_fn(const free_fleet_msgs::RobotMode& _msg)
{
  FreeFleetData_RobotMode mode{_msg.mode};

  WriteLock robot_state_lock(robot_state_mutex);
  robot_state.mode = mode;
}

void Client::battery_state_callback_fn(const sensor_msgs::BatteryState& _msg)
{
  WriteLock robot_state_lock(robot_state_mutex);
  robot_state.battery_percent = _msg.percentage;
}

void Client::level_name_callback_fn(const std_msgs::String& _msg)
{
  WriteLock robot_state_lock(robot_state_mutex);
  dds_string_free(robot_state.location.level_name);
  robot_state.location.level_name = dds_string_alloc_and_copy(_msg.data);
}

void Client::path_callback_fn(const free_fleet_msgs::PathSequence& _msg)
{
  WriteLock robot_state_lock(robot_state_mutex);

  FreeFleetData_Location_free(robot_state.path._buffer, DDS_FREE_ALL);

  size_t path_size = _msg.path.size();
  robot_state.path._maximum = static_cast<uint32_t>(path_size);
  robot_state.path._length = static_cast<uint32_t>(path_size);
  robot_state.path._buffer = 
      FreeFleetData_RobotState_path_seq_allocbuf(path_size);
  robot_state.path._buffer = (FreeFleetData_Location*)dds_alloc(path_size);
  robot_state.path._release = false;

  for (size_t i = 0; i < path_size; ++i)
  {
    robot_state.path._buffer[i].sec = _msg.path[i].time.sec;
    robot_state.path._buffer[i].nanosec = _msg.path[i].time.nsec;
    robot_state.path._buffer[i].x = _msg.path[i].x;
    robot_state.path._buffer[i].y = _msg.path[i].y;
    robot_state.path._buffer[i].yaw = _msg.path[i].yaw;
    robot_state.path._buffer[i].level_name = 
        dds_string_alloc_and_copy(_msg.path[i].level_name);
  }
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

bool Client::read_commands()
{
  return_code = dds_read(
      location_command_sub.reader, location_command_samples, infos, 1, 1);
  if (return_code < 0)
  {
    ROS_FATAL("dds_read: %s", dds_strretcode(-return_code));
    return false;
  }

  if ((return_code > 0) && (infos[0].valid_data))
  {
    // TODO: probably make an action or service call here
    return true;
  }
  return false;
}

bool Client::send_commands()
{
  return false;
}

float Client::get_yaw_from_transform(
    const geometry_msgs::TransformStamped& _transform_stamped) const
{
  tf2::Quaternion tf2_quat;
  fromMsg(_transform_stamped.transform.rotation, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat);

  // ignores pitch and roll, but the api call is so nice though
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
    rate->sleep();
    ros::spinOnce();

    /// Update the robot's current known pose and tries to publish the state
    /// over DDS
    if (!get_robot_transform() || !publish_robot_state())
      continue;

    if (!read_commands() || !send_commands())
      continue;
  }
}

char* Client::dds_string_alloc_and_copy(const std::string& _str)
{
  char* ptr = dds_string_alloc(_str.length());
  for (size_t i = 0; i < _str.length(); ++i)
  {
    ptr[i] = _str[i];
  }
  return ptr;
}

} // namespace free_fleet
