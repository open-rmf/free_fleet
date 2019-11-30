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
  move_base_client(_config.move_base_server_name, true)
{
  ready = false;

  participant = dds_create_participant(
      static_cast<dds_domainid_t>(client_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    ROS_FATAL("couldn't create DDS participate: %s", 
        dds_strretcode(-participant));
    return;
  }

  /// -------------------------------------------------------------------------
  /// create all the dds stuff needed for sending out the robot states

  std::string state_dds_topic_name = client_config.dds_state_topic;
  state_topic = dds_create_topic(
      participant, &FreeFleetData_RobotState_desc, 
      state_dds_topic_name.c_str(), NULL, NULL);
  if (state_topic < 0)
  {
    ROS_FATAL("dds_create_topic: %s\n", dds_strretcode(-state_topic));
    return;
  }

  dds_qos_t* state_qos = dds_create_qos();
  dds_qset_reliability(state_qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  state_writer = dds_create_writer(
      participant, state_topic, state_qos, NULL);
  if (state_writer < 0)
  {
    ROS_FATAL("dds_create_writer: %s\n", dds_strretcode(-state_writer));
    return;
  }
  dds_delete_qos(state_qos);

  /// -------------------------------------------------------------------------
  /// create all the dds stuff needed for getting mode commands

  mode_command_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_RobotMode>(
          participant, &FreeFleetData_RobotMode_desc, 
          client_config.dds_mode_command_topic));
  if (!mode_command_sub->is_ready())
    return;
  
  /// -------------------------------------------------------------------------
  /// create all the dds stuff needed for getting location commands

  location_command_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_Location>(
          participant, &FreeFleetData_Location_desc, 
          client_config.dds_location_command_topic));
  if (!location_command_sub->is_ready())
    return;

  /// -------------------------------------------------------------------------
  /// create all the dds stuff needed for getting path commands

  path_command_sub.reset(
      new dds::DDSSubscribeHandler<FreeFleetData_Path>(
          participant, &FreeFleetData_Path_desc,
          client_config.dds_path_command_topic));
  if (!path_command_sub->is_ready())
    return;

  /// -------------------------------------------------------------------------
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
        client_config.move_base_server_name.c_str());

    t_now = ros::Time::now();
    ros::Duration(1.0).sleep(); 
  }
  if (!action_client_done)
  {
    ROS_ERROR("timed out waiting for action server: %s",
        client_config.move_base_server_name.c_str());
    ready = false;
    return;
  }

  goal_path.clear();

  ready = true;
}

Client::~Client()
{
  if (run_thread.joinable())
  {
    run_thread.join();
    ROS_INFO("Client: run_thread joined.");
  }

  return_code = dds_delete(participant);
  if (return_code != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }

  WriteLock robot_state_lock(robot_state_mutex);
  dds_string_free(robot_state.name);
  dds_string_free(robot_state.model);
  FreeFleetData_Location_free(robot_state.path._buffer, DDS_FREE_ALL);

  // FreeFleetData_Location_free(location_command_samples[0], DDS_FREE_ALL);
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
  rate.reset(new ros::Rate(client_config.operate_frequency));

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

  last_write_time = ros::Time::now();

  ROS_INFO("Client: starting run thread.");
  run_thread = std::thread(std::bind(&Client::run_thread_fn, this));
}

void Client::battery_state_callback_fn(const sensor_msgs::BatteryState& _msg)
{
  {
    WriteLock battery_state_lock(battery_state_mutex);
    battery_state = _msg;
  }
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

bool Client::get_robot_transform()
{
  try {
    geometry_msgs::TransformStamped tmp_transform_stamped = 
        tf2_buffer.lookupTransform(
            client_config.robot_frame, 
            client_config.map_frame, 
            ros::Time(0));
    prev_robot_transform_stamped = robot_transform_stamped;
    robot_transform_stamped = tmp_transform_stamped;
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

void Client::get_robot_mode()
{
  FreeFleetData_RobotMode robot_mode;

  /// Checks if the robot is charging
  ReadLock battery_state_lock(battery_state_mutex);
  if (battery_state.power_supply_status == 
      battery_state.POWER_SUPPLY_STATUS_CHARGING)
    robot_mode.mode = FreeFleetData_RobotMode_Constants_MODE_CHARGING;
  
  /// Checks if the robot is moving
  else if(!is_transform_close(
      robot_transform_stamped, prev_robot_transform_stamped))
    robot_mode.mode = FreeFleetData_RobotMode_Constants_MODE_MOVING;
  
  /// Otherwise, robot is neither charging nor moving,
  else
  {
    /// Checks if the robot has tasks, and is just paused
    if (!goal_path.empty())
      robot_mode.mode = FreeFleetData_RobotMode_Constants_MODE_PAUSED;

    /// Otherwise, robot has queued tasks, it is paused or waiting,
    /// default to use pausing for now
    else 
      robot_mode.mode = FreeFleetData_RobotMode_Constants_MODE_IDLE;
  }

  WriteLock robot_state_lock(robot_state_mutex);
  robot_state.mode = robot_mode;
}

void Client::publish_robot_state()
{
  get_robot_mode();

  ReadLock robot_state_lock(robot_state_mutex);
  return_code = dds_write(state_writer, &robot_state);

  ROS_INFO("dds publishing: msg sec %d", robot_state.location.sec);

  if (return_code != DDS_RETCODE_OK)
    ROS_ERROR("dds write failed: %s", dds_strretcode(-return_code));
}

bool Client::read_mode_commands()
{
  return false;
}

bool Client::read_location_commands()
{
  auto msg = location_command_sub->read();
  if (msg == nullptr)
    return false;

  location_command_goal.target_pose.header.frame_id = client_config.map_frame;
  location_command_goal.target_pose.header.stamp.sec = msg->sec;
  location_command_goal.target_pose.header.stamp.nsec = msg->nanosec;
  location_command_goal.target_pose.pose.position.x = msg->x;
  location_command_goal.target_pose.pose.position.y = msg->y;
  location_command_goal.target_pose.pose.position.z = 0.0;
  location_command_goal.target_pose.pose.orientation = 
      get_quat_from_yaw(msg->yaw);
  
  ROS_INFO(
      "received command: %d sec %d nanosec", msg->sec, msg->nanosec);
  return true;
}

bool Client::read_path_commands()
{
  return false;
}

void Client::handle_commands()
{
  if (!move_base_client.isServerConnected())
  {
    ROS_WARN("MoveBaseAction server is not connected.");
    return;
  }
  move_base_client.sendGoal(location_command_goal);
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

geometry_msgs::Quaternion Client::get_quat_from_yaw(float _yaw) const
{
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, _yaw);
  quat_tf.normalize();

  geometry_msgs::Quaternion quat = tf2::toMsg(quat_tf);
  return quat;
}

bool Client::is_close(double x, double y, double min) const
{
  if (abs(x - y) > min)
    return false;
  return true;
} 

bool Client::is_transform_close(
    const geometry_msgs::TransformStamped& _t1,
    const geometry_msgs::TransformStamped& _t2) const
{
  if (_t1.header.frame_id == _t2.header.frame_id && 
      _t1.child_frame_id == _t2.child_frame_id &&
      is_close(_t1.transform.translation.x, _t2.transform.translation.x) &&
      is_close(_t1.transform.translation.y, _t2.transform.translation.y) &&
      is_close(_t1.transform.translation.z, _t2.transform.translation.z) &&
      is_close(_t1.transform.rotation.x, _t2.transform.rotation.x) &&
      is_close(_t1.transform.rotation.y, _t2.transform.rotation.y) &&
      is_close(_t1.transform.rotation.z, _t2.transform.rotation.z) &&
      is_close(_t1.transform.rotation.w, _t2.transform.rotation.w))
    return true;
  return false;      
}

void Client::run_thread_fn()
{
  while (node->ok())
  {
    rate->sleep();
    ros::spinOnce();

    /// Update the robot's current known pose and tries to publish the state
    /// over DDS
    double elapsed_write_seconds = 
        ros::Time::now().toSec() - last_write_time.toSec();
    if (elapsed_write_seconds > client_config.state_publish_frequency &&
        get_robot_transform())
    {
      publish_robot_state();
      last_write_time = ros::Time::now();
    }

    /// Tries to accept any commands over DDS, and sends out robot commands
    /// using MoveBaseAction
    if (read_location_commands())
      handle_commands();
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
