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

  state_pub.reset(
      new dds::DDSPublishHandler<FreeFleetData_RobotState>(
          participant, &FreeFleetData_RobotState_desc,
          client_config.dds_state_topic));
  if (!state_pub->is_ready())
    return;

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
  if (update_thread.joinable())
  {
    update_thread.join();
    ROS_INFO("Client: update_thread joined.");
  }

  if (publish_thread.joinable())
  {
    publish_thread.join();
    ROS_INFO("Client: publish_thread joined.");
  }

  return_code = dds_delete(participant);
  if (return_code != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
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
  update_rate.reset(new ros::Rate(client_config.update_frequency));
  publish_rate.reset(new ros::Rate(client_config.publish_frequency));

  battery_percent_sub = node->subscribe(
      client_config.battery_state_topic, 1,
      &Client::battery_state_callback_fn, this);

  level_name_sub = node->subscribe(
      client_config.level_name_topic, 1, 
      &Client::level_name_callback_fn, this);

  emergency = false;
  paused = false;

  ROS_INFO("Client: starting update thread.");
  update_thread = std::thread(std::bind(&Client::update_thread_fn, this));

  ROS_INFO("Client: starting publish thread.");
  publish_thread = std::thread(std::bind(&Client::publish_thread_fn, this));
}

void Client::battery_state_callback_fn(const sensor_msgs::BatteryState& _msg)
{
  WriteLock battery_state_lock(battery_state_mutex);
  current_battery_state = _msg;
}

void Client::level_name_callback_fn(const std_msgs::String& _msg)
{
  WriteLock level_name_lock(level_name_mutex);
  current_level_name = _msg;
}

bool Client::get_robot_transform()
{
  try {
    geometry_msgs::TransformStamped tmp_transform_stamped = 
        tf2_buffer.lookupTransform(
            client_config.robot_frame, 
            client_config.map_frame, 
            ros::Time(0));
    WriteLock robot_transform_lock(robot_transform_mutex);
    previous_robot_transform = current_robot_transform;
    current_robot_transform = tmp_transform_stamped;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}

uint32_t Client::get_robot_mode()
{
  if (emergency)
    return FreeFleetData_RobotMode_Constants_MODE_EMERGENCY;

  {
    ReadLock battery_state_lock(battery_state_mutex);
    ReadLock robot_transform_lock(robot_transform_mutex);

    /// Checks if the robot is charging
    if (current_battery_state.power_supply_status == 
        current_battery_state.POWER_SUPPLY_STATUS_CHARGING)
      return FreeFleetData_RobotMode_Constants_MODE_CHARGING;
    
    /// Checks if the robot is moving
    else if(!is_transform_close(
        current_robot_transform, previous_robot_transform))
      return FreeFleetData_RobotMode_Constants_MODE_MOVING;
  }
  
  /// Otherwise, robot is neither charging nor moving,
  /// Checks if the robot is paused
  if (paused)
    return FreeFleetData_RobotMode_Constants_MODE_PAUSED;

  /// Otherwise, robot has queued tasks, it is paused or waiting,
  /// default to use pausing for now
  else 
    return FreeFleetData_RobotMode_Constants_MODE_IDLE;
}

void Client::publish_robot_state()
{
  FreeFleetData_RobotState* current_robot_state = 
      FreeFleetData_RobotState__alloc();

  dds_string_free(current_robot_state->name);
  current_robot_state->name = 
      dds_string_alloc_and_copy(client_config.robot_name);

  dds_string_free(current_robot_state->model);
  current_robot_state->model = 
      dds_string_alloc_and_copy(client_config.robot_model);

  current_robot_state->mode.mode = get_robot_mode();
  
  {
    ReadLock battery_state_lock(battery_state_mutex);
    current_robot_state->battery_percent = current_battery_state.percentage;
  }

  {
    ReadLock robot_transform_lock(robot_transform_mutex);
    current_robot_state->location.sec = current_robot_transform.header.stamp.sec;
    current_robot_state->location.nanosec = 
        current_robot_transform.header.stamp.nsec;
    current_robot_state->location.x = 
        current_robot_transform.transform.translation.x;
    current_robot_state->location.y = 
        current_robot_transform.transform.translation.y;
    current_robot_state->location.yaw = 
        get_yaw_from_transform(current_robot_transform);
    
    ReadLock level_name_lock(level_name_mutex);
    dds_string_free(current_robot_state->location.level_name);
    current_robot_state->location.level_name = 
        dds_string_alloc_and_copy(current_level_name.data);
  }

  {
    ReadLock goal_path_lock(goal_path_mutex);
    uint32_t n_goals = static_cast<uint32_t>(goal_path.size());

    current_robot_state->path._maximum = n_goals;
    current_robot_state->path._length = n_goals;
    current_robot_state->path._buffer = 
        FreeFleetData_RobotState_path_seq_allocbuf(n_goals);
    current_robot_state->path._release = false;

    for (uint32_t i = 0; i < n_goals; ++i)
    {
      current_robot_state->path._buffer[i].sec =
          goal_path[i].goal.target_pose.header.stamp.sec;
      current_robot_state->path._buffer[i].nanosec =
          goal_path[i].goal.target_pose.header.stamp.nsec;
      current_robot_state->path._buffer[i].x =
          goal_path[i].goal.target_pose.pose.position.x;
      current_robot_state->path._buffer[i].y =
          goal_path[i].goal.target_pose.pose.position.y;
      current_robot_state->path._buffer[i].yaw =
          get_yaw_from_quat(goal_path[i].goal.target_pose.pose.orientation);
      current_robot_state->path._buffer[i].level_name =
          dds_string_alloc_and_copy(goal_path[i].level_name);
    }
  }

  if (state_pub->write(current_robot_state))
    ROS_INFO("dds publishing: msg sec %u", current_robot_state->location.sec);

  FreeFleetData_RobotState_free(current_robot_state, DDS_FREE_ALL);
}


move_base_msgs::MoveBaseGoal Client::location_to_goal(
    std::shared_ptr<const FreeFleetData_Location> _location) const
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = client_config.map_frame;
  goal.target_pose.header.stamp.sec = _location->sec;
  goal.target_pose.header.stamp.nsec = _location->nanosec;
  goal.target_pose.pose.position.x = _location->x;
  goal.target_pose.pose.position.y = _location->y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = get_quat_from_yaw(_location->yaw);
  return goal;
}

move_base_msgs::MoveBaseGoal Client::location_to_goal(
    const FreeFleetData_Location& _location) const
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = client_config.map_frame;
  goal.target_pose.header.stamp.sec = _location.sec;
  goal.target_pose.header.stamp.nsec = _location.nanosec;
  goal.target_pose.pose.position.x = _location.x;
  goal.target_pose.pose.position.y = _location.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = get_quat_from_yaw(_location.yaw);
  return goal;
}

void Client::pause_robot()
{
  if (paused)
    return;

  move_base_client.cancelAllGoals();
  WriteLock goal_path_lock(goal_path_mutex);
  if (!goal_path.empty())
    goal_path[0].sent = false;
  
  paused = true;
}

void Client::resume_robot()
{
  if (!paused)
    return;
  paused = false;
}

void Client::read_commands()
{
  auto mode_msg = mode_command_sub->read();
  if (mode_msg)
  {
    if (mode_msg->mode == FreeFleetData_RobotMode_Constants_MODE_PAUSED)
    {
      ROS_INFO("received a PAUSE command.");
      pause_robot();
    }
    else if (mode_msg->mode == FreeFleetData_RobotMode_Constants_MODE_MOVING)
    {
      ROS_INFO("received a RESUME command.");
      resume_robot();
    }
    else if (
        mode_msg->mode == FreeFleetData_RobotMode_Constants_MODE_EMERGENCY)
    {
      ROS_INFO("received an EMERGENCY command.");
      paused = false;
      emergency = true;
    }
    return;
  }

  auto path_msg = path_command_sub->read();
  if (path_msg)
  {
    ROS_INFO("received a Path command.");

    WriteLock goal_path_lock(goal_path_mutex);
    goal_path.clear();
    for (int i = 0; i < path_msg->path._length; ++i)
    {
      Goal new_goal { 
        std::string(path_msg->path._buffer[i].level_name),
        location_to_goal(path_msg->path._buffer[i]),
        false
      };
      goal_path.push_back(new_goal);
    }
    return;
  }

  auto location_msg = location_command_sub->read();
  if (location_msg)
  {
    ROS_INFO("received a Location command.");

    WriteLock goal_path_lock(goal_path_mutex);
    goal_path.clear();
    Goal new_goal {
      std::string(location_msg->level_name),
      location_to_goal(location_msg),
      false
    };
    goal_path.push_back(new_goal);
  }
}

void Client::handle_commands()
{
  // nothing works if the move_base_client is not connected!
  if (!move_base_client.isServerConnected())
  {
    ROS_WARN("lost connection to MoveBaseAction server.");
    return;
  }

  // if there is an emergency or the robot is paused
  if (emergency || paused)
    return;

  // ooohh we have goals!
  WriteLock goal_path_lock(goal_path_mutex);
  if (!goal_path.empty())
  {
    // Goals must have been updated since last handling, execute them now
    if (!goal_path.front().sent)
    {
      ROS_INFO("sending next goal.");
      move_base_client.sendGoal(goal_path.front().goal);
      goal_path.front().sent = true;
      return;
    }

    // Goals have been sent, check the goal states now
    GoalState current_goal_state = move_base_client.getState();
    if (current_goal_state == GoalState::SUCCEEDED)
    {
      ROS_INFO("current goal state: SUCCEEEDED.");
      goal_path.pop_front();
      return;
    }
    else if (current_goal_state == GoalState::ACTIVE)
    {
      return;
    }
    else
    {
      ROS_INFO(
          "current goal state: %s", current_goal_state.toString().c_str());
      ROS_INFO("Client: no idea what to do now, doh!");
      // emergency = true;
    }
  }

  // otherwise, mode is correct, nothing in queue, nothing else to do then
}

double Client::get_yaw_from_quat(
    const geometry_msgs::Quaternion& _quat) const
{
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(_quat, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat);
  
  // ignores pitch and roll, but the api call is so nice though
  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

double Client::get_yaw_from_transform(
    const geometry_msgs::TransformStamped& _transform_stamped) const
{
  return get_yaw_from_quat(_transform_stamped.transform.rotation);
}

geometry_msgs::Quaternion Client::get_quat_from_yaw(double _yaw) const
{
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, _yaw);
  quat_tf.normalize();

  geometry_msgs::Quaternion quat = tf2::toMsg(quat_tf);
  return quat;
}

bool Client::is_transform_close(
    const geometry_msgs::TransformStamped& _first,
    const geometry_msgs::TransformStamped& _second) const
{
  if (_first.header.frame_id != _second.header.frame_id || 
      _first.child_frame_id != _second.child_frame_id)
    return false;

  double elapsed_sec = (_second.header.stamp - _first.header.stamp).toSec();
  tf2::Vector3 first_pos;
  tf2::Vector3 second_pos;
  tf2::fromMsg(_first.transform.translation, first_pos);
  tf2::fromMsg(_second.transform.translation, second_pos);
  double distance = second_pos.distance(first_pos);
  double speed = abs(distance / elapsed_sec);
  if (speed > 0.01)
    return false;

  double first_yaw = get_yaw_from_transform(_first);
  double second_yaw = get_yaw_from_transform(_second);
  double turning_speed = abs((second_yaw - first_yaw) / elapsed_sec);
  if (turning_speed > 0.01)
    return false;

  return true;
}

void Client::update_thread_fn()
{
  while (node->ok())
  {
    update_rate->sleep();
    ros::spinOnce();

    /// Update the robot's current known pose
    get_robot_transform();

    /// Tries to accept any commands over DDS, figures out the priority of the
    /// incoming commands and executes them.
    read_commands();
    handle_commands();
  }
}

void Client::publish_thread_fn()
{
  while (node->ok())
  {
    publish_rate->sleep();
    publish_robot_state();
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
