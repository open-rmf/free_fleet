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

#ifndef FREE_FLEET__ROS2__CLIENTNODE_HPP
#define FREE_FLEET__ROS2__CLIENTNODE_HPP

#include <deque>
#include <shared_mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/impl/utils.h>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav2_util/robot_utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <free_fleet/Client.hpp>
#include <free_fleet/ClientConfig.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

#include <free_fleet/Client.hpp>
#include <free_fleet/messages/Location.hpp>

#include "free_fleet/ros2/client_node_config.hpp"

namespace free_fleet
{
namespace ros2
{

class ClientNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ClientNode>;
  using Mutex = std::shared_mutex;
  using ReadLock = std::shared_lock<Mutex>;
  using WriteLock = std::unique_lock<Mutex>;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ClientNode() override;

  struct Fields
  {
    /// Free fleet client
    Client::SharedPtr client;

    // navigation2 action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr move_base_client;

    // Docker server client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_trigger_client;
  };

  void print_config();

private:
  // --------------------------------------------------------------------------
  // Battery handling

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr  battery_percent_sub;
  Mutex battery_state_mutex;
  sensor_msgs::msg::BatteryState current_battery_state;
  void battery_state_callback_fn(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // --------------------------------------------------------------------------
  // Robot pose handling

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  Mutex robot_pose_mutex;
  geometry_msgs::msg::PoseStamped current_robot_pose;
  geometry_msgs::msg::PoseStamped previous_robot_pose;

  bool get_robot_pose();

  // --------------------------------------------------------------------------
  // Mode handling

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  // TODO: figure out a better way to handle multiple triggered modes
  std::atomic<bool> request_error;
  std::atomic<bool> emergency;
  std::atomic<bool> paused;

  messages::RobotMode get_robot_mode();
  bool read_mode_request();

  // --------------------------------------------------------------------------
  // Path request handling

  bool read_path_request();

  // --------------------------------------------------------------------------
  // Destination request handling

  bool read_destination_request();

  // --------------------------------------------------------------------------
  // Task handling

  bool is_valid_request(
      const std::string& request_fleet_name,
      const std::string& request_robot_name,
      const std::string& request_task_id);

  Mutex task_id_mutex;
  std::string current_task_id;

  NavigateToPose::Goal location_to_nav_goal(
    const messages::Location& _location) const;

  struct Goal
  {
    std::string level_name;
    NavigateToPose::Goal goal;
    bool sent = false;
    uint32_t aborted_count = 0;
    rclcpp::Time goal_end_time;
  };

  Mutex goal_path_mutex;
  std::deque<Goal> goal_path;

  void read_requests();
  void handle_requests();
  void publish_robot_state();

  // --------------------------------------------------------------------------
  // publish and update functions and timers

  std::shared_ptr<rclcpp::TimerBase> update_timer;
  std::shared_ptr<rclcpp::TimerBase> publish_timer;
  void update_fn();
  void publish_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;
  Fields fields;

  void start(Fields fields);
};

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET__ROS2__CLIENTNODE_HPP