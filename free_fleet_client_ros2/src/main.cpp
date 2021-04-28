/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <iostream>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/impl/utils.h>

#include <nav2_util/robot_utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

class Client : public rclcpp::Node
{
private:
  const std::string global_frame = "map";
  const std::string robot_frame = "base_link";
  const double transform_tolerance = 1.0;

  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::shared_ptr<rclcpp::TimerBase> timer;

public:
  void get_pose()
  {
    geometry_msgs::msg::PoseStamped current_pose;
    nav2_util::getCurrentPose(
      current_pose, *tf, global_frame, robot_frame, transform_tolerance);

    RCLCPP_INFO(
      get_logger(),
      "x: %.2f, y: %.2f, Y: %.2f",
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      tf2::impl::getYaw(
        tf2::impl::toQuaternion(current_pose.pose.orientation)));
  }

  Client()
  : Node("free_fleet_client_ros2")
  {
    tf = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf->setCreateTimerInterface(timer_interface);
    tf->setUsingDedicatedThread(true);
    tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf, this, false);
  
    using namespace std::chrono_literals;
    timer = create_wall_timer(200ms, std::bind(&Client::get_pose, this));
  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Client>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
