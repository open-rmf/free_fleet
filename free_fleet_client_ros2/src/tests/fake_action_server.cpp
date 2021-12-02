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

#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "handle_goal_fn"), "Received goal request with frame_id %s",
    goal->pose.header.frame_id.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("handle_cancel_fn"), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "execute_fn"), "executing goal: send feedback for 5s to simulate work");
  auto clock = rclcpp::Clock(RCL_STEADY_TIME);
  auto start = clock.now();
  // do lots of awesome groundbreaking robot stuff here
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();

  for (int i = 0; (i < 5) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("execute_fn"), "Goal canceled");
      return;
    }
    // Update navigation time
    feedback->navigation_time = clock.now() - start;
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("execute_fn"), "Publish feedback");
    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("execute_fn"), "Goal succeeded");
  }
}

void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("handle_accepted_fn"), "starting execution thread");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(execute, _1), goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fake_nav2_action_server");
  using namespace std::placeholders;  // for _1, _2, _3...
  auto action_server = rclcpp_action::create_server<NavigateToPose>(
    node, "navigate_to_pose_fake",
    std::bind(handle_goal, _1, _2),
    std::bind(handle_cancel, _1),
    std::bind(handle_accepted, _1));
  rclcpp::spin(node);

  // Cleanup and exit
  rclcpp::shutdown();
  return 0;
}
