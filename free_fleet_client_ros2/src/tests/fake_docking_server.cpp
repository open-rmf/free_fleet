/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

void trigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  RCLCPP_INFO(rclcpp::get_logger("trigger_fn"), "Docking trigger received.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fake_docking_server");
  using namespace std::placeholders;  // for _1, _2, _3...
  auto service = node->create_service<std_srvs::srv::Trigger>("dock_fake", std::bind(trigger, _1, _2));
  rclcpp::spin(node);
  
  // Cleanup and exit
  rclcpp::shutdown();
  return 0;
}
