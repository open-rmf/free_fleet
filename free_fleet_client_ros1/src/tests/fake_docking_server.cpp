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

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

bool trigger(
  std_srvs::Trigger::Request& req,
  std_srvs::Trigger::Response& res)
{
  res.success = true;
  ROS_INFO("Docking trigger received.");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_docking_server");
  ros::NodeHandle n;

  ros::ServiceServer service =
    n.advertiseService("fake_docking_server", trigger);
  ros::spin();
  return 0;
}

