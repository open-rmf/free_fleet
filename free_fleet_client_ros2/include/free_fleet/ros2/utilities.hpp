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

#ifndef FREE_FLEET__ROS2__UTILITIES_HPP
#define FREE_FLEET__ROS2__UTILITIES_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace free_fleet
{
namespace ros2
{

double get_yaw_from_pose(
    const geometry_msgs::msg::PoseStamped& pose_stamped); 

geometry_msgs::msg::Quaternion get_quat_from_yaw(double _yaw);

bool is_pose_close(
    const geometry_msgs::msg::PoseStamped& pose_1,
    const geometry_msgs::msg::PoseStamped& pose_2);

} // namespace ros2
} // namespace free_fleet

#endif // FREE_FLEET__ROS2__UTILITIES_HPP