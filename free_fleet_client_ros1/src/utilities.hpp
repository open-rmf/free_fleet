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

#ifndef FREE_FLEET_CLIENT_ROS1__SRC__UTILITIES_HPP
#define FREE_FLEET_CLIENT_ROS1__SRC__UTILITIES_HPP

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace free_fleet
{
namespace ros1
{

double get_yaw_from_quat(const geometry_msgs::Quaternion& quat);

double get_yaw_from_transform(
    const geometry_msgs::TransformStamped& transform_stamped); 

geometry_msgs::Quaternion get_quat_from_yaw(double yaw);

bool is_transform_close(
    const geometry_msgs::TransformStamped& transform_1,
    const geometry_msgs::TransformStamped& transform_2);

} // namespace ros1
} // namespace free_fleet

#endif // FREE_FLEET_CLIENT_ROS1__SRC__UTILITIES_HPP
