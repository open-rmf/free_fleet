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

#include "utilities.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace free_fleet
{
namespace ros1
{

double get_yaw_from_quat(const geometry_msgs::Quaternion& _quat)
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

double get_yaw_from_transform(
    const geometry_msgs::TransformStamped& _transform_stamped)
{
  return get_yaw_from_quat(_transform_stamped.transform.rotation);
}

geometry_msgs::Quaternion get_quat_from_yaw(double _yaw)
{
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, _yaw);
  quat_tf.normalize();

  geometry_msgs::Quaternion quat = tf2::toMsg(quat_tf);
  return quat;
}

bool is_transform_close(
    const geometry_msgs::TransformStamped& _first,
    const geometry_msgs::TransformStamped& _second)
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

} // namespace ros1
} // namespace free_fleet
