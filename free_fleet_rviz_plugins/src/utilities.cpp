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

#include <cstdlib>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utilities.hpp"

namespace free_fleet {

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

std::string generate_random_task_id(size_t length)
{
  auto randchar = []() -> char
  {
      const char charset[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
      const size_t max_index = (sizeof(charset) - 1);
      return charset[ rand() % max_index ];
  };
  std::string str(length,0);
  std::generate_n( str.begin(), length, randchar );
  return str;
}

} // namespace free_fleet
