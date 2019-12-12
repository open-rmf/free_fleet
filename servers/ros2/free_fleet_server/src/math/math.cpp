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

#include "math.hpp"

namespace free_fleet
{
namespace math
{

Eigen::Matrix3d convert(const std::array<double, 9>& _data)
{
  Eigen::Matrix3d mat;
  mat << _data[0], _data[1], _data[2],
      _data[3], _data[4], _data[5],
      _data[6], _data[7], _data[8];
  return mat;
}

void transform_location(
    const Eigen::Matrix3d& _transform, 
    double _yaw_transform,
    rmf_fleet_msgs::msg::Location& _location)
{
  Eigen::Vector3d pos(_location.x, _location.y, 1.0);
  Eigen::Vector3d transformed_pos = _transform * pos;
  _location.x = transformed_pos[0] / transformed_pos[2];
  _location.y = transformed_pos[1] / transformed_pos[2];

  _location.yaw += _yaw_transform;
}

void transform_robot_state(
    const Eigen::Matrix3d& _transform,
    double _yaw_transform, 
    rmf_fleet_msgs::msg::RobotState& _robot_state)
{
  transform_location(_transform, _yaw_transform, _robot_state.location);
}

} // namespace math
} // namespace free_fleet
