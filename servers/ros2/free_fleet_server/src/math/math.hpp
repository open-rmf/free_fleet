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

#ifndef FREEFLEETSERVER__SRC__MATH__MATH_HPP
#define FREEFLEETSERVER__SRC__MATH__MATH_HPP

#include <Eigen/Geometry>

#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>

namespace free_fleet
{
namespace math
{

Eigen::Matrix3d convert(const std::array<double, 9>& data);

void transform_location(
    const Eigen::Matrix3d& trasform, 
    double transform_yaw,
    rmf_fleet_msgs::msg::Location& location);

void transform_robot_state(
    const Eigen::Matrix3d& transform, 
    double transform_yaw,
    rmf_fleet_msgs::msg::RobotState& robot_state);

} // namespace math
} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__MATH__MATH_HPP
