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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP

#include <string>
#include <vector>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet {
namespace messages {

struct RobotState
{
  /// Name of the robot.
  std::string name;

  /// Model of the robot.
  std::string model;

  /// Task ID of the task it is currently performing.
  uint32_t task_id;

  /// Current mode of the robot.
  RobotMode mode;

  /// Current battery percentage, values from 0 to 1.
  double battery_percent;

  /// Current location of the robot.
  Location location;

  /// Index of the most recently past 
  uint32_t path_target_index;
};

} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP
