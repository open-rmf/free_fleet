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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__MODEREQUEST_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__MODEREQUEST_HPP

#include <string>
#include <vector>

#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/ModeParameter.hpp>

namespace free_fleet {
namespace messages {

struct ModeRequest
{
  /// Robot to perform this request
  std::string robot_name;

  /// Unique task ID issued by the fleet manager
  std::string task_id;

  /// Desired mode for this robot
  RobotMode mode;

  /// Additional parameters if any when parsing the desired mode
  std::vector<ModeParameter> parameters;
};

} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__MODEREQUEST_HPP
