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
  /// Robot to perform this request.
  std::string robot_name;

  /// Task ID issued by the fleet manager.
  uint32_t task_id;

  /// Desired mode for this robot.
  RobotMode mode;

  /// Additional parameters if any when parsing the desired mode.
  std::vector<ModeParameter> parameters;

  /// Comparing operator
  friend bool operator==(
    const ModeRequest& lhs,
    const ModeRequest& rhs)
  {
    if (lhs.robot_name == rhs.robot_name &&
      lhs.task_id == rhs.task_id &&
      lhs.mode == rhs.mode &&
      lhs.parameters.size() == rhs.parameters.size())
    {
      for (std::size_t i = 0; i < lhs.parameters.size(); ++i)
      {
        if (lhs.parameters[i] == rhs.parameters[i])
          continue;
        else
          return false;
      }
      return true;
    }
    return false;
  }
};

} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__MODEREQUEST_HPP
