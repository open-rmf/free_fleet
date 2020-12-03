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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__RELOCALIZATIONREQUEST_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__RELOCALIZATIONREQUEST_HPP

#include <string>

#include <free_fleet/messages/Location.hpp>

namespace free_fleet {
namespace messages {

struct RelocalizationRequest
{
  /// Robot to perform this request.
  std::string robot_name;

  /// Task ID issued by the fleet manager.
  uint32_t task_id;

  /// Location information to be used for relocalization.
  Location location;

  /// Last visited waypoint index for its navigation graph.
  std::size_t last_visited_waypoint_index;

  /// Comparing operator
  friend bool operator==(
    const RelocalizationRequest& lhs,
    const RelocalizationRequest& rhs)
  {
    if (lhs.robot_name == rhs.robot_name &&
      lhs.task_id == rhs.task_id &&
      lhs.location == rhs.location &&
      lhs.last_visited_waypoint_index == rhs.last_visited_waypoint_index)
      return true;
    return false;
  }
};

} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__RELOCALIZATIONREQUEST_HPP
