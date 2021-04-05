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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP

#include <string>
#include <cstdint>

namespace free_fleet {
namespace messages {

struct Location
{
  /// Time in seconds and nanoseconds.
  int32_t sec;
  uint32_t nanosec;

  /// x, y positions in meters and yaw in radians.
  double x;
  double y;
  double yaw;

  /// Current level/map name.
  std::string level_name;

  /// Comparing operator
  friend bool operator==(
    const Location& lhs,
    const Location& rhs)
  {
    if (lhs.sec == rhs.sec &&
      lhs.nanosec == rhs.nanosec &&
      abs(lhs.x - rhs.x) < 1e-3 &&
      abs(lhs.y - rhs.y) < 1e-3 &&
      abs(lhs.yaw - rhs.yaw) < 1e-3 &&
      lhs.level_name == rhs.level_name)
      return true;
    return false;
  }
};

} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP
