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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP

#include <cstdint>

namespace free_fleet {
namespace messages {

struct RobotMode
{
  uint32_t mode;
  static const uint32_t MODE_IDLE = 0;
  static const uint32_t MODE_CHARGING = 1;
  static const uint32_t MODE_MOVING = 2;
  static const uint32_t MODE_PAUSED = 3;
  static const uint32_t MODE_WAITING = 4;
  static const uint32_t MODE_EMERGENCY = 5;
  static const uint32_t MODE_GOING_HOME = 6;
  static const uint32_t MODE_DOCKING = 7;
  static const uint32_t MODE_REQUEST_ERROR = 8;
};

} // namespace messages
} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP
