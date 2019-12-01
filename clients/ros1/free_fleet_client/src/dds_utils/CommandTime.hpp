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

#ifndef FREEFLEETCLIENT__SRC__DDS_UTILS__COMMANDTIME_HPP
#define FREEFLEETCLIENT__SRC__DDS_UTILS__COMMANDTIME_HPP

#include "../free_fleet/FreeFleet.h"

namespace free_fleet
{
namespace utils
{

struct CommandTime
{

  int32_t sec;

  uint32_t nanosec;

  CommandTime(const FreeFleetData_Location& location) :
    sec(location.sec),
    nanosec(location.nanosec)
  {}

  CommandTime(const FreeFleetData_Path& path)
  {
    if (path.path._length > 0)
    {
      sec = path.path._buffer[0].sec;
      nanosec = path.path._buffer[0].nanosec;
    }
    else
    {
      sec = -1;
      nanosec = 0;
    }
  }

  bool is_valid()
  {
    return sec >= 0;
  }

  bool is_newer_than(const CommandTime& command_time)
  {
    if (sec < 0 || !command_time.is_valid())
      return false;

    if (sec < command_time.sec)
      return true;

    if (sec == command_time.sec && nanosec <= command_time.nanosec)
      return true;

    return false;
  }

};

} // namespace utils
} // namespace free_fleet

#endif // FREEFLEETCLIENT__SRC__DDS_UTILS__COMMANDTIME_HPP
