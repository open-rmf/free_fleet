/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <free_fleet/Console.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class RelocalizationRequest::Implementation
{
public:

  std::string robot_name;

  TaskId task_id;

  Location location;

  std::size_t last_visited_waypoint_index;
};

//==============================================================================
RelocalizationRequest::RelocalizationRequest(
  const std::string& robot_name,
  TaskId task_id,
  const Location& location,
  std::size_t last_visited_waypoint_index)
{
  std::string error_message;

  if (robot_name.empty())
    error_message = "robot_name in RelocalizationRequest must not be empty.";

  if (!error_message.empty())
  {
    fferr << error_message << "\n";
    throw std::invalid_argument(error_message);
  }

  _pimpl = rmf_utils::make_impl<Implementation>(Implementation{
    robot_name,
    task_id,
    location,
    last_visited_waypoint_index});
}

//==============================================================================
const std::string& RelocalizationRequest::robot_name() const
{
  return _pimpl->robot_name;
}

//==============================================================================
TaskId RelocalizationRequest::task_id() const
{
  return _pimpl->task_id;
}

//==============================================================================
const Location& RelocalizationRequest::location() const
{
  return _pimpl->location;
}

//==============================================================================
std::size_t RelocalizationRequest::last_visited_waypoint_index() const
{
  return _pimpl->last_visited_waypoint_index;
}

//==============================================================================
bool operator==(
  const RelocalizationRequest& lhs,
  const RelocalizationRequest& rhs)
{
  if (lhs.robot_name() == rhs.robot_name() &&
    lhs.task_id() == rhs.task_id() &&
    lhs.location() == rhs.location() &&
    lhs.last_visited_waypoint_index() == rhs.last_visited_waypoint_index())
    return true;
  return false;
}

//==============================================================================
bool operator!=(
  const RelocalizationRequest& lhs,
  const RelocalizationRequest& rhs)
{
  return !(lhs == rhs);
}
  
//==============================================================================
} // namespace messages
} // namespace free_fleet
